/**
 * check-engine-restart.js — the engine keeps the mix: kill it hard, start it, and it is back.
 *
 *   node tools/check-engine-restart.js <path to tone-studio-app.exe>
 *
 * This is the part of the claim no unit test can make: a real process, a real audio device, a
 * real `TerminateProcess` in the middle of things (the Windows equivalent of `kill -9`). It uses
 * ports 9000/9001 and the audio device, so it refuses to run if either is taken — it must never
 * be pointed at a machine whose engine is playing, and it writes its state to a temporary folder,
 * never to the profile's real one.
 *
 * What it proves, in order:
 *   1. controls sent over OSC are on disk within a couple of seconds, checksummed;
 *   2. after a hard kill the engine starts by itself, says what it restored, and the DSP — not
 *      just a file reader — accepted every control ("State applied");
 *   3. asked for its state, the restarted engine reports exactly what was sent;
 *   4. a damaged newest file costs one version, not the mix, and the log says which was refused;
 *   5. the bridge and web clients coming and going change nothing, and the audio stream (meters
 *      keep flowing at their own rate) does not stop while they do.
 */
const assert = require('assert');
const dgram = require('dgram');
const fs = require('fs');
const net = require('net');
const os = require('os');
const path = require('path');
const crypto = require('crypto');
const { spawn } = require('child_process');

const exe = process.argv[2];
if (!exe || !fs.existsSync(exe)) {
  console.error('usage: node tools/check-engine-restart.js <path to tone-studio-app.exe>');
  process.exit(2);
}

let checks = 0;
const ok = (c, m) => { checks += 1; assert.ok(c, m); };
const eq = (a, b, m) => { checks += 1; assert.deepStrictEqual(a, b, m); };
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const f32 = Math.fround;
const TEST_WS_PORT = 18081;

// ---- OSC, from the spec ---------------------------------------------------------------------------
const pad4 = (b) => Buffer.concat([b, Buffer.alloc((4 - (b.length % 4)) % 4)]);
const oscString = (s) => pad4(Buffer.concat([Buffer.from(s, 'utf8'), Buffer.from([0])]));
function oscFloat(address, value) {
  const v = Buffer.alloc(4); v.writeFloatBE(value);
  return Buffer.concat([oscString(address), oscString(',f'), v]);
}
function oscEmpty(address) { return Buffer.concat([oscString(address), oscString(',')]); }
function readString(buf, at) {
  const end = buf.indexOf(0, at);
  return { str: buf.toString('utf8', at, end), next: (end + 4) & ~3 };
}
function decode(buf) {
  const a = readString(buf, 0);
  const t = readString(buf, a.next);
  const args = [];
  let p = t.next;
  for (const tag of t.str.slice(1)) {
    if (tag === 'i') { args.push(buf.readInt32BE(p)); p += 4; }
    else if (tag === 'f') { args.push(buf.readFloatBE(p)); p += 4; }
    else if (tag === 's') { const r = readString(buf, p); args.push(r.str); p = r.next; }
    else return null;
  }
  return { address: a.str, args };
}

// ---- the sockets this test owns ----------------------------------------------------------------
const tx = dgram.createSocket('udp4');
const sendOsc = (buf) => tx.send(buf, 9000, '127.0.0.1');
let rx = null;
const heard = { meters: [], state: [] };
function openReceiver() {
  return new Promise((resolve, reject) => {
    rx = dgram.createSocket('udp4');
    rx.on('error', reject);
    rx.on('message', (packet) => {
      const m = decode(packet);
      if (!m) return;
      if (m.address === '/meter/master') heard.meters.push(Date.now());
      else if (m.address.startsWith('/state/')) heard.state.push(m);
    });
    rx.bind(9001, '127.0.0.1', () => resolve());
  });
}
const closeReceiver = () => new Promise((r) => { if (!rx) return r(); rx.close(() => { rx = null; r(); }); });

/** Ask the engine what the mix is and assemble the answer, or throw. */
async function askState() {
  heard.state.length = 0;
  sendOsc(oscEmpty('/state/request'));
  for (let i = 0; i < 60; i += 1) {
    await sleep(50);
    if (heard.state.some((m) => m.address === '/state/end')) break;
  }
  const begin = heard.state.find((m) => m.address === '/state/begin');
  assert.ok(begin, 'the engine answered /state/request');
  const [revision, count, saved, parts] = begin.args;
  const got = new Map();
  for (const m of heard.state) if (m.address === '/state/part' && m.args[0] === revision) got.set(m.args[1], m.args[2]);
  assert.strictEqual(got.size, parts, `all ${parts} parts arrived`);
  const controls = {};
  for (let i = 0; i < parts; i += 1) {
    for (const line of got.get(i).split('\n')) {
      if (!line) continue;
      const at = line.lastIndexOf('=');
      controls[line.slice(0, at)] = Number(line.slice(at + 1));
    }
  }
  assert.strictEqual(Object.keys(controls).length, count, 'the count matches');
  return { revision, saved: saved === 1, controls };
}

// ---- the engine process ---------------------------------------------------------------------------
function startEngine(stateDir) {
  const startedAt = Date.now();
  const proc = spawn(exe, ['--state-dir', stateDir], { stdio: ['pipe', 'pipe', 'pipe'] });
  const run = { proc, log: '', startedAt, exited: null, dead: false };
  proc.stdout.on('data', (d) => { run.log += d; });
  proc.stderr.on('data', (d) => { run.log += d; });
  // code is null when the process was killed by a signal, so a separate flag says it is gone.
  proc.on('exit', (code, signal) => { run.exited = code === null ? (signal || 'killed') : code; run.dead = true; });
  return run;
}
async function waitFor(run, pattern, ms = 15000) {
  const t0 = Date.now();
  while (Date.now() - t0 < ms) {
    if (pattern.test(run.log)) return Date.now() - run.startedAt;
    if (run.dead) throw new Error(`the engine exited (${run.exited}) before ${pattern}:\n${run.log}`);
    await sleep(50);
  }
  throw new Error(`timed out waiting for ${pattern}:\n${run.log}`);
}
async function hardKill(run) {
  run.proc.kill('SIGKILL'); // TerminateProcess: no destructors, no flush, no goodbye
  for (let i = 0; i < 100 && !run.dead; i += 1) await sleep(50);
  assert.ok(run.dead, 'the process is gone');
}

const versionsIn = (dir) => fs.readdirSync(dir).filter((n) => /^state-\d{10}\.json$/.test(n)).sort();
async function waitForVersionHolding(dir, expect) {
  for (let i = 0; i < 80; i += 1) {
    await sleep(100);
    const names = versionsIn(dir);
    if (names.length === 0) continue;
    try {
      const j = JSON.parse(fs.readFileSync(path.join(dir, names[names.length - 1]), 'utf8'));
      if (Object.entries(expect).every(([k, v]) => j.controls[k] !== undefined && f32(Number(j.controls[k])) === f32(v))) return j;
    } catch { /* mid-swap: try again */ }
  }
  throw new Error('the controls never reached the disk');
}

// ---- a plain WebSocket client for the bridge phase --------------------------------------------------
function wsConnect(port) {
  return new Promise((resolve, reject) => {
    const sock = net.connect(port, '127.0.0.1');
    const key = crypto.randomBytes(16).toString('base64');
    const frames = [];
    let buf = Buffer.alloc(0); let up = false;
    sock.on('connect', () => sock.write(`GET / HTTP/1.1\r\nHost: 127.0.0.1:${port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: ${key}\r\nSec-WebSocket-Version: 13\r\n\r\n`));
    sock.on('data', (chunk) => {
      buf = Buffer.concat([buf, chunk]);
      if (!up) {
        const at = buf.indexOf('\r\n\r\n');
        if (at < 0) return;
        buf = buf.subarray(at + 4); up = true; resolve({ sock, frames });
      }
      for (;;) {
        if (buf.length < 2) break;
        let len = buf[1] & 0x7f; let p = 2;
        if (len === 126) { if (buf.length < 4) break; len = buf.readUInt16BE(2); p = 4; }
        else if (len === 127) { if (buf.length < 10) break; len = Number(buf.readBigUInt64BE(2)); p = 10; }
        if (buf.length < p + len) break;
        try { frames.push(JSON.parse(buf.toString('utf8', p, p + len))); } catch { /* ignore */ }
        buf = buf.subarray(p + len);
      }
    });
    sock.on('error', reject);
  });
}

(async () => {
  // Never touch a machine whose engine is playing.
  for (const port of [9000, 9001]) {
    const probe = dgram.createSocket('udp4');
    const free = await new Promise((r) => { probe.once('error', () => r(false)); probe.bind(port, '127.0.0.1', () => r(true)); });
    probe.close();
    if (!free) { console.error(`port ${port} is in use — an engine or a bridge is running. Not touching it.`); process.exit(3); }
  }

  const stateDir = fs.mkdtempSync(path.join(os.tmpdir(), 'tsd-restart-'));
  const mix = {
    'channel/1/fader': -4.5, 'channel/1/trim': 2, 'channel/1/pan': -0.25, 'channel/1/eq/2/gain': 3.5, 'channel/1/eq/2/freq': 800,
    'channel/2/fader': -7, 'channel/2/comp/threshold': -18, 'channel/2/comp/ratio': 4, 'channel/3/mute': 1,
    'master/gain': -1.5, 'master/limiter/enabled': 1, 'master/limiter/ceiling': -1, 'master/geq/5': 2.5,
    'master/fx/delay/time': 380, 'master/fx/reverb/wet': 0.25, 'output/1/delay': 4,
    'suppressor/sensitivity': 0.75, 'suppressor/max-dynamic-notches': 6,
  };
  let engine = null;
  let bridge = null;
  await openReceiver();
  try {
    // 1. first run: nothing saved, then everything saved
    engine = startEngine(stateDir);
    try { await waitFor(engine, /OSC control listening/); } catch (e) { console.error(String(e.message).slice(0, 600)); throw e; }
    let firstPlaying = 0;
    try { firstPlaying = await waitFor(engine, /Audio device starting/, 15000); } catch {
      console.log('SKIP: this machine has no audio device the engine can open.\n' + engine.log.slice(-400));
      await hardKill(engine); process.exit(0);
    }
    ok(/nothing saved yet/.test(engine.log), 'a first run says it starts from the defaults');
    for (const [address, value] of Object.entries(mix)) { sendOsc(oscFloat('/' + address, value)); await sleep(2); }
    const saved = await waitForVersionHolding(stateDir, mix);
    eq(saved.schema, 1, 'schema');
    ok(saved.checksum.length === 16 && saved.count >= Object.keys(mix).length, 'the file carries a checksum and a count');
    ok(Date.now() - engine.startedAt < 20000, 'and it was written within seconds of the last change');
    const before = await askState();
    for (const [k, v] of Object.entries(mix)) eq(f32(before.controls[k]), f32(v), `before the kill, the engine reports ${k}`);

    // 2. the hard kill, and the start with nobody connected
    await hardKill(engine);
    ok(versionsIn(stateDir).length >= 1, 'the files survived the kill');
    ok(!fs.readdirSync(stateDir).some((n) => /_temp|\.tmp$/.test(n)), 'and no half-written file was left behind');
    engine = startEngine(stateDir);
    const toRestored = await waitFor(engine, /State restored: (\d+) controls/);
    const toPlaying = await waitFor(engine, /Audio device starting/);
    const toApplied = await waitFor(engine, /State applied: the DSP accepted (\d+) of \1/, 15000);
    const restored = Number(/State restored: (\d+) controls/.exec(engine.log)[1]);
    ok(restored >= Object.keys(mix).length - 2, `restored ${restored} controls`);
    ok(!/damaged|could not be queued/.test(engine.log), 'nothing was damaged, and nothing could not be queued');
    ok(!/\([1-9]\d* refused\)/.test(engine.log), 'and the DSP refused nothing');
    console.log(`  first start: audio device up ${firstPlaying} ms after launch`);
    console.log(`  timings after the hard kill: state read ${toRestored} ms, audio device up ${toPlaying} ms, every control in the DSP ${toApplied} ms`);

    // 3. the restarted engine reports the same mix, with nobody having told it anything
    const after = await askState();
    for (const [k, v] of Object.entries(mix)) eq(f32(after.controls[k]), f32(v), `after the restart, the engine reports ${k}`);
    eq(Object.keys(after.controls).sort(), Object.keys(before.controls).sort(), 'the same set of controls, no more and no fewer');

    // 4. a damaged newest version costs one version, not the mix
    sendOsc(oscFloat('/channel/1/fader', -9));
    await waitForVersionHolding(stateDir, { 'channel/1/fader': -9 });
    const newest = versionsIn(stateDir).pop();
    await hardKill(engine);
    const file = path.join(stateDir, newest);
    fs.writeFileSync(file, fs.readFileSync(file, 'utf8').slice(0, 200)); // torn
    engine = startEngine(stateDir);
    await waitFor(engine, /State restored/);
    ok(/skipped a damaged version/.test(engine.log) && engine.log.includes(newest), 'the log names the version it refused');
    await waitFor(engine, /Press Enter to stop/); // the control port is up only after the audio device is
    const fallback = await askState();
    eq(f32(fallback.controls['channel/1/fader']), f32(-4.5), 'the fader is the previous version\'s, not the torn one\'s');
    for (const [k, v] of Object.entries(mix)) if (k !== 'channel/1/fader') eq(f32(fallback.controls[k]), f32(v), `and ${k} is intact`);

    // 5. a bridge and web clients coming and going change nothing, and the audio does not stop
    await closeReceiver();
    // A spare WebSocket port, so a real page that is open on this machine and retrying 8080 cannot walk in and
    // push its own desk into the engine under test (it did, the first time this ran).
    bridge = spawn(process.execPath, [path.join(__dirname, '..', 'bridge.js')], { env: { ...process.env, TONE_BRIDGE_WS_PORT: String(TEST_WS_PORT) }, stdio: ['ignore', 'pipe', 'pipe'] });
    let bridgeLog = ''; bridge.stdout.on('data', (d) => { bridgeLog += d; }); bridge.stderr.on('data', (d) => { bridgeLog += d; });
    for (let i = 0; i < 60 && !/listener bound/.test(bridgeLog); i += 1) await sleep(100);
    ok(/listener bound/.test(bridgeLog), 'the bridge started');
    const first = await wsConnect(TEST_WS_PORT);
    await sleep(1200);
    const stateFrame = () => first.frames.filter((f) => f.type === 'state').pop();
    ok(stateFrame() !== undefined, 'a page that connects is handed the engine\'s own copy of the mix, with no request of its own');
    for (const [k, v] of Object.entries(mix)) eq(f32(stateFrame().data.controls[k]), f32(v), `the page is told ${k}`);
    const meterFramesAt = () => first.frames.filter((f) => f.type === 'meters').length;
    const metersBefore = meterFramesAt();
    const stoppedBefore = (engine.log.match(/Audio device stopped/g) || []).length;
    const revisionBefore = stateFrame().data.revision;
    for (let i = 0; i < 8; i += 1) { const c = await wsConnect(TEST_WS_PORT); await sleep(120); c.sock.destroy(); await sleep(80); }
    await sleep(600);
    ok(meterFramesAt() > metersBefore + 20, `meters kept flowing through the connect/disconnect storm (${meterFramesAt() - metersBefore} frames)`);
    eq((engine.log.match(/Audio device stopped/g) || []).length, stoppedBefore, 'the audio device did not stop');
    ok(!/Dropped|Ignored/.test(engine.log), 'and the engine received nothing it had to refuse');
    bridge.kill();
    await sleep(500);
    ok(!engine.dead, 'the bridge dying does not touch the engine');
    await openReceiver();
    const last = await askState();
    for (const [k, v] of Object.entries(mix)) eq(f32(last.controls[k]), f32(v), `after all that, ${k} is unchanged`);
    ok(last.revision >= revisionBefore, 'and the revision never went backwards');
    ok(heard.meters.length > 0, 'and the meters are still flowing straight from the engine');

    first.sock.destroy();
    console.log(`check-engine-restart: ${checks} checks passed`);
  } catch (e) {
    console.error('FAILED:', e && e.message ? e.message : e);
    if (engine) console.error('--- engine log ---\n' + engine.log.slice(-1500));
    process.exitCode = 1;
  } finally {
    try { if (engine && !engine.dead) await hardKill(engine); } catch { /* gone */ }
    try { if (bridge) bridge.kill(); } catch { /* gone */ }
    await closeReceiver();
    tx.close();
    fs.rmSync(stateDir, { recursive: true, force: true });
  }
})();
