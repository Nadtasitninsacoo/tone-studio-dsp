/**
 * check-state-echo.js — the bridge's half of "ask the engine what the mix is".
 *
 *   node tools/check-state-echo.js
 *
 * Starts the real `bridge.js` as a child process on spare ports, plays the engine with a UDP
 * socket, and reads what a browser would read from a plain WebSocket. A stand-in that shares the
 * bridge's assumptions cannot test them, so the OSC here is encoded from the spec rather than with
 * the bridge's own encoder.
 */
const assert = require('assert');
const crypto = require('crypto');
const dgram = require('dgram');
const net = require('net');
const path = require('path');
const { spawn } = require('child_process');

const WS = 18080, OUT = 19000, IN = 19001;
let checks = 0;
const ok = (c, m) => { checks += 1; assert.ok(c, m); };
const eq = (a, b, m) => { checks += 1; assert.deepStrictEqual(a, b, m); };
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// ---- OSC, from the spec -------------------------------------------------------------------------
const pad4 = (b) => Buffer.concat([b, Buffer.alloc((4 - (b.length % 4)) % 4)]);
const oscString = (s) => pad4(Buffer.concat([Buffer.from(s, 'utf8'), Buffer.from([0])]));
function osc(address, args) {
  const tags = ',' + args.map((a) => (typeof a === 'string' ? 's' : 'i')).join('');
  const parts = [oscString(address), oscString(tags)];
  for (const a of args) {
    if (typeof a === 'string') parts.push(oscString(a));
    else { const b = Buffer.alloc(4); b.writeInt32BE(a); parts.push(b); }
  }
  return Buffer.concat(parts);
}
function addressOf(packet) {
  const end = packet.indexOf(0);
  return packet.toString('utf8', 0, end);
}

// ---- the fake engine ----------------------------------------------------------------------------
const engineSock = dgram.createSocket('udp4');
let requests = 0;
let behaviour = 'good';
let controls = {};
const send = (buf) => engineSock.send(buf, IN, '127.0.0.1');

function partsOf(entries, maxBytes) {
  const out = [];
  let cur = '';
  for (const [a, v] of entries) {
    const line = `${a}=${v}\n`;
    if (cur.length + line.length > maxBytes && cur) { out.push(cur); cur = ''; }
    cur += line;
  }
  if (cur) out.push(cur);
  return out;
}

engineSock.on('message', async (packet) => {
  if (addressOf(packet) !== '/state/request') return;
  requests += 1;
  const entries = Object.entries(controls).sort(([a], [b]) => (a < b ? -1 : 1));
  const parts = partsOf(entries, 20000);
  const rev = 7;
  if (behaviour === 'silent') return;
  const claimedParts = behaviour === 'missing-part' ? parts.length + 1 : parts.length;
  const claimedCount = behaviour === 'wrong-count' ? entries.length + 1 : entries.length;
  send(osc('/state/begin', [rev, claimedCount, 1, claimedParts]));
  parts.forEach((p, seq) => send(osc('/state/part', [rev, seq, p])));
  send(osc('/state/end', [rev]));
});

// ---- a plain WebSocket client -------------------------------------------------------------------
function connect() {
  return new Promise((resolve, reject) => {
    const sock = net.connect(WS, '127.0.0.1');
    const key = crypto.randomBytes(16).toString('base64');
    const frames = [];
    let buf = Buffer.alloc(0);
    let upgraded = false;
    sock.on('connect', () => sock.write(
      `GET / HTTP/1.1\r\nHost: 127.0.0.1:${WS}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: ${key}\r\nSec-WebSocket-Version: 13\r\n\r\n`));
    sock.on('data', (chunk) => {
      buf = Buffer.concat([buf, chunk]);
      if (!upgraded) {
        const at = buf.indexOf('\r\n\r\n');
        if (at < 0) return;
        if (!buf.toString('utf8', 0, at).startsWith('HTTP/1.1 101')) return reject(new Error('no upgrade'));
        buf = buf.subarray(at + 4);
        upgraded = true;
        resolve({ sock, frames });
      }
      for (;;) {
        if (buf.length < 2) break;
        let len = buf[1] & 0x7f;
        let p = 2;
        if (len === 126) { if (buf.length < 4) break; len = buf.readUInt16BE(2); p = 4; }
        else if (len === 127) { if (buf.length < 10) break; len = Number(buf.readBigUInt64BE(2)); p = 10; }
        if (buf.length < p + len) break;
        try { frames.push(JSON.parse(buf.toString('utf8', p, p + len))); } catch { /* not ours */ }
        buf = buf.subarray(p + len);
      }
    });
    sock.on('error', reject);
  });
}
function wsSend(sock, obj) {
  const payload = Buffer.from(JSON.stringify(obj));
  const mask = crypto.randomBytes(4);
  const masked = Buffer.from(payload.map((b, i) => b ^ mask[i % 4]));
  const header = payload.length < 126 ? Buffer.from([0x81, 0x80 | payload.length]) : Buffer.from([0x81, 0x80 | 126, payload.length >> 8, payload.length & 255]);
  sock.write(Buffer.concat([header, mask, masked]));
}
const stateFrames = (c) => c.frames.filter((f) => f.type === 'state');

(async () => {
  engineSock.bind(OUT, '127.0.0.1');
  await sleep(200);
  const bridge = spawn(process.execPath, [path.join(__dirname, '..', 'bridge.js')], {
    env: { ...process.env, TONE_BRIDGE_WS_PORT: String(WS), TONE_OSC_OUT_PORT: String(OUT), TONE_OSC_IN_PORT: String(IN) },
    stdio: ['ignore', 'pipe', 'pipe'],
  });
  let log = '';
  bridge.stdout.on('data', (d) => { log += d; });
  bridge.stderr.on('data', (d) => { log += d; });
  try {
    for (let i = 0; i < 50 && !/listener bound/.test(log); i += 1) await sleep(100);
    ok(/listener bound/.test(log), 'the bridge started on the spare ports');

    // 1. a small state, asked for by the bridge on connect
    controls = { 'channel/1/fader': -4.5, 'master/gain': -1.5, 'master/fx/delay/time': 380, 'channel/2/pan': 0.3 };
    const a = await connect();
    await sleep(500);
    ok(requests >= 1, 'the bridge asked the engine as soon as a page connected');
    eq(stateFrames(a).length, 1, 'one state frame arrived');
    const f = stateFrames(a)[0];
    eq(f.data.controls, controls, 'every control arrived with its exact value');
    eq([f.data.revision, f.data.count, f.data.saved], [7, 4, true], 'revision, count and saved flag');
    ok(!a.frames.some((x) => x.type === 'meters'), 'a state dump is not mistaken for meters (no meters frame without a meter packet)');

    // 2. a page can ask again, and gets a fresh copy
    const before = requests;
    await sleep(350);
    wsSend(a.sock, { type: 'state-request' });
    await sleep(500);
    ok(requests === before + 1, 'a state-request from the page reaches the engine');
    eq(stateFrames(a).length, 2, 'and the answer reaches the page');

    // 3. a big mix: many parts, none lost, none reordered into the wrong control
    controls = {};
    for (let i = 0; i < 3200; i += 1) controls[`channel/${(i % 32) + 1}/eq/${(i % 4) + 1}/gain/${i}`] = Math.round(((i * 7919) % 2400) - 1200) / 100;
    await sleep(350);
    wsSend(a.sock, { type: 'state-request' });
    await sleep(800);
    const big = stateFrames(a)[2];
    ok(big !== undefined, 'a 3,200-control state arrived');
    eq(Object.keys(big.data.controls).length, 3200, 'all of it');
    eq(big.data.controls, controls, 'and each value is the one that was sent');

    // 4. a state that does not add up is dropped, never forwarded half
    for (const mode of ['missing-part', 'wrong-count']) {
      behaviour = mode;
      controls = { 'channel/1/fader': -3, 'master/gain': 0 };
      const n = stateFrames(a).length;
      await sleep(350);
      wsSend(a.sock, { type: 'state-request' });
      await sleep(700);
      eq(stateFrames(a).length, n, `${mode}: nothing forwarded`);
      ok(/state dump dropped/.test(log), `${mode}: and the bridge said so`);
    }
    behaviour = 'good';

    // 5. an engine that never answers costs nothing
    behaviour = 'silent';
    const n = stateFrames(a).length;
    await sleep(350);
    wsSend(a.sock, { type: 'state-request' });
    await sleep(500);
    eq(stateFrames(a).length, n, 'no answer, no frame');
    ok(bridge.exitCode === null, 'and the bridge is still running');
    behaviour = 'good';

    // 6. a second page asks too, and both are told
    controls = { 'master/gain': -9 };
    await sleep(350);
    const b = await connect();
    await sleep(600);
    eq(stateFrames(b).length >= 1 && stateFrames(b)[0].data.controls, controls, 'a new page gets the engine\'s copy at once');
    ok(stateFrames(a).some((x) => x.data.controls['master/gain'] === -9), 'a page already connected hears it too');

    a.sock.destroy(); b.sock.destroy();
    console.log(`check-state-echo: ${checks} checks passed`);
  } catch (e) {
    console.error('--- bridge log ---\n' + log);
    throw e;
  } finally {
    bridge.kill();
    engineSock.close();
  }
})().catch((e) => { console.error(e); process.exit(1); });
