/**
 * Tone Studio — WebSocket ⇄ OSC gateway between the browser and the C++ DSP engine.
 *
 *   node bridge.js
 *
 * Browser  ⇄  this bridge  ⇄  ToneStudioHeadless.exe
 *   ws://localhost:8080        udp 127.0.0.1:9000  (control, bridge → engine)
 *                              udp 127.0.0.1:9001  (meters,  engine → bridge)
 *
 * ---------------------------------------------------------------------------
 * WHAT CHANGED, AND WHY IT WAS NOT WORKING
 *
 * The previous version was a one-way control plane. Three things followed from that, and
 * all three were invisible from the browser, which is what made them expensive:
 *
 *  1. **It never sent anything back.** There was no `ws.send` in the file and no UDP
 *     receive socket, so `{type:'meters'}` — the only message the web app listens for —
 *     could not arrive. `/feedback` would connect, report itself connected, and show an
 *     empty notch list and no RTA forever.
 *  2. **It silently dropped half the commands it was given.** `geq` and all four
 *     `suppressor-*` messages fell through the if-chain to `address = ''` and were
 *     discarded without a word. The web app had just been fixed so its controls refuse
 *     rather than lie when a command cannot be sent; that failure had simply moved one
 *     layer out, to here, where nothing could see it.
 *  3. **It could not start.** `require('ws')` with no `package.json` and no `node_modules`
 *     throws immediately. This version has **no dependencies at all** — the RFC 6455
 *     handshake and framing are ~80 lines below — so `node bridge.js` works on a clean
 *     checkout with nothing installed.
 *
 * ---------------------------------------------------------------------------
 * THE ONE RULE THIS FILE IS BUILT AROUND: **absent is not zero.**
 *
 * A meters frame is emitted only once the engine has actually sent something, and it
 * carries only the fields that have actually arrived — no `rta` key until an RTA message
 * has been seen, no `feedbackNotches` until a notch message has. Filling them with zeros or
 * empty arrays would be cheaper and it would be a lie: a flat spectrum and an empty notch
 * list are *readings*, and they are exactly what a healthy quiet room looks like. Somebody
 * would have spent a day debugging a suppressor that was never running.
 *
 * The same rule gives `STALE_MS`: if the engine goes quiet the bridge stops emitting
 * frames rather than repeating the last one. A meter frozen at its final value reads as a
 * live signal that happens not to be changing.
 * ------------------------------------------------------------------------- */

const http = require('http');
const crypto = require('crypto');
const dgram = require('dgram');

const WS_PORT = 8080;
const OSC_HOST = '127.0.0.1';
const OSC_OUT_PORT = 9000; // bridge → engine, control
const OSC_IN_PORT = 9001; // engine → bridge, meters

/* ===========================================================================
 * WHO IS ALLOWED TO TALK TO THIS THING
 *
 * This process is a remote control for a PA. Everything below exists because it
 * had no answer to that question at all, and two of the three were confirmed by
 * driving the running bridge rather than by reading it.
 * ======================================================================== */

/**
 * **Loopback by default.** `server.listen(port)` with no host binds `0.0.0.0`, so the bridge
 * was reachable from every machine on the network — anyone on the venue's Wi-Fi could pull a
 * fader, unmute a channel or switch the master limiter off. The two UDP sockets were already
 * bound to `127.0.0.1`; only this one was open.
 *
 * A tablet on the same network controlling the desk is a real thing to want, so it is an
 * opt-in rather than a removal — and it says so loudly at startup, because a bridge listening
 * to the world should never be something somebody discovers later.
 */
const WS_HOST = process.env.TONE_BRIDGE_HOST || '127.0.0.1';

/**
 * **WebSockets are not covered by the same-origin policy.** A page on any site the operator
 * happens to have open can `new WebSocket('ws://localhost:8080')` and drive the engine; the
 * browser will connect and hand over the socket. Nothing here checked, and the connect line
 * *logged* the origin without ever comparing it.
 *
 * Confirmed live: a handshake carrying `Origin: http://evil.example` was accepted, and the
 * bridge's own log recorded it connecting.
 *
 * A missing Origin is allowed on purpose: browsers always send one, so its absence means a
 * non-browser client (the check suite, `curl`, a control surface). Those already have
 * whatever access the machine gives them, and refusing them would break the harness while
 * stopping nobody.
 */
/**
 * **Any port on loopback, and nothing else by default.**
 *
 * The first version listed `http://localhost:3000` and this project's dev server turned out
 * to run on **3001**, so the real app was refused on the first run — a security control that
 * blocks the legitimate user is one that gets switched off, and switched off is worse than
 * never added. Guessing the port is the wrong shape of rule anyway: the threat here is a page
 * served by a *remote* site, and anything served from the operator's own machine already has
 * whatever access the machine gives it.
 *
 * The pattern is fully anchored on purpose. `startsWith('http://localhost:3000')` would also
 * admit `http://localhost:3000.evil.example`, which is a domain an attacker can register.
 */
const LOOPBACK_ORIGIN = /^https?:\/\/(localhost|127\.0\.0\.1|\[::1\])(:\d{1,5})?$/;
const ALLOWED_ORIGINS = (process.env.TONE_BRIDGE_ORIGINS || '')
  .split(',')
  .map((s) => s.trim())
  .filter(Boolean);

/**
 * A frame bigger than this is not a mixer command, and the length is a 64-bit number the
 * *client* chooses. Without a ceiling, one connection announcing a 2^53-byte frame and then
 * dribbling bytes grows `buf` until the process dies — no authentication needed, and nothing
 * in the log to explain it. The web app puts a 64 KB ceiling on its own route handlers for
 * exactly this reason (`lib/httpBody.ts`); this is the same rule on the other side.
 */
const MAX_FRAME_BYTES = 64 * 1024;
/** Ceiling on the reassembly buffer, so a stream of legal-but-endless partial frames cannot. */
const MAX_BUFFER_BYTES = 256 * 1024;
/** One desk does not need a hundred control surfaces. Bounds the meter fan-out too. */
const MAX_CLIENTS = 8;

/** Anchored loopback match, or an exact entry from the operator's list. Never a prefix test. */
function isOriginAllowed(origin) {
  if (origin === undefined || origin === null || origin === '') return true; // not a browser
  return LOOPBACK_ORIGIN.test(origin) || ALLOWED_ORIGINS.includes(origin);
}

/** Stop reporting meters if the engine has said nothing for this long. */
const STALE_MS = 1000;
/** How often to push a meters frame to the browser. */
const PUSH_INTERVAL_MS = 33;

const GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11';

const started = Date.now();
const log = (m) => console.log(`[${((Date.now() - started) / 1000).toFixed(1).padStart(7)}s] ${m}`);

/* ===========================================================================
 * OSC encoding / decoding. Hand-rolled to keep this dependency-free.
 * ======================================================================== */

const pad4 = (buf) => Buffer.concat([buf, Buffer.alloc(4 - (buf.length % 4), 0)]);

/** An OSC message with any number of float32 arguments. */
function encodeOsc(address, floats) {
  const addr = pad4(Buffer.from(address, 'ascii'));
  const tags = pad4(Buffer.from(',' + 'f'.repeat(floats.length), 'ascii'));
  const args = Buffer.alloc(4 * floats.length);
  floats.forEach((f, i) => args.writeFloatBE(Number.isFinite(f) ? f : 0, i * 4));
  return Buffer.concat([addr, tags, args]);
}

function readOscString(buf, offset) {
  let end = offset;
  while (end < buf.length && buf[end] !== 0) end++;
  const str = buf.slice(offset, end).toString('ascii');
  return { str, next: offset + Math.ceil((str.length + 1) / 4) * 4 };
}

/**
 * Decode one OSC message. Returns `null` for anything malformed rather than throwing —
 * a stray UDP packet on this port is not a reason to take the bridge down.
 *
 * Bundles (`#bundle`) are unwrapped recursively, because JUCE's `OSCSender` sends one if
 * you hand it an `OSCBundle`, and a bridge that silently ignored bundles would look
 * identical to an engine that never sent anything.
 */
function decodeOsc(buf, out = []) {
  try {
    if (buf.length < 4) return out;
    if (buf.slice(0, 7).toString('ascii') === '#bundle') {
      let p = 16; // '#bundle\0' (8) + timetag (8)
      while (p + 4 <= buf.length) {
        const size = buf.readInt32BE(p);
        p += 4;
        if (size <= 0 || p + size > buf.length) break;
        decodeOsc(buf.slice(p, p + size), out);
        p += size;
      }
      return out;
    }

    const { str: address, next } = readOscString(buf, 0);
    if (!address.startsWith('/')) return out;
    const { str: tags, next: argStart } = readOscString(buf, next);
    if (!tags.startsWith(',')) return out;

    const args = [];
    let p = argStart;
    for (const t of tags.slice(1)) {
      if (t === 'f') {
        if (p + 4 > buf.length) return out;
        args.push(buf.readFloatBE(p));
        p += 4;
      } else if (t === 'i') {
        if (p + 4 > buf.length) return out;
        args.push(buf.readInt32BE(p));
        p += 4;
      } else if (t === 's') {
        const r = readOscString(buf, p);
        args.push(r.str);
        p = r.next;
      } else {
        return out; // an unknown tag makes every later offset a guess
      }
    }
    out.push({ address, args });
    return out;
  } catch {
    return out;
  }
}

const udpOut = dgram.createSocket('udp4');
function sendOsc(address, floats) {
  const packet = encodeOsc(address, floats);
  udpOut.send(packet, 0, packet.length, OSC_OUT_PORT, OSC_HOST, (err) => {
    if (err) console.error(`OSC send failed for ${address}:`, err.message);
  });
}

/* ===========================================================================
 * Browser → engine.
 * ======================================================================== */

/**
 * Every message type the web app can send, and where it goes. The eight that existed
 * before keep their exact addresses so the current C++ handler is unaffected.
 *
 * A `null` here would be a message we knowingly drop; there are none. Anything not in this
 * table is logged as unknown rather than discarded quietly — that is the whole point.
 */
function addressFor(type, index) {
  switch (type) {
    case 'fader':
      return `/channel/${index + 1}/fader`;
    case 'trim':
      return `/channel/${index + 1}/trim`;
    case 'pan':
      return `/channel/${index + 1}/pan`;
    case 'mute':
      return `/channel/${index + 1}/mute`;
    case 'masterGain':
      return '/master/gain';
    case 'masterMute':
      return '/master/mute';
    case 'limiterEnabled':
      return '/master/limiter/enabled';
    case 'limiterCeiling':
      return '/master/limiter/ceiling';

    // --- Added. These were being dropped. ---
    case 'geq':
      // 31 ISO bands, 0-based from the browser, 1-based on the wire like the channels.
      return `/master/geq/${index + 1}`;
    case 'clear-notches':
      // Dynamic slots only. The engine's 8 fixed slots are frequencies somebody chose to
      // hold down permanently; this page does not own them and must not clear them.
      return '/suppressor/clear-dynamic';
    case 'suppressor-bypass':
      return '/suppressor/bypass';
    case 'suppressor-max-notches':
      return '/suppressor/max-dynamic-notches';
    case 'suppressor-sensitivity':
      // Browser sends 0..1. If you ever see 50 arrive here, the web side has regressed.
      return '/suppressor/sensitivity';

    // --- Master FX Rack ---
    case 'fx-reverb-enabled':
      return '/master/fx/reverb/enabled';
    case 'fx-reverb-room':
      return '/master/fx/reverb/room';
    case 'fx-reverb-damping':
      return '/master/fx/reverb/damping';
    case 'fx-reverb-width':
      return '/master/fx/reverb/width';
    case 'fx-reverb-wet':
      return '/master/fx/reverb/wet';
    case 'fx-delay-enabled':
      return '/master/fx/delay/enabled';
    case 'fx-delay-time':
      return '/master/fx/delay/time';
    case 'fx-delay-feedback':
      return '/master/fx/delay/feedback';
    case 'fx-delay-wet':
      return '/master/fx/delay/wet';
    case 'fx-delay-pingpong':
      return '/master/fx/delay/pingpong';
    case 'fx-delay-hpf':
      return '/master/fx/delay/hpf';
    case 'fx-delay-lpf':
      return '/master/fx/delay/lpf';

    default:
      return null;
  }
}

function handleBrowserMessage(msg) {
  const { type, index, value } = msg;
  const address = addressFor(type, Number(index) || 0);
  if (!address) {
    log(`UNKNOWN command from browser, not forwarded: ${JSON.stringify(msg)}`);
    return;
  }
  if (type === 'suppressor-sensitivity' && (value < 0 || value > 1)) {
    log(`WARNING sensitivity ${value} is outside 0..1 — forwarding anyway, but check the web side`);
  }
  sendOsc(address, [Number(value)]);
}

/* ===========================================================================
 * Engine → browser.
 *
 * Addresses this bridge understands, all floats:
 *
 *   /meter/master              peakDbL, peakDbR        dBFS
 *   /meter/channel/<n>         peakDb, inputDb         dBFS, n is 1-based
 *   /meter/rta                 31 band levels          dBFS  (passed through as dB)
 *   /meter/notch/<slot>        freqHz, gainDb, active  slot 1-based, active 0|1
 *
 * NOTE ON UNITS, and it is not consistent on purpose — this matches what the web app
 * actually consumes:
 *   - `channels[].peak/.input`, `masterL`, `masterR` are **linear 0..1**. `useMixer` writes
 *     them straight into a meter's `peak`, and `peak` is linear everywhere in that app
 *     (`amplitudeToDb` is what converts it). So dB arriving here is converted.
 *   - `rta` is **dBFS**, passed through untouched, because that is what the engine stores
 *     (`Metering.cpp`: `gainToDecibels(maxVal / RtaFftSize, -120.0f)`) and what
 *     `lib/notchPlot.ts` now reads.
 * ======================================================================== */

/**
 * dBFS → the linear 0..1 the web app's meters take.
 *
 * **The top is deliberately not clamped, and it used to be.** `Math.min(1, …)` meant a
 * master 6 dB over full scale arrived as exactly 1.0 — the same value as a master sitting
 * politely at 0 dBFS. An overload was therefore invisible on the one readout whose job is to
 * show it, and it was invisible in the most convincing way: the meter looked pegged and
 * healthy rather than wrong.
 *
 * Found while driving the desk into its limiter on purpose: both the channel and the master
 * read `0.0` exactly, which is not a number real audio produces.
 *
 * Nothing downstream breaks on a value above 1: `amplitudeToMeter` clamps for drawing, and
 * `CLIP_THRESHOLD` (0.99) is a *floor* for lighting the lamp, so a larger number lights it
 * harder rather than not at all. The bottom stays clamped at 0, and −120 dB stays exactly 0,
 * because "nothing" has to be distinguishable from "very quiet".
 */
const dbToLinear = (db) => (db <= -120 ? 0 : Math.max(0, Math.pow(10, db / 20)));

/**
 * The latest thing the engine has said about each quantity. `null` means **never heard**,
 * and stays out of the frame entirely.
 */
const meters = {
  master: null, // { l, r }
  channels: new Map(), // index (0-based, as string) -> { peak, input }
  rta: null, // number[31], dBFS
  notches: new Map(), // slot -> { frequencyHz, gainDb, active }
  lastSeenAt: 0,
};

let sawFirstPacket = false;

function handleOscFromEngine({ address, args }) {
  meters.lastSeenAt = Date.now();
  if (!sawFirstPacket) {
    sawFirstPacket = true;
    log(`first meters packet from the engine (${address}) — meters are now flowing`);
  }

  if (address === '/meter/master' && args.length >= 2) {
    meters.master = { l: dbToLinear(args[0]), r: dbToLinear(args[1]) };
    return;
  }
  if (address === '/meter/rta') {
    if (args.length !== 31) {
      // The web app refuses any length but 31 rather than spreading it across the axis, so
      // say so here where somebody can act on it.
      log(`WARNING /meter/rta carried ${args.length} bands, expected 31 — the page will refuse it`);
    }
    meters.rta = args.slice();
    return;
  }
  const chan = address.match(/^\/meter\/channel\/(\d+)$/);
  if (chan && args.length >= 2) {
    const idx0 = Number(chan[1]) - 1;
    if (idx0 >= 0) {
      meters.channels.set(String(idx0), {
        peak: dbToLinear(args[0]),
        input: dbToLinear(args[1]),
        // Per-channel, for exactly the reason the notches below carry one. The engine
        // decides how many channels it reports (`ReportedChannels` in Main.cpp) and that
        // number can change — it is a constant somebody will edit. Without an expiry the
        // retired channels stayed in the map for ever and the web app went on drawing their
        // last reading as a live meter, which is the frozen-meter failure this whole file
        // is built to avoid. This was the one map that had no expiry while the file
        // carried a long comment explaining why the other one needed it.
        seenAt: Date.now(),
      });
    }
    return;
  }
  const notch = address.match(/^\/meter\/notch\/(\d+)$/);
  if (notch && args.length >= 3) {
    meters.notches.set(Number(notch[1]), {
      frequencyHz: args[0],
      gainDb: args[1],
      active: args[2] >= 0.5,
      // Per-slot, not global. The engine can legitimately change how many slots it reports
      // — it started sending all 16 and now sends only the 8 dynamic ones — and without an
      // expiry the map kept the retired eight for ever. Stale entries in a *list* are worse
      // than in a single reading: the page sizes its grid from the count, so eight dead
      // slots became eight permanent empty cards for filters that no longer exist.
      seenAt: Date.now(),
    });
    return;
  }
  log(`unhandled OSC from engine: ${address} (${args.length} args)`);
}

/**
 * Build the frame, carrying only what has actually been heard.
 * Returns `null` when there is nothing honest to say.
 */
function buildMetersFrame() {
  if (!sawFirstPacket) return null;
  if (Date.now() - meters.lastSeenAt > STALE_MS) return null;
  // `masterL`/`masterR` are non-optional on the web side and get written straight into a
  // meter, so a frame without them would put `undefined` on a fader's peak. Wait for them.
  if (!meters.master) return null;

  const cutoff = Date.now() - STALE_MS;
  for (const [id, c] of meters.channels) {
    if (c.seenAt < cutoff) meters.channels.delete(id);
  }
  for (const [slot, n] of meters.notches) {
    if (n.seenAt < cutoff) meters.notches.delete(slot);
  }

  const data = {
    // `seenAt` is bookkeeping and is not part of the contract the web app reads, so it is
    // stripped rather than sent. `useMixer` writes these straight onto a meter.
    channels: Object.fromEntries(
      [...meters.channels].map(([id, c]) => [id, { peak: c.peak, input: c.input }]),
    ),
    masterL: meters.master.l,
    masterR: meters.master.r,
  };
  if (meters.rta) data.rta = meters.rta;
  if (meters.notches.size > 0) {
    data.feedbackNotches = [...meters.notches.entries()]
      .sort((a, b) => a[0] - b[0])
      .map(([, n]) => ({ frequencyHz: n.frequencyHz, gainDb: n.gainDb, active: n.active }));
  }
  return { type: 'meters', data };
}

const udpIn = dgram.createSocket('udp4');
udpIn.on('message', (packet) => {
  for (const m of decodeOsc(packet)) handleOscFromEngine(m);
});
udpIn.on('error', (err) => {
  console.error('OSC receive socket error:', err.message);
});
udpIn.bind(OSC_IN_PORT, OSC_HOST, () => {
  log(`OSC meters listener bound on udp://${OSC_HOST}:${OSC_IN_PORT}`);
});

/* ===========================================================================
 * WebSocket server, RFC 6455, no dependencies.
 * ======================================================================== */

function encodeWsFrame(text) {
  const payload = Buffer.from(text, 'utf8');
  const len = payload.length;
  let header;
  if (len < 126) {
    header = Buffer.alloc(2);
    header[1] = len;
  } else if (len < 65536) {
    header = Buffer.alloc(4);
    header[1] = 126;
    header.writeUInt16BE(len, 2);
  } else {
    header = Buffer.alloc(10);
    header[1] = 127;
    header.writeBigUInt64BE(BigInt(len), 2);
  }
  header[0] = 0x81; // FIN + text
  return Buffer.concat([header, payload]);
}

/**
 * Pull whole frames from a rolling buffer. Client → server frames are always masked.
 *
 * `onAbort(reason)` is called for anything this refuses to carry on parsing — an
 * over-long frame, or a continuation we have no state for. The caller destroys the socket:
 * there is no way to resynchronise a WebSocket stream once a length has been rejected, and
 * pretending otherwise leaves the parser reading payload as headers.
 */
function decodeWsFrames(buf, onText, onClose, onAbort) {
  let offset = 0;
  for (;;) {
    if (buf.length - offset < 2) break;
    const first = buf[offset];
    const fin = (first & 0x80) !== 0;
    const opcode = first & 0x0f;
    const b1 = buf[offset + 1];
    const masked = (b1 & 0x80) !== 0;
    let len = b1 & 0x7f;
    let p = offset + 2;
    if (len === 126) {
      if (buf.length < p + 2) break;
      len = buf.readUInt16BE(p);
      p += 2;
    } else if (len === 127) {
      if (buf.length < p + 8) break;
      // Read as BigInt and compare there: `Number()` on a 64-bit length loses precision
      // above 2^53 and can produce a value that passes a naive check.
      const big = buf.readBigUInt64BE(p);
      if (big > BigInt(MAX_FRAME_BYTES)) {
        onAbort(`frame length ${big} exceeds ${MAX_FRAME_BYTES}`);
        return Buffer.alloc(0);
      }
      len = Number(big);
      p += 8;
    }
    if (len > MAX_FRAME_BYTES) {
      onAbort(`frame length ${len} exceeds ${MAX_FRAME_BYTES}`);
      return Buffer.alloc(0);
    }
    let mask = null;
    if (masked) {
      if (buf.length < p + 4) break;
      mask = buf.slice(p, p + 4);
      p += 4;
    }
    if (buf.length < p + len) break;
    const payload = buf.slice(p, p + len);
    if (mask) for (let i = 0; i < payload.length; i++) payload[i] ^= mask[i % 4];
    offset = p + len;

    if (opcode === 0x8) {
      onClose();
      return buf.slice(offset);
    }
    // A ping must be answered or well-behaved clients eventually decide we are dead and
    // drop the connection mid-show. Echo the payload, as RFC 6455 requires.
    if (opcode === 0x9) {
      onText(null, payload);
      continue;
    }
    if (opcode === 0xa) continue; // pong; nothing to do
    if (opcode === 0x1) {
      // Fragmented text (FIN clear, continuations to follow) was silently treated as a
      // complete message, so the first fragment was handed to JSON.parse and logged as
      // unparseable. Nothing this app sends is anywhere near a fragmentation threshold, so
      // refusing is honest where half-parsing was not.
      if (!fin) {
        onAbort('fragmented text frames are not supported');
        return Buffer.alloc(0);
      }
      onText(payload.toString('utf8'), null);
    }
  }
  return buf.slice(offset);
}

/** A pong frame carrying the ping's payload. Server → client frames are never masked. */
function encodeWsPong(payload) {
  const header = Buffer.alloc(2);
  header[0] = 0x8a; // FIN + pong
  header[1] = Math.min(payload.length, 125);
  return Buffer.concat([header, payload.slice(0, 125)]);
}

const clients = new Set();

const server = http.createServer((_req, res) => {
  res.writeHead(426, { 'Content-Type': 'text/plain' });
  res.end('Tone Studio DSP bridge. Connect over WebSocket.\n');
});

server.on('upgrade', (req, socket) => {
  const key = req.headers['sec-websocket-key'];
  if (!key) {
    socket.destroy();
    return;
  }

  const origin = req.headers.origin;
  if (!isOriginAllowed(origin)) {
    // 403 before the upgrade, so the browser reports a refused WebSocket rather than a
    // connection that opens and then goes quiet. Logged, because a legitimate deployment
    // on another port lands here and the operator needs to know which value to allow.
    log(`REFUSED upgrade from origin ${origin} — add it to TONE_BRIDGE_ORIGINS to allow`);
    socket.write('HTTP/1.1 403 Forbidden\r\nConnection: close\r\n\r\n');
    socket.destroy();
    return;
  }

  if (clients.size >= MAX_CLIENTS) {
    log(`REFUSED upgrade: ${MAX_CLIENTS} clients already connected`);
    socket.write('HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\n');
    socket.destroy();
    return;
  }

  socket.write(
    'HTTP/1.1 101 Switching Protocols\r\n' +
      'Upgrade: websocket\r\n' +
      'Connection: Upgrade\r\n' +
      `Sec-WebSocket-Accept: ${crypto.createHash('sha1').update(key + GUID).digest('base64')}\r\n\r\n`,
  );
  socket.setNoDelay(true);
  clients.add(socket);
  log(`web client connected (${clients.size} open) from ${req.headers.origin || 'unknown origin'}`);
  if (!sawFirstPacket) {
    log('  ...but the engine has never sent a meters packet, so this client will see none');
  }

  let buf = Buffer.alloc(0);
  const abort = (reason) => {
    log(`dropping client: ${reason}`);
    socket.destroy();
    buf = Buffer.alloc(0);
  };

  socket.on('data', (chunk) => {
    if (socket.destroyed) return;
    buf = Buffer.concat([buf, chunk]);
    // Checked *after* the append and before parsing: a client that never completes a frame
    // grows this for ever, which is the shape of the denial-of-service the frame ceiling
    // alone does not close.
    if (buf.length > MAX_BUFFER_BYTES) {
      abort(`unparsed buffer exceeded ${MAX_BUFFER_BYTES} bytes`);
      return;
    }
    buf = decodeWsFrames(
      buf,
      (text, pingPayload) => {
        if (pingPayload) {
          if (socket.writable) socket.write(encodeWsPong(pingPayload));
          return;
        }
        try {
          handleBrowserMessage(JSON.parse(text));
        } catch {
          log(`unparseable frame from browser: ${text.slice(0, 120)}`);
        }
      },
      () => socket.end(),
      abort,
    );
  });

  const drop = () => {
    if (clients.delete(socket)) log(`web client gone (${clients.size} open)`);
  };
  socket.on('close', drop);
  socket.on('error', drop);
});

let wasStale = true;
setInterval(() => {
  const frame = buildMetersFrame();
  if (!frame) {
    if (sawFirstPacket && !wasStale) {
      wasStale = true;
      log(`engine silent for ${STALE_MS}ms — meters withheld rather than frozen`);
    }
    return;
  }
  if (wasStale) {
    wasStale = false;
    log('meters resumed');
  }
  if (clients.size === 0) return;
  const encoded = encodeWsFrame(JSON.stringify(frame));
  for (const c of clients) if (c.writable) c.write(encoded);
}, PUSH_INTERVAL_MS);

server.listen(WS_PORT, WS_HOST, () => {
  log('='.repeat(62));
  log('  Tone Studio WS ⇄ OSC Gateway Bridge');
  log('='.repeat(62));
  log(`  browser  ws://${WS_HOST}:${WS_PORT}`);
  log(`  control  → udp://${OSC_HOST}:${OSC_OUT_PORT}`);
  log(`  meters   ← udp://${OSC_HOST}:${OSC_IN_PORT}`);
  log(`  origins  any loopback port${ALLOWED_ORIGINS.length ? ', plus ' + ALLOWED_ORIGINS.join(', ') : ''}`);
  if (WS_HOST !== '127.0.0.1' && WS_HOST !== 'localhost') {
    // Never silent. This process can pull a fader on a live PA; who can reach it is not
    // something anybody should have to discover from a port scan.
    log('  !! LISTENING BEYOND LOOPBACK — anyone who can reach this host can drive the desk');
  }
  log('='.repeat(62));
  log('The engine must SEND meters to the port above. If it does not, this bridge');
  log('reports nothing rather than reporting zeros — see the header of this file.');
});
