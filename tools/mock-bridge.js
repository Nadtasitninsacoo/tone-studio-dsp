#!/usr/bin/env node
/**
 * mock-bridge.js — a stand-in for `bridge.js`, so `verify-limiter.js`'s verdicts can be
 * tested without an engine, a guitar or a human.
 *
 * This exists for one reason: the script it tests printed PASS over silence twice
 * yesterday. A refusal that has never been watched refusing is exactly as trustworthy as
 * the assertion that was wrong, so each scenario below drives the verifier into one of its
 * verdicts and the runner checks it landed there.
 *
 * **The frame format is copied from the real `bridge.js`, not invented**, because a mock
 * that shares the reader's assumption cannot test the reader's assumption — this project
 * has already been bitten by that once, when a mock sent RTA as 0..1 linear to a page that
 * had wrongly assumed 0..1 linear. The two things copied verbatim, with their source:
 *
 *   bridge.js:347  const dbToLinear = (db) => (db <= -120 ? 0 : Math.max(0, 10 ** (db/20)))
 *   bridge.js:454  { type: 'meters', data: { channels, masterL, masterR } }
 *
 * Usage:  node mock-bridge.js <scenario> [port]
 */

'use strict';

const http = require('http');
const crypto = require('crypto');

const SCENARIO = process.argv[2] ?? 'working';
const PORT = Number(process.argv[3] ?? 8080);

/* Copied from bridge.js:347. See the header. */
const dbToLinear = (db) => (db <= -120 ? 0 : Math.max(0, Math.pow(10, db / 20)));

/**
 * What the master reads, per scenario, as a function of whether the limiter is on.
 *
 * `channelDb` is what the strip meter shows, and it is deliberately independent: the
 * "signal at the channel but not the master" case is a real routing failure the verifier
 * is supposed to name separately, so it needs to be reachable here.
 */
/**
 * `expectText` is not decoration.
 *
 * Three of these scenarios are defended by **two** guards, so removing one leaves the exit
 * code green while the *reason printed* becomes wrong — `unrouted` would be reported as
 * "play harder", sending somebody to turn up a channel that is not reaching the bus. An
 * exit-code-only check cannot see that, and a wrong diagnosis on a page-sized wall of
 * numbers is the failure this project keeps writing down.
 */
const SCENARIOS = {
  /** The bug that started this: nothing at all. Must never be PASS. */
  silence: {
    on: -Infinity, off: -Infinity, channelDb: -Infinity, expect: 2,
    expectText: 'Nothing is arriving at the engine',
  },
  /** Signal, but far too quiet to ask the limiter for anything. */
  quiet: {
    on: -25, off: -25, channelDb: -22, expect: 2,
    expectText: 'not meaningfully over',
  },
  /** Signal at the strip, nothing on the master — an unrouted channel, not a quiet one. */
  unrouted: {
    on: -Infinity, off: -Infinity, channelDb: -8, expect: 2,
    expectText: 'not reaching the main bus',
  },
  /** The limiter holding, and the meter seeing it hold. The only PASS. */
  working: {
    on: -6.3, off: 2.0, channelDb: -4, expect: 0,
    expectText: 'tapped AFTER it',
  },
  /** Driven hard, and the reading does not move: not acting, or tapped upstream. */
  broken: {
    on: 2.0, off: 2.1, channelDb: -4, expect: 1,
    expectText: 'not acting on this signal at all',
  },
  /** Acting, but letting a transient through: yesterday's real Limiter bug. */
  overshoot: {
    on: -2.0, off: 2.0, channelDb: -4, expect: 1,
    expectText: 'IS acting',
  },
};

const scenario = SCENARIOS[SCENARIO];
if (!scenario) {
  console.error(`unknown scenario "${SCENARIO}" — have: ${Object.keys(SCENARIOS).join(', ')}`);
  process.exit(3);
}

/** Flipped by the `limiterEnabled` command the verifier sends between its phases. */
let limiterOn = true;

/* --------------------------------------------------------- minimal WS server */

const GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11';

function frameText(text) {
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

/** Only what this mock needs: unfragmented masked text frames from one client. */
function readFrames(buf) {
  const out = [];
  let off = 0;
  while (off + 2 <= buf.length) {
    const opcode = buf[off] & 0x0f;
    const masked = (buf[off + 1] & 0x80) !== 0;
    let len = buf[off + 1] & 0x7f;
    let pos = off + 2;
    if (len === 126) {
      len = buf.readUInt16BE(pos);
      pos += 2;
    } else if (len === 127) {
      len = Number(buf.readBigUInt64BE(pos));
      pos += 8;
    }
    let mask = null;
    if (masked) {
      mask = buf.subarray(pos, pos + 4);
      pos += 4;
    }
    if (pos + len > buf.length) break;
    const payload = Buffer.from(buf.subarray(pos, pos + len));
    if (mask) for (let i = 0; i < payload.length; i += 1) payload[i] ^= mask[i % 4];
    if (opcode === 0x1) out.push(payload.toString('utf8'));
    off = pos + len;
  }
  return { messages: out, consumed: off };
}

const server = http.createServer();

server.on('upgrade', (req, socket) => {
  const key = req.headers['sec-websocket-key'];
  socket.write(
    'HTTP/1.1 101 Switching Protocols\r\n' +
      'Upgrade: websocket\r\n' +
      'Connection: Upgrade\r\n' +
      `Sec-WebSocket-Accept: ${crypto
        .createHash('sha1')
        .update(key + GUID)
        .digest('base64')}\r\n\r\n`,
  );

  let pending = Buffer.alloc(0);
  socket.on('data', (chunk) => {
    pending = Buffer.concat([pending, chunk]);
    const { messages, consumed } = readFrames(pending);
    pending = pending.subarray(consumed);
    for (const text of messages) {
      let msg;
      try {
        msg = JSON.parse(text);
      } catch {
        continue;
      }
      if (msg.type === 'limiterEnabled') {
        limiterOn = Number(msg.value) >= 0.5;
        console.error(`[mock] limiter ${limiterOn ? 'ON' : 'OFF'}`);
      }
    }
  });

  // 30 Hz, the same rate the engine's meter timer runs at.
  const timer = setInterval(() => {
    const masterDb = limiterOn ? scenario.on : scenario.off;
    // A little jitter, so a maximum means something and the verifier is not reading one
    // constant repeated — a flat signal would let a broken max-tracker pass.
    const jitter = (n) => (n === -Infinity ? -Infinity : n - Math.random() * 1.5);
    const payload = {
      type: 'meters',
      data: {
        channels: { 0: { peak: dbToLinear(jitter(scenario.channelDb)), input: 0 } },
        masterL: dbToLinear(jitter(masterDb)),
        masterR: dbToLinear(jitter(masterDb)),
      },
    };
    try {
      socket.write(frameText(JSON.stringify(payload)));
    } catch {
      clearInterval(timer);
    }
  }, 33);

  socket.on('close', () => clearInterval(timer));
  socket.on('error', () => clearInterval(timer));
});

/**
 * Only when run directly.
 *
 * `check-verifier.js` requires this file for `SCENARIOS`, and without the guard that
 * *started a server on 8080* as a side effect of reading a table — which would fight a
 * real `bridge.js` for the port on the machine where this is most likely to be used.
 */
if (require.main === module) {
  server.listen(PORT, '127.0.0.1', () => {
    console.error(
      `[mock] scenario "${SCENARIO}" on ws://127.0.0.1:${PORT} (expect exit ${scenario.expect})`,
    );
  });
}

module.exports = { SCENARIOS };
