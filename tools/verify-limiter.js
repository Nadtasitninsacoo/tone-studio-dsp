#!/usr/bin/env node
/**
 * verify-limiter.js — does the engine's master limiter actually hold its ceiling?
 *
 * ===========================================================================
 * WHY THIS IS NOT A "READ THE METER AND COMPARE" SCRIPT
 *
 * The previous attempt printed PASS twice over total silence. That is the failure this
 * file is built around, and it is not a slip anybody makes once: the natural assertion
 * is `maxPeak <= ceiling`, and **silence satisfies it perfectly**. A script that measures
 * nothing and a script that measures a working limiter print the same word.
 *
 * So there are three verdicts here, not two, and the third is the important one:
 *
 *   PASS          the limiter was driven over its ceiling and held it
 *   FAIL          the limiter was driven over its ceiling and did not hold it
 *   INCONCLUSIVE  we never got it over the ceiling, so nothing was tested
 *
 * INCONCLUSIVE is the default. Every path that cannot prove signal reached the limiter
 * ends there — no frames, no bridge, silence, or a signal too quiet to ask the limiter
 * for anything. `PASS` is reachable from exactly one place in this file.
 *
 * ===========================================================================
 * WHY IT RUNS TWO PHASES
 *
 * "Peak stayed under −6" has two causes and they need opposite fixes: the limiter is
 * working, or the meter is not reading the thing the limiter acts on. One phase cannot
 * separate them — that is the open question this test exists to close.
 *
 * So the limiter is switched OFF for a second phase at the same drive. If the peak goes
 * over the ceiling with it off and stays under with it on, both the limiter *and* the
 * meter tap are confirmed at once, and the difference between the two phases is the gain
 * reduction, measured rather than reported by the thing doing it.
 *
 * (The engine deinterleaves the post-limiter master back into `mainBusL/R` before
 * metering — `MixingEngine.cpp`, "post-limiter, see the deinterleave above" — so the tap
 * is *supposed* to be after the limiter. Phase B is what turns that comment into a
 * reading.)
 *
 * ===========================================================================
 * UNITS
 *
 * `bridge.js` converts the master meter from dBFS to linear with `10^(db/20)`, floored at
 * 0 for −120 and **deliberately not clamped at the top** — so a master over full scale
 * arrives above 1.0 and is recoverable. This inverts it exactly. `rta` is left alone
 * because nothing here reads it.
 *
 * Usage:
 *   node verify-limiter.js [--ch 0] [--secs 12] [--url ws://localhost:8080]
 */

'use strict';

/* ------------------------------------------------------------------ config */

const args = process.argv.slice(2);
const arg = (name, fallback) => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 && args[i + 1] !== undefined ? args[i + 1] : fallback;
};

const URL = arg('url', 'ws://localhost:8080');
/** Which mixer channel the instrument is on. 0-based here, as the browser sends it. */
const CHANNEL = Number(arg('ch', '0'));
/** How long to listen per phase. */
const PHASE_SECS = Number(arg('secs', '12'));

/** The ceiling under test. `Limiter::setCeiling` does `min(0, val)`, so this is accepted. */
const CEILING_DB = -6;
/** `MaxTrimDb` in `MixingEngine.h`. Asking for more is silently clamped to this. */
const TRIM_DB = 24;
/** `MaxFaderDb`. The master gain clamp shares it. */
const MASTER_DB = 12;

/** Below this a frame is not programme material, it is a noise floor. */
const SIGNAL_FLOOR_DB = -40;
/** Frames of real signal needed before a phase is allowed to mean anything. */
const MIN_LOUD_FRAMES = 15;
/**
 * How far over the ceiling phase B has to reach before phase A proves anything.
 *
 * This is the guard the silence bug taught, generalised: it is not enough for signal to
 * exist, it has to have been *loud enough to ask the limiter for something*. A guitar
 * peaking at −8 dBFS never touches a −6 ceiling, so "A stayed under" would be true, free,
 * and evidence of nothing.
 */
const OVERDRIVE_MARGIN_DB = 3;
/**
 * Tolerance on the ceiling itself — how far over it a `PASS` may still be given.
 *
 * **The first justification written here was backwards and the first real run disproved
 * it.** It read: "the meter is true-peak (inter-sample), the limiter is a lookahead peak
 * limiter, and they are not obliged to agree to the last tenth." Both halves are the wrong
 * way round. `Limiter.cpp` oversamples 4× and limits at that rate — it is a genuine
 * true-peak limiter. `Metering.cpp` does `peakL = max(peakL, abs(xl))` at base rate under a
 * field named `truePeakDbL` — it is a **sample-peak** meter with a true-peak name.
 *
 * That ordering makes an overshoot *impossible* on paper: true peaks held at the ceiling
 * bound the sample peaks below it. The first run measured **−5.1 dBFS against a −6.0
 * ceiling** anyway, and that 0.9 dB is unexplained. The leading candidate is ringing in the
 * decimation filter on the way back down from 4×, which is a known effect and normally
 * answered by limiting slightly below the target internally — but nobody has measured that
 * here, so it is a hypothesis.
 *
 * The tolerance therefore stays wide enough that a real, small, systematic overshoot does
 * not read as a broken limiter. **What is not allowed is calling that "under the ceiling".**
 * See the PASS text: the number is reported against the ceiling either way.
 */
const CEILING_TOLERANCE_DB = 1.0;
/** Below this difference, phases A and B are the same reading and the limiter did nothing. */
const SAME_READING_DB = 1.0;

/* ------------------------------------------------------------------- utils */

const linToDb = (linear) =>
  typeof linear === 'number' && Number.isFinite(linear) && linear > 0
    ? 20 * Math.log10(linear)
    : -Infinity;

const fmt = (db) => (db === -Infinity ? '  -inf' : db.toFixed(1).padStart(6));
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

let ws = null;
const send = (type, value, index = 0) => {
  ws.send(JSON.stringify({ type, index, value }));
};

/* ------------------------------------------------------- the frame listener */

/** Latest decoded frame. `null` until the bridge has emitted one. */
let latest = null;
let framesSeen = 0;

function onFrame(data) {
  framesSeen += 1;
  const masterDb = Math.max(linToDb(data.masterL), linToDb(data.masterR));
  const ch = data.channels?.[String(CHANNEL)];
  latest = {
    masterDb,
    channelDb: ch ? linToDb(ch.peak) : -Infinity,
    hasChannel: Boolean(ch),
  };
}

/**
 * Listen for `secs`, and return what was seen.
 *
 * Returns counts as well as maxima on purpose: a maximum alone cannot distinguish "loud
 * once" from "loud throughout", and one stray frame is not a test either.
 */
async function collect(label, secs) {
  const started = Date.now();
  const out = {
    frames: 0,
    loudFrames: 0,
    maxMasterDb: -Infinity,
    maxChannelDb: -Infinity,
    overCeilingFrames: 0,
  };
  let lastPrinted = 0;
  let seen = framesSeen;

  while (Date.now() - started < secs * 1000) {
    await sleep(50);
    if (framesSeen === seen || !latest) continue;
    seen = framesSeen;

    out.frames += 1;
    if (latest.masterDb > out.maxMasterDb) out.maxMasterDb = latest.masterDb;
    if (latest.channelDb > out.maxChannelDb) out.maxChannelDb = latest.channelDb;
    if (latest.masterDb > SIGNAL_FLOOR_DB) out.loudFrames += 1;
    if (latest.masterDb > CEILING_DB + CEILING_TOLERANCE_DB) out.overCeilingFrames += 1;

    const now = Date.now();
    if (now - lastPrinted > 400) {
      lastPrinted = now;
      const left = Math.ceil(secs - (now - started) / 1000);
      process.stdout.write(
        `\r  ${label}  master ${fmt(latest.masterDb)} dBFS   ` +
          `ch${CHANNEL} ${fmt(latest.channelDb)} dBFS   ` +
          `peak ${fmt(out.maxMasterDb)}   ${left}s left   `,
      );
    }
  }
  process.stdout.write('\n');
  return out;
}

/* ------------------------------------------------------------------ verdict */

let exitCode = 2; // INCONCLUSIVE unless something below says otherwise.

function inconclusive(reason, detail) {
  console.log('');
  console.log('  ══ INCONCLUSIVE ═══════════════════════════════════════════');
  console.log(`  ${reason}`);
  if (detail) for (const line of detail) console.log(`  ${line}`);
  console.log('');
  console.log('  Nothing was proved about the limiter. This is not a failure —');
  console.log('  it is the script refusing to judge a test it did not get to run.');
  exitCode = 2;
}

function fail(reason, detail) {
  console.log('');
  console.log('  ══ FAIL ═══════════════════════════════════════════════════');
  console.log(`  ${reason}`);
  if (detail) for (const line of detail) console.log(`  ${line}`);
  exitCode = 1;
}

function pass(detail) {
  console.log('');
  console.log('  ══ PASS ═══════════════════════════════════════════════════');
  for (const line of detail) console.log(`  ${line}`);
  exitCode = 0;
}

/* --------------------------------------------------------------------- main */

async function main() {
  console.log('');
  console.log('  Master limiter verification');
  console.log(`  bridge ${URL} · channel ${CHANNEL} · ${PHASE_SECS}s per phase`);
  console.log('');

  /* -- connect ------------------------------------------------------------ */
  ws = new WebSocket(URL);
  const opened = await new Promise((resolve) => {
    const timer = setTimeout(() => resolve(false), 5000);
    ws.onopen = () => {
      clearTimeout(timer);
      resolve(true);
    };
    ws.onerror = () => {
      clearTimeout(timer);
      resolve(false);
    };
  });

  if (!opened) {
    inconclusive(`The bridge did not accept a connection on ${URL}.`, [
      'Start it with `node bridge.js` in the tone-studio-dsp directory.',
    ]);
    return;
  }
  console.log('  · bridge connected');

  ws.onmessage = (event) => {
    let msg;
    try {
      msg = JSON.parse(event.data);
    } catch {
      return;
    }
    if (msg?.type === 'meters' && msg.data) onFrame(msg.data);
  };

  /* -- wait for the engine ------------------------------------------------ */
  // The bridge emits nothing until the engine has actually spoken, so this separates
  // "no bridge" from "bridge up, engine silent" — which have different fixes.
  const waitStart = Date.now();
  while (framesSeen === 0 && Date.now() - waitStart < 8000) await sleep(100);

  if (framesSeen === 0) {
    inconclusive('The bridge is up but has never emitted a meters frame.', [
      'That means the engine has not sent OSC on udp://127.0.0.1:9001.',
      'Check the engine is running and did not print "0 in" for its input device,',
      'and that it did not print "No audio device — withholding meters".',
    ]);
    return;
  }
  console.log(`  · meters flowing (${framesSeen} frames while waiting)`);

  /* -- set the test up ---------------------------------------------------- */
  send('trim', TRIM_DB, CHANNEL);
  send('fader', 0, CHANNEL);
  /**
   * **1 is muted, 0 is open, on both.** Written out because it is not guessable and it is
   * expressed differently at each end — `Main.cpp` has `ch.routedToMain = (value < 0.5f)`
   * for a channel and `master.muted = (value >= 0.5f)` for the master, which land on the
   * same convention through opposite comparisons. Sending 1 here is a script that mutes
   * the desk and then measures the silence it just caused; it would report INCONCLUSIVE
   * honestly and never be capable of anything else.
   */
  send('mute', 0, CHANNEL);
  send('masterMute', 0);
  send('masterGain', MASTER_DB);
  send('limiterCeiling', CEILING_DB);
  send('limiterEnabled', 1);
  await sleep(600); // let the parameter smoothers arrive

  console.log(
    `  · trim +${TRIM_DB} · master +${MASTER_DB} · ceiling ${CEILING_DB} · limiter ON`,
  );
  console.log('');
  console.log('  ─────────────────────────────────────────────────────────');
  console.log('  PHASE A — limiter ON.  PLAY HARD. Dig in. We need it over the ceiling.');
  console.log('  ─────────────────────────────────────────────────────────');
  for (let i = 3; i > 0; i -= 1) {
    process.stdout.write(`\r  starting in ${i}… `);
    await sleep(1000);
  }
  process.stdout.write('\r                     \r');

  const a = await collect('A (limiter ON) ', PHASE_SECS);

  /* -- phase B ------------------------------------------------------------ */
  send('limiterEnabled', 0);
  await sleep(400);

  console.log('');
  console.log('  ─────────────────────────────────────────────────────────');
  console.log('  PHASE B — limiter OFF. Play the SAME way, just as hard.');
  console.log('  ─────────────────────────────────────────────────────────');
  for (let i = 3; i > 0; i -= 1) {
    process.stdout.write(`\r  starting in ${i}… `);
    await sleep(1000);
  }
  process.stdout.write('\r                     \r');

  const b = await collect('B (limiter OFF)', PHASE_SECS);

  /* -- put it back -------------------------------------------------------- */
  // Not the values it had before — nothing here read those, and inventing them would be
  // worse than saying so. These are the engine's own defaults.
  send('limiterEnabled', 1);
  send('limiterCeiling', 0);
  send('masterGain', 0);
  send('trim', 0, CHANNEL);
  await sleep(300);

  /* -- report ------------------------------------------------------------- */
  console.log('');
  console.log('  ─── what was measured ───────────────────────────────────');
  console.log(`                       frames   loud   over ceil   peak dBFS`);
  console.log(
    `    A limiter ON      ${String(a.frames).padStart(6)} ${String(a.loudFrames).padStart(6)}` +
      `   ${String(a.overCeilingFrames).padStart(9)}   ${fmt(a.maxMasterDb)}`,
  );
  console.log(
    `    B limiter OFF     ${String(b.frames).padStart(6)} ${String(b.loudFrames).padStart(6)}` +
      `   ${String(b.overCeilingFrames).padStart(9)}   ${fmt(b.maxMasterDb)}`,
  );
  console.log(
    `    channel ${CHANNEL} peak      ` +
      `A ${fmt(a.maxChannelDb)}   B ${fmt(b.maxChannelDb)}`,
  );
  console.log('');

  /* -- the refusals, in order -------------------------------------------- */

  if (a.frames === 0 || b.frames === 0) {
    inconclusive('The meters stopped during the run.', [
      'The engine went quiet mid-test — check its log for a device drop.',
    ]);
    return;
  }

  if (a.loudFrames < MIN_LOUD_FRAMES || b.loudFrames < MIN_LOUD_FRAMES) {
    const chSaw = Math.max(a.maxChannelDb, b.maxChannelDb) > SIGNAL_FLOOR_DB;
    inconclusive(
      `No signal reached the master bus (needed ${MIN_LOUD_FRAMES} frames above ` +
        `${SIGNAL_FLOOR_DB} dBFS, got A=${a.loudFrames} B=${b.loudFrames}).`,
      chSaw
        ? [
            `But channel ${CHANNEL} DID see signal (peak ${fmt(Math.max(a.maxChannelDb, b.maxChannelDb))} dBFS).`,
            'So the input is fine and the channel is not reaching the main bus —',
            'check routing/mute, the fader, and that the instrument is on this channel.',
          ]
        : [
            `Channel ${CHANNEL} saw nothing either (peak ${fmt(Math.max(a.maxChannelDb, b.maxChannelDb))} dBFS).`,
            'Nothing is arriving at the engine at all. Check the input device it opened',
            `(it prints the channel count at startup), and try --ch <n> for another strip.`,
          ],
    );
    return;
  }

  if (b.maxMasterDb <= CEILING_DB + OVERDRIVE_MARGIN_DB) {
    inconclusive(
      `With the limiter OFF the master only reached ${fmt(b.maxMasterDb)} dBFS, which is ` +
        `not meaningfully over the ${CEILING_DB} dBFS ceiling.`,
      [
        `The limiter was never asked to do anything, so phase A staying under the ceiling`,
        `would have been true for free. Play harder, or raise the drive and run again.`,
        `Needed: over ${CEILING_DB + OVERDRIVE_MARGIN_DB} dBFS with the limiter off.`,
      ],
    );
    return;
  }

  /* -- now, and only now, a real verdict --------------------------------- */

  const reduction = b.maxMasterDb - a.maxMasterDb;

  if (a.maxMasterDb <= CEILING_DB + CEILING_TOLERANCE_DB) {
    /**
     * **Report the number against the ceiling, never the word "under".**
     *
     * This said "under the ceiling" regardless, and the first real run printed it about
     * −5.1 dBFS against a −6.0 ceiling — 0.9 dB *over*. The verdict was defensible (the
     * tolerance exists for exactly that) and the sentence was not: it is a caption
     * describing something other than the number beside it, which is the defect this
     * project keeps correcting in its own readouts. A tolerance is permission to pass, not
     * permission to round the report in the direction that flatters it.
     */
    const overshoot = a.maxMasterDb - CEILING_DB;
    const relation =
      overshoot > 0.05
        ? `${overshoot.toFixed(1)} dB OVER the ${CEILING_DB} dBFS ceiling ` +
          `(inside the ${CEILING_TOLERANCE_DB} dB tolerance, but over it)`
        : `${Math.abs(overshoot).toFixed(1)} dB under the ${CEILING_DB} dBFS ceiling`;

    const lines = [
      `Driven to ${fmt(b.maxMasterDb)} dBFS with the limiter off, the master peaked at`,
      `${fmt(a.maxMasterDb)} dBFS with it on — ${relation}.`,
      '',
      `Measured gain reduction: ${reduction.toFixed(1)} dB.`,
      '',
      'This confirms two things at once, which is why phase B exists:',
      '  · the limiter acts on the master bus, and',
      '  · the meter is tapped AFTER it, so the readout reflects what leaves the engine.',
    ];

    if (overshoot > 0.05) {
      lines.push(
        '',
        'It does NOT confirm the ceiling is being reached exactly, and the overshoot above',
        'is not rounding. `Limiter.cpp` limits at 4× oversampling — a true-peak limiter —',
        'while `Metering.cpp` stores `max(abs(x))` at base rate under the name',
        '`truePeakDbL`, which is a sample peak. True peaks held at the ceiling bound the',
        'sample peaks below it, so on paper this reading cannot happen.',
        '',
        'To find out which end is wrong, run again with a different ceiling:',
        '  · overshoot stays about the same → systematic, most likely decimation-filter',
        '    ringing on the way back down from 4×, answered by limiting a little below',
        '    the target internally;',
        '  · overshoot scales or vanishes → it is specific to this ceiling value, and the',
        '    place to look is `setCeiling` and its conversion to linear.',
      );
    }

    pass(lines);
    return;
  }

  if (Math.abs(reduction) < SAME_READING_DB) {
    fail(
      `The master peaked at ${fmt(a.maxMasterDb)} dBFS with the limiter ON — over the ` +
        `${CEILING_DB} dBFS ceiling — and switching it off changed the reading by only ` +
        `${reduction.toFixed(1)} dB.`,
      [
        'The limiter is not acting on this signal at all. Two candidates:',
        `  · /master/limiter/enabled or /ceiling is not reaching Limiter (check the`,
        `    engine log — the addTokens fix should have made these arrive), or`,
        '  · the meter is tapped upstream of the limiter, so neither phase shows it.',
        '',
        'These are distinguishable by ear: if the sound audibly squashes between the two',
        'phases while the number does not move, it is the tap.',
      ],
    );
    return;
  }

  fail(
    `The limiter IS acting — ${reduction.toFixed(1)} dB of reduction between the phases — ` +
      `but the master still peaked at ${fmt(a.maxMasterDb)} dBFS, over the ` +
      `${CEILING_DB} dBFS ceiling.`,
    [
      'It is working and not reaching its target. Look at the attack: an easing attack',
      'against a lookahead window lets the first transient through, which is exactly the',
      'bug fixed yesterday (a +6 dBFS input left at 1.29 against a 0.891 ceiling).',
      `Overshoot here: ${(a.maxMasterDb - CEILING_DB).toFixed(1)} dB.`,
    ],
  );
}

main()
  .catch((err) => {
    console.error('');
    console.error('  script error:', err?.message ?? err);
    exitCode = 3;
  })
  .finally(async () => {
    await sleep(150);
    try {
      ws?.close();
    } catch {}
    console.log('');
    process.exit(exitCode);
  });
