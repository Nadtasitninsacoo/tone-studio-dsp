#!/usr/bin/env node
/**
 * check-verifier.js — does `verify-limiter.js` actually refuse?
 *
 * The verifier's whole value is the verdict it declines to give. That is not something to
 * take on trust from reading it: the version written yesterday looked correct and printed
 * PASS over silence twice. So each scenario is driven end to end against `mock-bridge.js`
 * and the exit code is asserted.
 *
 * Exit codes under test:  0 PASS · 1 FAIL · 2 INCONCLUSIVE · 3 script error
 *
 * The one that matters most is `silence` → 2. If that ever goes to 0, the verifier has
 * regressed into the thing it was written to replace.
 */

'use strict';

const { spawn } = require('child_process');
const path = require('path');

const HERE = __dirname;
const { SCENARIOS } = require(path.join(HERE, 'mock-bridge.js'));

const PORT = 8099; // not 8080: a real bridge may be running while this is checked
const PHASE_SECS = 2;

const run = (scenario) =>
  new Promise((resolve) => {
    const mock = spawn(process.execPath, [path.join(HERE, 'mock-bridge.js'), scenario, String(PORT)], {
      stdio: ['ignore', 'ignore', 'pipe'],
    });

    let done = false;
    const finish = (code, output) => {
      if (done) return;
      done = true;
      try {
        mock.kill();
      } catch {}
      resolve({ code, output });
    };

    // Give the mock a moment to bind before the verifier dials it.
    setTimeout(() => {
      const child = spawn(
        process.execPath,
        [
          path.join(HERE, 'verify-limiter.js'),
          '--url',
          `ws://127.0.0.1:${PORT}`,
          '--secs',
          String(PHASE_SECS),
        ],
        { stdio: ['ignore', 'pipe', 'pipe'] },
      );
      let output = '';
      child.stdout.on('data', (d) => (output += d));
      child.stderr.on('data', (d) => (output += d));
      child.on('exit', (code) => finish(code, output));
    }, 400);

    mock.on('error', () => finish(3, 'mock failed to start'));
  });

const NAMES = { 0: 'PASS', 1: 'FAIL', 2: 'INCONCLUSIVE', 3: 'ERROR' };

(async () => {
  console.log('');
  console.log('  check-verifier — asserting what verify-limiter.js refuses to say');
  console.log('');

  let failures = 0;
  for (const [name, spec] of Object.entries(SCENARIOS)) {
    process.stdout.write(`  ${name.padEnd(11)} expect ${NAMES[spec.expect].padEnd(12)} … `);
    const { code, output } = await run(name);
    const codeOk = code === spec.expect;
    // The verdict and the reason are separate claims. See `expectText` in mock-bridge.js:
    // several scenarios reach the right exit code down the wrong branch.
    const textOk = output.includes(spec.expectText);
    const ok = codeOk && textOk;
    console.log(
      ok
        ? `got ${NAMES[code]}  ✓`
        : `got ${NAMES[code] ?? code}` +
            (codeOk ? ` but reason is wrong (wanted "${spec.expectText}")` : '') +
            '  ✗',
    );
    if (!ok) {
      failures += 1;
      console.log('  ─── output ───────────────────────────────────────────');
      console.log(
        output
          .split('\n')
          .map((l) => `  | ${l}`)
          .join('\n'),
      );
      console.log('  ──────────────────────────────────────────────────────');
    }
  }

  console.log('');
  if (failures === 0) {
    console.log(`  ${Object.keys(SCENARIOS).length} scenarios, all verdicts as specified.`);
    console.log('  Most importantly: silence is INCONCLUSIVE, not PASS.');
  } else {
    console.log(`  ${failures} scenario(s) gave the wrong verdict.`);
  }
  console.log('');
  process.exit(failures === 0 ? 0 : 1);
})();
