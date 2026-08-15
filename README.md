# tone-studio-dsp

A headless live-sound DSP engine: 32 input channels, 16 outputs, a master bus with a
31-band graphic EQ, a feedback suppressor, a brickwall limiter and a sub/main crossover.
It runs with no window, opens an audio device, and is driven over OSC.

It is the engine half of **Tone Studio**. The browser half
([`tone-studio-web`](https://github.com/Nadtasitninsacoo/tone-studio-web)) reaches it
through `bridge.js`, a small WebSocket ⇄ OSC gateway in this repository.

## What this is not

**It is not a guitar amplifier.** There is no amp model, no cabinet, no drive stage
anywhere in this codebase. Those live in the web app, which has its own six instrument
racks and convolution cabinets and does not need this engine at all.

This is a mixing console: trim, gate, EQ, de-esser and compressor per channel, then a
master bus. Plug a guitar straight into it and you get a clean DI through a channel strip.

**The web app does not need this engine to work.** It has its own Web Audio mixer and its
own in-browser feedback detector. This engine is an upgrade for anyone running a real PA —
it processes its own audio device, at its own latency, whether or not a browser is open.

## Building

Requires CMake 3.22+ and a C++20 compiler. JUCE is fetched automatically by
`FetchContent`; nothing needs to be installed for it.

```
cmake -B build
cmake --build build --config Release
```

The engine lands at `build/app/tone-studio-app_artefacts/Release/tone-studio-app` and the
test runner at `build/engine/Release/dsp_engine_tests`.

On Windows this was built with Visual Studio 2022 Build Tools and CMake 4.4.2.

## Running

```
tone-studio-app --list-devices
tone-studio-app --input "Microphone (USB-Audio)" --output "Speakers (USB-Audio)"
```

| flag | |
|---|---|
| `--list-devices` | every device this machine offers, in the exact spelling the other flags want |
| `--input <name>` | input device (default: the system default) |
| `--output <name>` | output device (default: the system default) |
| `--device-type <t>` | `"Windows Audio"`, `"DirectSound"`, `"ASIO"`, … |
| `--sample-rate <hz>` | ask the driver for a rate |
| `--buffer <samples>` | ask the driver for a buffer size — this is your latency |
| `--help` | |

A name that matches nothing is a **hard failure with the name printed**, not a quiet
fallback. A typo that silently lands on a laptop's built-in microphone gives you an engine
that runs, meters that move and a PA carrying the wrong source; a process that stops and
says which name it could not find is easier to fix at a venue.

The startup line prints the device that actually opened **with its channel counts**:

```
Audio device starting: Speakers (USB-Audio) @ 48000 Hz, buffer size: 480 samples, 2 in / 8 out
```

`0 in` is worth reading. An output-only endpoint gives you an engine where every meter
reads zero and nothing is wrong with it.

### The bridge

```
node bridge.js
```

No dependencies — the RFC 6455 handshake and the OSC codec are both in the file. It
listens on `ws://127.0.0.1:8080`, sends control to `udp://127.0.0.1:9000` and receives
meters on `udp://127.0.0.1:9001`.

**Loopback only by default.** This process can pull a fader on a live PA. Set
`TONE_BRIDGE_HOST=0.0.0.0` to reach it from a tablet on the same network, and it will say
so loudly at startup. Browser origins are checked: any loopback port is allowed, anything
else needs `TONE_BRIDGE_ORIGINS`.

## Testing

```
cmake --build build --target dsp_engine_tests --config Release
./build/engine/Release/dsp_engine_tests
```

1,041 assertions across every DSP module plus the top-level graph. They run in a few
seconds and need no audio hardware.

### Verifying the master limiter against real sound

The unit tests prove the `Limiter` class holds its ceiling. They cannot prove the *running
engine* does, because the question that is actually open is whether the meter is reading
the signal the limiter acted on — and that lives in `MixingEngine::processBlock`, the
bridge, and the wire between them.

```
tools/verify-limiter.js      drives the running engine through the bridge and judges
tools/mock-bridge.js         a stand-in bridge, so the judging can be tested with no engine
tools/check-verifier.js      asserts what the verifier refuses to say
```

With the engine and `node bridge.js` both running:

```
node tools/verify-limiter.js            # --ch 0  --secs 12  --url ws://localhost:8080
```

It sets trim +24, master +12 and a −6 dB ceiling, then listens through **two phases** —
limiter on, then limiter off at the same drive — while you play. Two phases because "the
peak stayed under −6" has two causes needing opposite fixes: the limiter is working, or the
meter is not tapped after it. One phase cannot tell them apart, which is exactly the
question this tool exists to close.

**It has three verdicts, and the third is the point:**

| exit | | |
|---|---|---|
| 0 | `PASS` | driven over the ceiling, and held |
| 1 | `FAIL` | driven over the ceiling, and not held |
| 2 | `INCONCLUSIVE` | never got it over the ceiling — **nothing was tested** |

`INCONCLUSIVE` is the default and `PASS` is reachable from one place in the file. An
earlier version of this test printed PASS twice over total silence, which is not a slip
anybody makes once: the natural assertion is `peak <= ceiling`, and **silence satisfies it
perfectly**. A script that measures nothing and a script that measures a working limiter
print the same word.

So `tools/check-verifier.js` drives seven scenarios against the mock and asserts both the
verdict *and* the reason given — several reach the right exit code down the wrong branch,
and a wrong diagnosis ("play harder" at a channel that is not routed to the bus) is its own
defect. Run it any time the verifier is edited; it needs no hardware:

```
node tools/check-verifier.js
```

## Status, honestly

Two things are worth knowing before you rely on this.

**Verified against real audio**, on Windows with a USB audio interface: the input path end
to end (device → channel meters → OSC → bridge → browser), the RTA producing a real
spectrum, the master fader and mute, and the control plane in both directions.

**The master limiter has now been measured against a real guitar**, which it had never
been. Driven to **+21.2 dBFS** with the limiter off, the master peaked at **−5.1 dBFS**
with it on — **26.3 dB** of reduction, and 0 of 236 frames over the ceiling against 237 of
237 with it off. That settles both halves of the question at once: the limiter acts on the
master bus, *and* the meter is tapped after it.

**And the same run opened a smaller one that is not settled.** The ceiling was −6.0 dBFS
and the peak with the limiter on was −5.1 — **0.9 dB over it**. On paper that cannot
happen, and the reason is worth stating because the two components are named misleadingly:

- `Limiter.cpp` oversamples **4×** and limits at that rate. It is a genuine true-peak
  limiter.
- `Metering.cpp` stores `peakL = max(peakL, abs(xl))` at base rate, in a field called
  `truePeakDbL`, behind `getTruePeakDbL()`, published as `/meter/master` and described in
  `MixingEngine.cpp` as "Stereo True Peak lookahead". **It is a sample-peak meter with a
  true-peak name.** Its own comment says it "uses oversampled peaks from the limiter, or
  calculates here" and it does neither.

True peaks held at the ceiling bound the sample peaks below it, so a sample-peak meter
should read *at or under* −6, never over. The leading hypothesis is ringing in the
decimation filter on the way back down from 4× — a known effect, normally answered by
limiting slightly below the target internally — but that has not been measured. Running the
verifier again at a different ceiling separates the two cases; the tool prints how.

**Still not verified**: the audio-device recovery path is written but has never been watched
recovering, and the 20 Hz and 25 Hz RTA bands are still bin-limited and report the same
number as each other. Each is stated where it lives in the code as well as here.

Nothing in this repository has been used at a live show.

## Layout

```
engine/          the DSP library — one module per file, each with its own tests
  include/       public headers
  src/
  tests/         JUCE UnitTest, one suite per module + MixingEngineTests for the graph
app/src/Main.cpp the headless host: device selection, OSC control, OSC metering
bridge.js        WebSocket ⇄ OSC gateway for the browser
tools/           verification against a running engine — see Testing above
```

## Licence

**GPLv3.** See [LICENSE](LICENSE).

Copyright (C) 2026 Nadtasit Keng.

This program is free software: you can redistribute it and/or modify it under the terms of
the GNU General Public License as published by the Free Software Foundation, either
version 3 of the License, or (at your option) any later version. It is distributed in the
hope that it will be useful, but **WITHOUT ANY WARRANTY** — without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

### Why GPLv3, and what it does and does not cover

JUCE is dual-licensed: free under GPL, or paid for closed-source distribution. This engine
takes the GPL path, so the binary can be given to anyone at no cost and with no JUCE splash
screen. If this engine is ever to be shipped closed-source, that decision has to be revisited
together with a JUCE Indie or Pro licence — read
[juce.com/legal/juce-7-licence](https://juce.com/legal/juce-7-licence) first, since the terms
differ between JUCE 7 (fetched here) and JUCE 8.

**The web app is not covered by this licence.** It reaches this engine over a WebSocket and
a UDP socket — two processes exchanging messages, not one program linking another — so it is
not a derivative work of anything here. That separation exists for architectural reasons and
happens to keep the licences apart as well. It would *not* survive a move to AGPLv3, which
closes that gap deliberately; GPLv3 is the correct choice here and not an arbitrary one.
