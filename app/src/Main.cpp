#include <juce_core/juce_core.h>
#include <juce_events/juce_events.h>
#include <juce_audio_devices/juce_audio_devices.h>
#include <juce_osc/juce_osc.h>
#include "MixingEngine.h"
#include <iostream>
#include <memory>
#include <thread>

class AudioCallback : public juce::AudioIODeviceCallback {
public:
    AudioCallback(dsp::MixingEngine& e) : engine(e) {}

    void audioDeviceIOCallbackWithContext(
        const float* const* inputChannels, int numInputChannels,
        float* const* outputChannels, int numOutputChannels,
        int numSamples, const juce::AudioIODeviceCallbackContext& /*context*/
    ) override {
        // MixingEngine takes `const float**`; JUCE hands over `const float* const*`. The
        // cast drops only the constness of the pointer array itself, not of the samples.
        engine.processAudio(
            const_cast<const float**>(inputChannels), numInputChannels,
            const_cast<float**>(outputChannels), numOutputChannels,
            numSamples
        );
    }

    void audioDeviceAboutToStart(juce::AudioIODevice* device) override {
        /**
         * **The channel counts are printed, and they were the missing line.**
         *
         * The first real run of this engine opened `Headphones (Synaptics Audio)` — an
         * output-only endpoint — so every channel meter, the master meter and all 31 RTA
         * bands reported zero, and there was no way to tell that from a working desk in a
         * quiet room. The device name alone does not say whether anything can get in.
         */
        const int ins = device->getActiveInputChannels().countNumberOfSetBits();
        const int outs = device->getActiveOutputChannels().countNumberOfSetBits();
        std::cout << "Audio device starting: "
                  << device->getName()
                  << " @ " << device->getCurrentSampleRate() << " Hz, "
                  << "buffer size: " << device->getCurrentBufferSizeSamples() << " samples, "
                  << ins << " in / " << outs << " out"
                  << std::endl;
        if (ins == 0) {
            std::cout << "  NOTE: this device has no inputs, so every meter will read zero."
                      << std::endl
                      << "        Use --list-devices to see what else is available, then"
                      << " --input \"<name>\"." << std::endl;
        }
        engine.prepare(device->getCurrentSampleRate(), device->getCurrentBufferSizeSamples());
    }

    void audioDeviceStopped() override {
        std::cout << "Audio device stopped." << std::endl;
        engine.reset();
    }

private:
    dsp::MixingEngine& engine;
};

// =====================================================================================
// OSC control plane — messages arriving from bridge.js on port 9000.
//
// This handled exactly two addresses (/channel/*/fader and /channel/*/trim), so every
// other control on the web UI reached the bridge, was translated, was sent, and landed
// nowhere. A command that is delivered and ignored is worse than one that is refused: the
// page has no way to tell the difference, so it reports success.
// =====================================================================================
class OscControlServer : public juce::OSCReceiver,
                         private juce::OSCReceiver::Listener<juce::OSCReceiver::MessageLoopCallback> {
public:
    OscControlServer(dsp::MixingEngine& e) : engine(e) {
        /**
         * **Loopback only.**
         *
         * `OSCReceiver::connect(port)` binds every interface, so this control plane — faders,
         * mute, the master limiter's on switch, the feedback suppressor's bypass — was
         * reachable from any machine on the network, with no authentication of any kind and
         * bypassing the bridge entirely. The bridge only ever sends to `127.0.0.1`, so
         * nothing legitimate is lost.
         *
         * There is no `connect(host, port)` overload, so the socket is bound here and handed
         * over with `connectToSocket`. It is a member, not a local: the receiver keeps using
         * it for its whole life and a stack socket would close the moment this constructor
         * returned.
         */
        if (socket.bindToPort(9000, juce::String("127.0.0.1")) && connectToSocket(socket)) {
            std::cout << "OSC control listening on 127.0.0.1:9000" << std::endl;
            // One listener for everything. `addListener(this, "/a/*/b")` needs an
            // OSCAddress, which does not accept wildcards, and the per-address overload
            // needs ListenerWithOSCAddress — a different base class. Dispatching on the
            // address below is what this class was already doing anyway, so there is one
            // routing table rather than two that can disagree.
            addListener(this);
        } else {
            std::cerr << "Error: OSC control failed to bind to port 9000" << std::endl;
        }
    }

private:
    static float arg0(const juce::OSCMessage& m) {
        if (m.size() == 0) return 0.0f;
        if (m[0].isFloat32()) return m[0].getFloat32();
        if (m[0].isInt32()) return static_cast<float>(m[0].getInt32());
        return 0.0f;
    }

    void oscMessageReceived(const juce::OSCMessage& message) override {
        const auto address = message.getAddressPattern().toString();
        const float value = arg0(message);

        juce::StringArray tokens;
        tokens.addTokens(address, "/", "");
        /**
         * **`removeEmptyStrings()`, and without it this whole class has never done
         * anything.**
         *
         * Every OSC address starts with `/`, and `StringArray::addTokens` emits an empty
         * token before a leading separator. So `/master/gain` parsed as
         * `[""]["master"]["gain"]` and `tokens[0]` was `""` — while every branch below tests
         * `tokens[0] == "channel"`, `== "master"`, `== "suppressor"`. All three are false for
         * every message that has ever arrived.
         *
         * **No command from the web app has ever reached the DSP.** Not a fader, not the
         * master, not the GEQ, not one of the suppressor controls. The bridge was fixed once
         * for silently dropping half the commands it was given; the other half were being
         * dropped one layer further in, by an off-by-one nobody could see. It survived
         * because the two ends had never been connected: the engine's own test suite does
         * not reach this file, and the bridge's harness checks what goes *out* on the wire
         * rather than what the engine does with it — a stand-in that shares the assumption
         * cannot test the assumption.
         *
         * Found by printing the tokens after `/master/mute 1` arrived, was logged as
         * received, and changed nothing.
         */
        tokens.removeEmptyStrings();

        // ---- /channel/<n>/<param>, n is 1-based on the wire ----------------------
        if (tokens.size() >= 3 && tokens[0] == "channel") {
            const int chanIdx = tokens[1].getIntValue() - 1;
            if (chanIdx < 0 || chanIdx >= dsp::MixingEngine::MaxChannels) return;
            auto& ch = engine.getChannel(chanIdx);
            const auto param = tokens[2];

            // Clamped at the boundary, and again where the gain is computed. These two were
            // the only unbounded writes in the engine — see `MixingEngine::clampFaderDb`.
            if (param == "fader")      ch.faderDb = dsp::MixingEngine::clampFaderDb(value);
            else if (param == "trim")  ch.trimDb = dsp::MixingEngine::clampTrimDb(value);
            else if (param == "pan")   ch.panner.setPan(value);
            // The engine has no `muted` flag; unrouting from the main bus is the same thing
            // and is the only mute this mixer has. Named here so nobody adds a second one.
            else if (param == "mute")  ch.routedToMain = (value < 0.5f);
            return;
        }

        // ---- /master/... ---------------------------------------------------------
        if (tokens.size() >= 2 && tokens[0] == "master") {
            auto& master = engine.getMaster();

            if (tokens.size() >= 3 && tokens[1] == "limiter") {
                if (tokens[2] == "enabled") master.limiter.setEnabled(value >= 0.5f);
                else if (tokens[2] == "ceiling") master.limiter.setCeiling(value);
                return;
            }
            if (tokens.size() >= 3 && tokens[1] == "geq") {
                const int band = tokens[2].getIntValue() - 1;
                if (band >= 0 && band < dsp::GraphicEQ::NumBands) {
                    master.geqL.setBandGain(band, value);
                    master.geqR.setBandGain(band, value);
                }
                return;
            }
            if (tokens.size() >= 4 && tokens[1] == "fx") {
                auto& fx = engine.getFx();
                const auto fxType = tokens[2];
                const auto param = tokens[3];

                if (fxType == "reverb") {
                    if (param == "enabled")       fx.setReverbEnabled(value >= 0.5f);
                    else if (param == "room")     fx.setReverbRoomSize(value);
                    else if (param == "damping")  fx.setReverbDamping(value);
                    else if (param == "width")    fx.setReverbWidth(value);
                    else if (param == "wet")      fx.setReverbWetLevel(value);
                } else if (fxType == "delay") {
                    if (param == "enabled")       fx.setDelayEnabled(value >= 0.5f);
                    else if (param == "time")      fx.setDelayMs(value);
                    else if (param == "feedback")  fx.setDelayFeedback(value);
                    else if (param == "wet")       fx.setDelayWetLevel(value);
                    else if (param == "pingpong")  fx.setDelayPingPong(value >= 0.5f);
                    else if (param == "hpf")       fx.setDelayHpf(value);
                    else if (param == "lpf")       fx.setDelayLpf(value);
                }
                return;
            }
            /**
             * `/master/gain` and `/master/mute` used to be dropped here, with a comment
             * saying the engine had no such field and that inventing a multiply *in this
             * file* would put a gain stage outside the DSP that owns the master bus.
             *
             * The second half of that was right and is still the rule; the answer was to
             * give `MasterBus` the stage it was missing rather than to keep discarding the
             * command. `bridge.js` had been translating both faithfully the whole time, so
             * the web app's master fader moved and did nothing — see `MasterBus::gainDb`.
             */
            if (tokens.size() >= 2 && tokens[1] == "gain") {
                master.gainDb = dsp::MixingEngine::clampFaderDb(value);
                return;
            }
            if (tokens.size() >= 2 && tokens[1] == "mute") {
                master.muted = (value >= 0.5f);
                return;
            }
            return;
        }

        // ---- /suppressor/... -----------------------------------------------------
        if (tokens.size() >= 2 && tokens[0] == "suppressor") {
            auto& s = engine.getMaster().suppressor;
            const auto what = tokens[1];

            // Dynamic slots only. The 8 fixed slots are frequencies somebody chose to hold
            // down permanently; the web page does not own them and must not clear them.
            if (what == "clear-dynamic")            s.clearDynamicSlots();
            else if (what == "bypass")              s.setEnabled(value < 0.5f);
            else if (what == "sensitivity")         s.setSensitivity(value);
            else if (what == "max-dynamic-notches") s.setMaxDynamicNotches(static_cast<int>(value));
            return;
        }
    }

    dsp::MixingEngine& engine;
    /** Bound to loopback and handed to the receiver. Outlives the connection by construction. */
    juce::DatagramSocket socket;
};

// =====================================================================================
// Keeping the audio device open.
//
// Observed, not theorised: a headphone jack was pulled while the engine was running. The
// `Headphones (Synaptics Audio)` endpoint disappeared, `audioDeviceStopped` fired — and
// nothing ever tried to open anything again. The process stayed alive, the OSC meter timer
// went on sending 30 frames a second, and every one of them was a valid number describing
// nothing. Plugging the jack back in did not help; six seconds later the log still read
// `Audio device stopped.`
//
// On a stage that is a PA that goes silent and stays silent until somebody finds the laptop,
// and the trigger is ordinary: a jack, a USB re-enumeration, a driver update. The web app's
// notes record this pedal firing `ended` five times in a single session.
// =====================================================================================
class DeviceKeeper : private juce::Timer {
public:
    DeviceKeeper(juce::AudioDeviceManager& dm, juce::AudioDeviceManager::AudioDeviceSetup s)
        : manager(dm), setup(std::move(s)) {
        startTimer(2000);
    }
    ~DeviceKeeper() override { stopTimer(); }

    /** Whether audio is actually flowing. The meter sender asks before it reports anything. */
    bool isRunning() const {
        auto* device = manager.getCurrentAudioDevice();
        return device != nullptr && device->isPlaying();
    }

private:
    void timerCallback() override {
        if (isRunning()) {
            if (down) {
                down = false;
                attempts = 0;
                std::cout << "Audio device recovered." << std::endl;
            }
            return;
        }

        if (!down) {
            down = true;
            attempts = 0;
            std::cerr << "Audio device is not running — retrying every 2 s." << std::endl;
        }

        // Re-initialise with the same requested devices. `selectDefaultDeviceOnFailure` is
        // true, so if the named device is still absent this lands on whatever exists rather
        // than leaving the desk silent — and the startup line prints what actually opened.
        ++attempts;
        const auto err = manager.initialise(32, 16, nullptr, true, {}, &setup);
        if (err.isEmpty() && isRunning()) {
            down = false;
            std::cout << "Audio device reopened after " << attempts << " attempt(s)." << std::endl;
        } else if (attempts % 15 == 0) {
            // Every 30 s rather than every 2, so a genuinely unplugged interface does not
            // bury everything else in the log.
            std::cerr << "  still no audio device (" << attempts << " attempts): "
                      << (err.isEmpty() ? juce::String("opened but not playing") : err) << std::endl;
        }
    }

    juce::AudioDeviceManager& manager;
    juce::AudioDeviceManager::AudioDeviceSetup setup;
    bool down { false };
    int attempts { 0 };
};

// =====================================================================================
// OSC metering — messages sent to bridge.js on port 9001.
//
// There was no sender at all, so `{type:'meters'}` could never reach the browser: the
// /feedback and /mixer pages connected, reported themselves connected, and showed nothing
// for ever. Everything below exists to close that loop.
//
// Units are what the web app consumes, and they are not uniform on purpose:
//   - levels are sent in dBFS; the bridge converts the ones the app wants linear
//   - the RTA is sent in dBFS and passed straight through, because that is what
//     lib/notchPlot.ts reads
// =====================================================================================
class OscMeterSender : private juce::Timer {
public:
    OscMeterSender(dsp::MixingEngine& e, const DeviceKeeper& k) : engine(e), keeper(k) {
        if (sender.connect("127.0.0.1", 9001)) {
            std::cout << "OSC meters sending to 127.0.0.1:9001" << std::endl;
            // 30 Hz. The browser polls its own copy at 10 Hz, so this is comfortably ahead
            // of the display without putting a message on the wire per audio block.
            startTimerHz(30);
        } else {
            std::cerr << "Error: OSC meter sender failed to connect" << std::endl;
        }
    }

    ~OscMeterSender() override { stopTimer(); }

private:
    void timerCallback() override {
        /**
         * **Absent is not zero, and this is where it was being sent as zero.**
         *
         * When the audio device stops, the engine keeps running and this timer keeps firing.
         * Every field below then reports a perfectly valid −∞ — indistinguishable, on the
         * wire and on the page, from a desk in a quiet room. That is exactly the lie
         * `bridge.js` was written to avoid on its own side (`STALE_MS`, and no `rta` key
         * until an RTA message has actually arrived); the engine was defeating it by
         * continuing to talk.
         *
         * Saying nothing lets the bridge's staleness timeout do its job: the page reports no
         * engine, which is true, instead of a working engine measuring silence.
         */
        if (!keeper.isRunning()) {
            if (!wasSilent) {
                wasSilent = true;
                std::cout << "No audio device — withholding meters rather than sending zeros."
                          << std::endl;
            }
            return;
        }
        if (wasSilent) {
            wasSilent = false;
            std::cout << "Meters resumed." << std::endl;
        }

        auto& master = engine.getMaster();

        sender.send("/meter/master",
                    master.metering.getTruePeakDbL(),
                    master.metering.getTruePeakDbR());

        // Only the channels that could plausibly be in use. Sending all 32 every frame is
        // 32 UDP packets 30 times a second for meters nobody is looking at.
        for (int i = 0; i < ReportedChannels; ++i) {
            auto& ch = engine.getChannel(i);
            sender.send(juce::OSCAddressPattern("/meter/channel/" + juce::String(i + 1)),
                        ch.metering.getPeakDb(),
                        ch.metering.getRmsDb());
        }

        // 31 ISO bands in one message. The web side refuses any other length rather than
        // spreading it across the axis, so this must stay exactly NumRtaBands wide.
        {
            const auto bands = master.metering.getRtaBains();
            juce::OSCMessage rta("/meter/rta");
            for (const auto b : bands) rta.addFloat32(b);
            sender.send(rta);
        }

        // One message per slot, carrying frequency, depth and whether it is deployed.
        {
            /**
             * **Only the dynamic slots.**
             *
             * getNotchReadout() returns all 16 — 8 fixed plus 8 dynamic. The web page owns
             * the dynamic half and nothing else: its Max Notch Filters slider caps automatic
             * detections, and CLEAR NOTCHES maps to clearDynamicSlots(). Reporting the fixed
             * ones too would put eight cards on that page for filters it cannot set, cannot
             * clear, and did not place — controls-that-do-nothing, one layer out.
             */
            const auto notches = master.suppressor.getNotchReadout();
            for (int i = dsp::FeedbackSuppressor::NumFixedSlots; i < (int) notches.size(); ++i) {
                const int slot = i - dsp::FeedbackSuppressor::NumFixedSlots + 1;
                sender.send(juce::OSCAddressPattern("/meter/notch/" + juce::String(slot)),
                            notches[i].frequencyHz,
                            notches[i].gainDb,
                            notches[i].active ? 1.0f : 0.0f);
            }
        }
    }

    static constexpr int ReportedChannels = 8;

    dsp::MixingEngine& engine;
    const DeviceKeeper& keeper;
    bool wasSilent { false };
    juce::OSCSender sender;
};

// =====================================================================================
// Choosing an audio device.
//
// There was no way to. `initialise(32, 16, nullptr, true)` takes whatever Windows calls the
// default, and on the first real run of this engine that was `Headphones (Synaptics Audio)` —
// an output with no inputs at all, so every meter read zero and nothing said why. For a tool
// somebody is meant to install themselves, "go and change your Windows default device" is not
// an answer: the interface they want is usually *not* the system default, precisely because
// they do not want Windows playing notification sounds through the PA.
// =====================================================================================
namespace {

struct Options {
    juce::String inputDevice;
    juce::String outputDevice;
    juce::String deviceType;   // "Windows Audio", "DirectSound", "ASIO", …
    double sampleRate { 0.0 };  // 0 = let the driver choose
    int bufferSize { 0 };       // 0 = let the driver choose
    bool listDevices { false };
    bool showHelp { false };
};

Options parseOptions(int argc, char* argv[]) {
    Options o;
    auto valueAfter = [&](int i) -> juce::String {
        return (i + 1 < argc) ? juce::String(argv[i + 1]) : juce::String();
    };
    for (int i = 1; i < argc; ++i) {
        const juce::String arg(argv[i]);
        if (arg == "--list-devices") o.listDevices = true;
        else if (arg == "--help" || arg == "-h") o.showHelp = true;
        else if (arg == "--input") { o.inputDevice = valueAfter(i); ++i; }
        else if (arg == "--output") { o.outputDevice = valueAfter(i); ++i; }
        else if (arg == "--device-type") { o.deviceType = valueAfter(i); ++i; }
        else if (arg == "--sample-rate") { o.sampleRate = valueAfter(i).getDoubleValue(); ++i; }
        else if (arg == "--buffer") { o.bufferSize = valueAfter(i).getIntValue(); ++i; }
        else {
            // Named rather than ignored. A mistyped flag that is silently dropped leaves the
            // engine on the default device with the operator believing otherwise, which is
            // the failure this whole block exists to remove.
            std::cerr << "Unknown option: " << arg << "  (try --help)" << std::endl;
        }
    }
    return o;
}

void printHelp() {
    std::cout
        << "Usage: tone-studio-app [options]\n\n"
        << "  --list-devices          print every audio device this machine offers, and exit\n"
        << "  --input <name>          input device to open   (default: the system default)\n"
        << "  --output <name>         output device to open  (default: the system default)\n"
        << "  --device-type <type>    e.g. \"Windows Audio\", \"DirectSound\", \"ASIO\"\n"
        << "  --sample-rate <hz>      ask the driver for this rate\n"
        << "  --buffer <samples>      ask the driver for this buffer size\n"
        << "  --help                  this text\n\n"
        << "Names must match --list-devices exactly, quotes included where there are spaces.\n"
        << "The control plane listens on 127.0.0.1:9000 and sends meters to 127.0.0.1:9001;\n"
        << "run bridge.js to reach it from a browser.\n"
        << std::endl;
}

void listDevices(juce::AudioDeviceManager& deviceManager) {
    std::cout << "Audio devices on this machine:\n" << std::endl;
    for (auto* type : deviceManager.getAvailableDeviceTypes()) {
        type->scanForDevices();
        std::cout << "[" << type->getTypeName() << "]" << std::endl;

        const auto outs = type->getDeviceNames(false);
        const auto ins = type->getDeviceNames(true);
        std::cout << "  inputs:" << (ins.isEmpty() ? "  (none)" : "") << std::endl;
        for (const auto& n : ins) std::cout << "    --input  \"" << n << "\"" << std::endl;
        std::cout << "  outputs:" << (outs.isEmpty() ? "  (none)" : "") << std::endl;
        for (const auto& n : outs) std::cout << "    --output \"" << n << "\"" << std::endl;
        std::cout << std::endl;
    }
}

} // namespace

// Headless Application Entry
int main(int argc, char* argv[]) {
    // Initialise JUCE system
    juce::ScopedJuceInitialiser_GUI initialiser;

    std::cout << "===========================================" << std::endl;
    std::cout << "   Tone Studio Headless DSP Engine v1.0.0  " << std::endl;
    std::cout << "===========================================" << std::endl;

    const Options options = parseOptions(argc, argv);
    if (options.showHelp) {
        printHelp();
        return 0;
    }

    juce::AudioDeviceManager deviceManager;

    if (options.listDevices) {
        listDevices(deviceManager);
        return 0;
    }

    dsp::MixingEngine engine;
    AudioCallback audioCallback(engine);

    if (options.deviceType.isNotEmpty()) {
        deviceManager.setCurrentAudioDeviceType(options.deviceType, true);
    }

    /**
     * The requested devices, if any, are handed over as `preferredSetupOptions`.
     *
     * **What actually happens with a name that matches nothing, measured rather than
     * assumed:** JUCE returns `No such device: <name>` and this exits, despite
     * `selectDefaultDeviceOnFailure` being true — that flag covers a failure to *open* a
     * device, not a name it cannot find. The comment here originally claimed the opposite.
     *
     * The behaviour is the right one and is kept deliberately. A typo that silently lands on
     * the laptop's built-in microphone gives an engine that runs, meters that move and a PA
     * carrying the wrong source — which is far worse at a venue than a process that stops and
     * says which name it could not find. The error names it and points at `--list-devices`.
     *
     * When a device *is* found, what actually opened is printed either way, by
     * `audioDeviceAboutToStart`, with its channel counts.
     */
    juce::AudioDeviceManager::AudioDeviceSetup setup;
    setup.inputDeviceName = options.inputDevice;
    setup.outputDeviceName = options.outputDevice;
    setup.sampleRate = options.sampleRate;
    setup.bufferSize = options.bufferSize;
    setup.useDefaultInputChannels = true;
    setup.useDefaultOutputChannels = true;

    // Initialise audio device with 32 inputs / 16 outputs target
    juce::String err = deviceManager.initialise(32, 16, nullptr, true, {}, &setup);

    if (err.isNotEmpty()) {
        std::cerr << "Audio Device Initialisation Warning: " << err << std::endl;
        // Retry with default settings (usually 2 inputs, 2 outputs)
        err = deviceManager.initialise(2, 2, nullptr, true, {}, &setup);
        if (err.isNotEmpty()) {
            std::cerr << "Audio Device Error: " << err << std::endl;
            std::cerr << "Run with --list-devices to see what this machine offers."
                      << std::endl;
            return 1;
        }
    }

    /**
     * **Say which devices actually opened, both of them, by name.**
     *
     * `AudioIODevice::getName()` returns the *output* device on Windows, so the startup log
     * named the headphones and said nothing at all about where audio was coming from. With
     * `selectDefaultDeviceOnFailure` true a mistyped `--input` silently lands on the system
     * default, and the only symptom is meters that read zero — which is also what a guitar
     * with its volume down looks like, and what a muted Windows endpoint looks like.
     *
     * Three indistinguishable causes with one symptom is exactly the situation this project
     * keeps paying for. One line removes one of them.
     */
    {
        // `getAudioDeviceSetup()`, not `getCurrentAudioDeviceSetup()` — the latter does not
        // exist in this JUCE version and the compiler is the only thing that says so.
        const auto actual = deviceManager.getAudioDeviceSetup();
        std::cout << "  input device : "
                  << (actual.inputDeviceName.isEmpty() ? "(none)" : actual.inputDeviceName)
                  << std::endl;
        std::cout << "  output device: "
                  << (actual.outputDeviceName.isEmpty() ? "(none)" : actual.outputDeviceName)
                  << std::endl;
        if (options.inputDevice.isNotEmpty() && actual.inputDeviceName != options.inputDevice) {
            std::cerr << "  WARNING: asked for input \"" << options.inputDevice
                      << "\" and got \"" << actual.inputDeviceName
                      << "\" — check the spelling against --list-devices" << std::endl;
        }
    }

    deviceManager.addAudioCallback(&audioCallback);

    // Setup control plane
    // The keeper must outlive the meter sender, which holds a reference to it.
    DeviceKeeper deviceKeeper(deviceManager, setup);
    OscControlServer oscServer(engine);
    OscMeterSender meterSender(engine, deviceKeeper);

    std::cout << "Press Enter to stop the engine..." << std::endl;

    /**
     * **The message loop has to actually run.**
     *
     * `juce::Timer` and `OSCReceiver::MessageLoopCallback` are both dispatched by the
     * MessageManager, and this blocked on `std::cin.get()` instead — so the loop never ran a
     * single iteration. The meter timer never fired and no OSC command was ever delivered:
     * the engine bound both ports, printed that it was listening and sending, and did
     * neither. From the outside it looked exactly like a working engine on a quiet desk.
     *
     * Enter still stops it; the wait just moved to its own thread so the main one can
     * dispatch.
     */
    /**
     * Enter-to-quit only when there is somebody there to press it.
     *
     * Run headless — as a service at a venue, or from any launcher that does not give the
     * process a console — `std::cin.get()` returns EOF instantly and the engine shuts itself
     * down a few milliseconds after starting. It looks identical to a crash, and the only
     * clue is that it printed its whole banner first.
     */
    std::thread quitWatcher([] {
        // EOF means there is no console behind stdin — a service, a launcher, a redirect.
        // `_isatty` is not the test: on Windows it reports true for NUL as well, so the
        // engine shut itself down a few milliseconds after printing its banner and looked
        // exactly like a crash. Waiting for a keypress that can never come is the correct
        // behaviour; quitting because nobody is there is not.
        if (std::cin.get() == EOF) {
            std::cout << "(no console attached - running until terminated)" << std::endl;
            return;
        }
        juce::MessageManager::getInstance()->stopDispatchLoop();
    });

    juce::MessageManager::getInstance()->runDispatchLoop();

    if (quitWatcher.joinable()) quitWatcher.join();
    deviceManager.removeAudioCallback(&audioCallback);
    return 0;
}
