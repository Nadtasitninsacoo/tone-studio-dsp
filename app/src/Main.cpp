#include <juce_core/juce_core.h>
#include <juce_events/juce_events.h>
#include <juce_audio_devices/juce_audio_devices.h>
#include <juce_osc/juce_osc.h>
#include "MixingEngine.h"
#include <atomic>
#include <cstdint>
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

        // The watchdog's two readings (see `DeviceController`). A block counter, so a stream
        // that has stopped calling back is visible even while `isPlaying()` says true; and a
        // run of exact digital zeros across every input, which is what a card on a second
        // clock delivers — a real input always carries some noise.
        blocks.fetch_add(1, std::memory_order_relaxed);
        bool allZero = numInputChannels > 0;
        for (int c = 0; c < numInputChannels && allZero; ++c) {
            const float* in = inputChannels[c];
            if (in == nullptr) continue;
            for (int i = 0; i < numSamples; ++i) {
                if (in[i] != 0.0f) { allZero = false; break; }
            }
        }
        if (allZero) zeroSamples.fetch_add(static_cast<uint64_t>(numSamples), std::memory_order_relaxed);
        else zeroSamples.store(0, std::memory_order_relaxed);
    }

    /** Callbacks run since start. Read by the watchdog on the message thread. */
    std::atomic<uint64_t> blocks { 0 };
    /** Consecutive samples in which every input was exactly zero. */
    std::atomic<uint64_t> zeroSamples { 0 };

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
        zeroSamples.store(0, std::memory_order_relaxed);
    }

    void audioDeviceStopped() override {
        std::cout << "Audio device stopped." << std::endl;
        engine.reset();
    }

private:
    dsp::MixingEngine& engine;
};

// =====================================================================================
// Keeping the audio device open, and choosing it from the web page.
//
// Observed, not theorised: a headphone jack was pulled while the engine was running, the
// endpoint disappeared, `audioDeviceStopped` fired — and nothing ever tried to open anything
// again. On a stage that is a PA that goes silent until somebody finds the laptop. So this
// reopens a device that stopped, every 2 s, with the same requested setup.
//
// It also owns the two things the web page's Device / I/O screen needs (1.0.15):
//
//  - **Choosing the cards, the rate and the buffer at runtime** (`/device/set`), so changing
//    the interface no longer means closing the launcher window. A change that opens is also
//    written to the launcher's own `devices.json`, so the next start uses it.
//  - **A watchdog for silence that should not be silence.** Two readings from the audio
//    callback: the block counter stops advancing (the stream died while `isPlaying()` still
//    says true — the failure that is otherwise invisible), or every input has delivered exact
//    digital zeros for 3 s (what an input on a second clock delivers; a connected ADC always
//    carries some noise). A stalled stream is always restarted. A dead input is reported, and
//    restarted only when auto-restart is on, at most once per 30 s, so a card that really is
//    delivering zeros cannot put the engine into a restart loop.
// =====================================================================================
/**
 * **A laptop's own microphone, by name.** Asked for as "เวลาใช้หน้าเวที เสียงต้องไม่ลูปเข้าไมค์
 * โน้ตบุ๊ค ต้องปิดไมค์ไปเลย เพราะอยู่ในโหมดมิกซ์คอนโทรลเลอร์": this engine feeds a PA, and a
 * room microphone in front of a PA is a howl, not an input. So the engine never opens one —
 * not from the web page, not from the launcher's saved choice, and not as a fallback when the
 * interface drops off the bus.
 *
 * A block list, not an allow list: an ASIO driver called "X32" or "Dante Virtual Soundcard"
 * matches no interface brand and must still open. What is listed is the laptop codecs and the
 * endpoint names Windows gives a built-in or headset microphone.
 */
static bool isBuiltInMic(const juce::String& name) {
    if (name.isEmpty()) return false;
    const auto n = name.toLowerCase();
    static const char* const words[] = {
        "microphone array", "internal mic", "built-in", "builtin", "integrated", "webcam", "camera",
        "headset", "external microphone", "realtek", "synaptics", "conexant", "smart sound",
        "intel(r) display", "cirrus", "idt high", "sigmatel", "high definition audio",
        "primary sound capture",
    };
    for (auto* w : words)
        if (n.contains(w)) return true;
    return false;
}

class DeviceKeeper : private juce::Timer {
public:
    DeviceKeeper(juce::AudioDeviceManager& dm, juce::AudioDeviceManager::AudioDeviceSetup s,
                 AudioCallback& cb)
        : manager(dm), setup(std::move(s)), callback(cb) {
        sender.connect("127.0.0.1", 9001);
        lastBlocks = callback.blocks.load();
        lastAdvance = juce::Time::getMillisecondCounter();
        startTimer(500);
    }
    ~DeviceKeeper() override { stopTimer(); }

    /** Whether audio is actually flowing. The meter sender asks before it reports anything. */
    bool isRunning() const {
        auto* device = manager.getCurrentAudioDevice();
        return device != nullptr && device->isPlaying() && ! stalled;
    }

    /** How many input / output channels the open device really has. 0 when none is open. */
    int activeInputs() const {
        auto* device = manager.getCurrentAudioDevice();
        return device != nullptr ? device->getActiveInputChannels().countNumberOfSetBits() : 0;
    }
    int activeOutputs() const {
        auto* device = manager.getCurrentAudioDevice();
        return device != nullptr ? device->getActiveOutputChannels().countNumberOfSetBits() : 0;
    }

    // ---- requests from the web page (message thread, via OscControlServer) ----------

    /** `/device/list` — every type, its inputs and outputs, and which Windows calls default. */
    void sendCatalog() {
        juce::Array<juce::var> types;
        for (auto* type : manager.getAvailableDeviceTypes()) {
            type->scanForDevices();
            auto* t = new juce::DynamicObject();
            t->setProperty("type", type->getTypeName());
            t->setProperty("separate", type->hasSeparateInputsAndOutputs());
            auto names = [&](bool input) {
                juce::Array<juce::var> list;
                const auto all = type->getDeviceNames(input);
                const int def = type->getDefaultDeviceIndex(input);
                for (int i = 0; i < all.size(); ++i) {
                    auto* d = new juce::DynamicObject();
                    d->setProperty("name", all[i]);
                    d->setProperty("default", i == def);
                    if (input) d->setProperty("builtIn", isBuiltInMic(all[i]));
                    list.add(juce::var(d));
                }
                return list;
            };
            t->setProperty("inputs", names(true));
            t->setProperty("outputs", names(false));
            types.add(juce::var(t));
        }
        auto* root = new juce::DynamicObject();
        root->setProperty("types", types);
        send("/device/catalog", juce::var(root));
    }

    /**
     * `/device/set` — a JSON object `{type, input, output, sampleRate, bufferSize}`. Any field
     * left out keeps what is open now. Replies `/device/result` with `{ok, error}`; a setup that
     * will not open puts the previous one back rather than leaving the desk with nothing.
     */
    void applyJson(const juce::String& json) {
        const auto v = juce::JSON::parse(json);
        if (! v.isObject()) { sendResult(false, "not a JSON object"); return; }

        const auto previousType = manager.getCurrentAudioDeviceType();
        const auto previous = setup;
        const auto openNow = manager.getAudioDeviceSetup();
        auto next = openNow;
        const auto typeName = v.getProperty("type", juce::var()).toString();
        if (typeName.isNotEmpty() && typeName != previousType) {
            bool known = false;
            for (auto* t : manager.getAvailableDeviceTypes()) known = known || t->getTypeName() == typeName;
            if (! known) { sendResult(false, "no such driver type: " + typeName); return; }
            manager.setCurrentAudioDeviceType(typeName, true);
            next = manager.getAudioDeviceSetup();
        }
        if (v.hasProperty("input"))      next.inputDeviceName = v["input"].toString();
        if (v.hasProperty("output"))     next.outputDeviceName = v["output"].toString();
        /*
         * A different card starts from **its own** rate and buffer unless the page named them.
         * Carrying the old card's figures across is how "back to the Tank-G" failed: the laptop
         * card was open at 48 kHz, the Tank-G only does 44.1, the open was refused, and the
         * rollback left the laptop microphone in charge — reported as "จะกลับก็ไม่ได้".
         */
        const bool cardChanged = next.inputDeviceName != openNow.inputDeviceName
                              || next.outputDeviceName != openNow.outputDeviceName
                              || typeName.isNotEmpty();
        if (cardChanged) { next.sampleRate = 0; next.bufferSize = 0; }
        if (v.hasProperty("sampleRate")) next.sampleRate = static_cast<double>(v["sampleRate"]);
        if (v.hasProperty("bufferSize")) next.bufferSize = static_cast<int>(v["bufferSize"]);
        next.useDefaultInputChannels = true;
        next.useDefaultOutputChannels = true;

        // One ASIO driver is both directions — the same rule as the command line.
        if (auto* type = manager.getCurrentDeviceTypeObject(); type != nullptr && ! type->hasSeparateInputsAndOutputs()) {
            if (v.hasProperty("input")) next.outputDeviceName = next.inputDeviceName;
            else if (v.hasProperty("output")) next.inputDeviceName = next.outputDeviceName;
        }

        if (isBuiltInMic(next.inputDeviceName)) {
            sendResult(false, "\"" + next.inputDeviceName + "\" เป็นไมค์ในเครื่อง — ปิดไว้ในโหมดมิกซ์คอนโทรลเลอร์ เพื่อไม่ให้เสียงลำโพงวนเข้าไมค์");
            return;
        }
        auto err = manager.setAudioDeviceSetup(next, true);
        if ((err.isNotEmpty() || manager.getCurrentAudioDevice() == nullptr)
            && (next.sampleRate > 0 || next.bufferSize > 0)) {
            // The card may simply not do the rate or buffer asked for. Let its driver choose
            // before giving up on the card itself.
            std::cerr << "  " << err << " — retrying with the card's own rate and buffer" << std::endl;
            next.sampleRate = 0;
            next.bufferSize = 0;
            err = manager.setAudioDeviceSetup(next, true);
        }
        if (err.isNotEmpty() || manager.getCurrentAudioDevice() == nullptr) {
            std::cerr << "Device change refused: " << err << " — going back to the previous setup" << std::endl;
            if (manager.getCurrentAudioDeviceType() != previousType) manager.setCurrentAudioDeviceType(previousType, true);
            manager.setAudioDeviceSetup(previous, true);
            resetWatch();
            sendResult(false, err.isNotEmpty() ? err : juce::String("the device did not open"));
            sendState();
            return;
        }
        setup = manager.getAudioDeviceSetup();
        resetWatch();
        saveLauncherConfig();
        std::cout << "Device changed from the web page: in \"" << setup.inputDeviceName
                  << "\" / out \"" << setup.outputDeviceName << "\" @ " << setup.sampleRate
                  << " Hz, " << setup.bufferSize << " samples" << std::endl;
        sendResult(true, {});
        sendState();
    }

    /** `/device/restart` — close and reopen the same setup, without ending the process. */
    void restart(const juce::String& reason) {
        std::cout << "Restarting audio (" << reason << ")" << std::endl;
        manager.closeAudioDevice();
        // `setAudioDeviceSetup`, never `initialise`: the latter fills an empty or missing name
        // with the Windows default, which is the laptop microphone.
        const auto err = manager.setAudioDeviceSetup(setup, true);
        ++restarts;
        lastRestartReason = reason;
        lastRestartAt = juce::Time::getMillisecondCounter();
        resetWatch();
        if (err.isNotEmpty()) std::cerr << "  restart failed: " << err << std::endl;
        sendState();
    }

    void setAutoRestart(bool on) { autoRestart = on; sendState(); }

private:
    static constexpr juce::uint32 StallMs = 3000;
    static constexpr double DeadInputSec = 3.0;
    static constexpr juce::uint32 AutoRestartGapMs = 30000;

    void timerCallback() override {
        const auto now = juce::Time::getMillisecondCounter();
        auto* device = manager.getCurrentAudioDevice();
        const bool playing = device != nullptr && device->isPlaying();

        // The callback counter, and whether it moved.
        const auto b = callback.blocks.load(std::memory_order_relaxed);
        if (b != lastBlocks) {
            lastBlocks = b;
            lastAdvance = now;
            stalled = false;
        } else if (playing && ! stalled && now - lastAdvance > StallMs) {
            stalled = true;
            std::cerr << "Audio stream stalled (no callback for " << (now - lastAdvance)
                      << " ms) — restarting it" << std::endl;
            restart("stream stalled");
            return;
        }

        if (playing) {
            if (down) { down = false; attempts = 0; std::cout << "Audio device recovered." << std::endl; }
            const double sr = device->getCurrentSampleRate();
            const double zeroSec = sr > 0 ? static_cast<double>(callback.zeroSamples.load()) / sr : 0.0;
            const bool dead = activeInputs() > 0 && zeroSec >= DeadInputSec;
            if (dead && ! inputDead) std::cerr << "Every input has delivered exact zeros for 3 s" << std::endl;
            inputDead = dead;
            if (dead && autoRestart && now - lastRestartAt > AutoRestartGapMs) {
                restart("inputs silent");
                return;
            }
        } else {
            inputDead = false;
            if (! down) { down = true; attempts = 0; std::cerr << "Audio device is not running — retrying every 2 s." << std::endl; }
            // Every fourth tick: the timer runs at 500 ms for the watchdog, the reopen stays at 2 s.
            //
            // **Never the default device.** `selectDefaultDeviceOnFailure` was true here, so a
            // Tank-G that dropped off the USB bus mid-show was replaced by whatever Windows calls
            // the default — the laptop's own microphone, straight into a PA. Asked as "ถ้าใช้
            // หน้าเวที มันจะไปรับไมค์โน้ตบุ๊คแทนอีกไหม". Recovery now waits for the card that was
            // chosen; silence until it is back is the safe failure, a room mic is not.
            if (++tick % 4 == 0) {
                ++attempts;
                const auto err = manager.setAudioDeviceSetup(setup, true);
                auto* reopened = manager.getCurrentAudioDevice();
                if (err.isEmpty() && reopened != nullptr && reopened->isPlaying()) {
                    down = false;
                    resetWatch();
                    std::cout << "Audio device reopened after " << attempts << " attempt(s)." << std::endl;
                } else if (attempts % 15 == 0) {
                    std::cerr << "  still no audio device (" << attempts << " attempts): "
                              << (err.isEmpty() ? juce::String("opened but not playing") : err) << std::endl;
                }
            }
        }
        sendState();
    }

    void resetWatch() {
        lastBlocks = callback.blocks.load();
        lastAdvance = juce::Time::getMillisecondCounter();
        stalled = false;
        inputDead = false;
        callback.zeroSamples.store(0);
    }

    /** `/device/state`, twice a second: what is open, what it could be, and the watchdog. */
    void sendState() {
        auto* device = manager.getCurrentAudioDevice();
        const auto actual = manager.getAudioDeviceSetup();
        auto* o = new juce::DynamicObject();
        o->setProperty("type", manager.getCurrentAudioDeviceType());
        o->setProperty("input", actual.inputDeviceName);
        o->setProperty("output", actual.outputDeviceName);
        o->setProperty("running", device != nullptr && device->isPlaying() && ! stalled);
        o->setProperty("stalled", stalled);
        o->setProperty("inputs", activeInputs());
        o->setProperty("outputs", activeOutputs());
        o->setProperty("inputDead", inputDead);
        o->setProperty("autoRestart", autoRestart);
        o->setProperty("restarts", restarts);
        o->setProperty("lastRestartReason", lastRestartReason);
        double zeroSec = 0;
        if (device != nullptr) {
            const double sr = device->getCurrentSampleRate();
            o->setProperty("sampleRate", sr);
            o->setProperty("bufferSize", device->getCurrentBufferSizeSamples());
            o->setProperty("latencyMs", 1000.0 * (device->getInputLatencyInSamples() + device->getOutputLatencyInSamples())
                                          / juce::jmax(1.0, sr));
            juce::Array<juce::var> rates, sizes;
            for (auto r : device->getAvailableSampleRates()) rates.add(r);
            for (auto z : device->getAvailableBufferSizes()) sizes.add(z);
            o->setProperty("sampleRates", rates);
            o->setProperty("bufferSizes", sizes);
            zeroSec = sr > 0 ? static_cast<double>(callback.zeroSamples.load()) / sr : 0.0;
        }
        o->setProperty("inputSilentSec", zeroSec);
        send("/device/state", juce::var(o));
    }

    void sendResult(bool ok, const juce::String& error) {
        auto* o = new juce::DynamicObject();
        o->setProperty("ok", ok);
        o->setProperty("error", error);
        send("/device/result", juce::var(o));
    }

    void send(const juce::String& address, const juce::var& value) {
        sender.send(juce::OSCAddressPattern(address), juce::JSON::toString(value, true));
    }

    /** The launcher reads this on its next start, so a choice made on the web page sticks. */
    void saveLauncherConfig() {
        const auto dir = juce::File::getSpecialLocation(juce::File::userApplicationDataDirectory)
                             .getChildFile("ToneStudioEngine");
        dir.createDirectory();
        auto* o = new juce::DynamicObject();
        o->setProperty("input", setup.inputDeviceName);
        o->setProperty("output", setup.outputDeviceName);
        o->setProperty("type", manager.getCurrentAudioDeviceType() == "ASIO" ? "ASIO" : "");
        if (setup.sampleRate > 0) o->setProperty("sampleRate", setup.sampleRate);
        if (setup.bufferSize > 0) o->setProperty("bufferSize", setup.bufferSize);
        dir.getChildFile("devices.json").replaceWithText(juce::JSON::toString(juce::var(o)), false, false, "\r\n");
    }

    juce::AudioDeviceManager& manager;
    juce::AudioDeviceManager::AudioDeviceSetup setup;
    AudioCallback& callback;
    juce::OSCSender sender;
    bool down { false };
    int attempts { 0 };
    int tick { 0 };
    uint64_t lastBlocks { 0 };
    juce::uint32 lastAdvance { 0 };
    bool stalled { false };
    bool inputDead { false };
    bool autoRestart { true };
    int restarts { 0 };
    juce::String lastRestartReason;
    juce::uint32 lastRestartAt { 0 };
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
    OscControlServer(dsp::MixingEngine& e, DeviceKeeper& k) : engine(e), keeper(k) {
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

        // ---- /channel/... and /master/... ---------------------------------------
        //
        // Queued, not applied. This is the message thread; the audio callback is running the
        // same filters right now, and writing into them from here is a torn setting mid-block.
        // `MixingEngine::postControl` hands the address to the audio thread, which applies it
        // at the top of its next block through `setControl` — the routing table that used to
        // be written out here, now where the test suite can reach it.
        if (tokens.size() >= 2 && (tokens[0] == "channel" || tokens[0] == "master" || tokens[0] == "output" || tokens[0] == "aux")) {
            if (!engine.postControl(address.toStdString(), value)) {
                // Said, not swallowed: too long an address, or a full queue.
                std::cerr << "Dropped " << address << " " << value
                          << " (control queue full or address too long)" << std::endl;
            }
            return;
        }
        // ---- /device/... --------------------------------------------------------
        // The web page's Device / I/O screen. On this thread on purpose: opening and closing
        // a device belongs to the message thread, never to the audio callback.
        if (tokens.size() >= 2 && tokens[0] == "device") {
            const auto what = tokens[1];
            if (what == "list")              keeper.sendCatalog();
            else if (what == "restart")      keeper.restart("requested from the web page");
            else if (what == "autorestart")  keeper.setAutoRestart(value >= 0.5f);
            else if (what == "set") {
                if (message.size() >= 1 && message[0].isString()) keeper.applyJson(message[0].getString());
                else std::cerr << "/device/set needs one JSON string argument" << std::endl;
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
    DeviceKeeper& keeper;
    /** Bound to loopback and handed to the receiver. Outlives the connection by construction. */
    juce::DatagramSocket socket;
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
            // 60 Hz — a display's own rate — so a ladder moves with each hit instead of in
            // 33 ms steps ("กระพริบเร็ว แบบเรียลไทม์ตามเสียง"). Loopback UDP; the cost is nothing.
            startTimerHz(60);
        } else {
            std::cerr << "Error: OSC meter sender failed to connect" << std::endl;
        }
    }

    ~OscMeterSender() override { stopTimer(); }

private:
    void timerCallback() override {
        // The racks' impulse responses are built here, on the message thread: building one
        // allocates, so the audio thread only flags that a setting made one stale. First, and
        // before the silent-device return below — a cabinet chosen while the device is down
        // must be ready when it comes back.
        engine.serviceBackgroundWork();

        // The audio thread applies queued controls and cannot print; it counts the ones its
        // table refused, and this is where that count becomes a line somebody can read.
        if (const int rejected = engine.rejectedControls(); rejected != lastRejected) {
            std::cerr << "Ignored " << (rejected - lastRejected)
                      << " control(s): unknown parameter, channel out of range, or not a number"
                      << std::endl;
            lastRejected = rejected;
        }

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

        // How many inputs the device really opened. Channel n reads input n, so on a
        // 2-input interface channels 3..32 are silent *because nothing is plugged into them*
        // — which on the page looks exactly like a dead meter unless the page is told.
        // Sent every frame rather than once: the bridge and the browser can both start after
        // the engine, and a device change mid-run changes the answer.
        sender.send("/engine/io", keeper.activeInputs(), keeper.activeOutputs());

        // Which build is running, so the web app's install button can offer an update. Every
        // frame for the same reason as the line above: whoever starts last must still hear it.
        sender.send("/engine/version", juce::String(TONE_STUDIO_VERSION));

        // Only the channels that could plausibly be in use. Sending all 32 every frame is
        // 32 UDP packets 30 times a second for meters nobody is looking at.
        const int inputs = keeper.activeInputs();
        for (int i = 0; i < ReportedChannels; ++i) {
            auto& ch = engine.getChannel(i);
            sender.send(juce::OSCAddressPattern("/meter/channel/" + juce::String(i + 1)),
                        ch.metering.getPeakDb(),
                        ch.metering.getRmsDb());
            // The seven-band shape the assistant's "ears" read — only for a channel patched to
            // an input the device has, since the rest are silent and would be 7 floats of
            // −120 each, thirty times a second.
            if (ch.inputIndex >= 0 && ch.inputIndex < inputs) {
                juce::OSCMessage bands(juce::OSCAddressPattern("/meter/channel/" + juce::String(i + 1) + "/bands"));
                for (int b = 0; b < dsp::ChannelMetering::NumShapeBands; ++b) bands.addFloat32(ch.metering.getBandDb(b));
                sender.send(bands);
            }
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

    // The engine's own figure: it also decides which unpatched channels get their meters zeroed.
    static constexpr int ReportedChannels = dsp::MixingEngine::MeteredChannels;
    int lastRejected { 0 };

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
        // `(default)` marks the one Windows itself is using, so the launcher can recommend the
        // speakers the machine already plays through. After the closing quote on purpose: a
        // reader matching `"(.+)"` still gets the bare name.
        const int defaultIn = type->getDefaultDeviceIndex(true);
        const int defaultOut = type->getDefaultDeviceIndex(false);
        std::cout << "  inputs:" << (ins.isEmpty() ? "  (none)" : "") << std::endl;
        for (int i = 0; i < ins.size(); ++i)
            std::cout << "    --input  \"" << ins[i] << "\""
                      << (i == defaultIn ? "  (default)" : "") << std::endl;
        std::cout << "  outputs:" << (outs.isEmpty() ? "  (none)" : "") << std::endl;
        for (int i = 0; i < outs.size(); ++i)
            std::cout << "    --output \"" << outs[i] << "\""
                      << (i == defaultOut ? "  (default)" : "") << std::endl;
        std::cout << std::endl;
    }
}

} // namespace

// Headless Application Entry
int main(int argc, char* argv[]) {
    // Initialise JUCE system
    juce::ScopedJuceInitialiser_GUI initialiser;

    std::cout << "===========================================" << std::endl;
    std::cout << "   Tone Studio Headless DSP Engine v" TONE_STUDIO_VERSION "  " << std::endl;
    std::cout << "===========================================" << std::endl;
    // The short notice GPLv3 asks an interactive program to show at startup.
    std::cout << "Copyright (C) 2026 Nadtasit Keng.  GPLv3 — see LICENSE." << std::endl;
    std::cout << "This program comes with ABSOLUTELY NO WARRANTY." << std::endl;

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

    /*
     * **An ASIO driver is one device for both directions.** Windows Audio lists inputs and
     * outputs separately, so `--input` alone is fine there. ASIO does not: the driver *is* the
     * interface, and only one can be open at a time. Given only `--input "X32"`, JUCE fills
     * the empty output with the type's default — on a machine that also has ASIO4ALL or a
     * second interface installed that is a *different* driver, and the open fails with an
     * error about a device nobody named. So when the type cannot separate them, one name
     * means both.
     */
    if (auto* type = deviceManager.getCurrentDeviceTypeObject();
        type != nullptr && ! type->hasSeparateInputsAndOutputs()) {
        if (setup.outputDeviceName.isEmpty()) setup.outputDeviceName = setup.inputDeviceName;
        if (setup.inputDeviceName.isEmpty()) setup.inputDeviceName = setup.outputDeviceName;
    }

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

    // A laptop microphone is never opened — not from the launcher's saved choice and not as
    // the default JUCE filled in. Output only until a real input is chosen. See `isBuiltInMic`.
    if (const auto opened = deviceManager.getAudioDeviceSetup(); isBuiltInMic(opened.inputDeviceName)) {
        std::cerr << "  REFUSED input \"" << opened.inputDeviceName
                  << "\": a built-in microphone is never opened in front of a PA."
                  << " Running output-only — choose the interface on the web page's Device / I/O screen."
                  << std::endl;
        auto outOnly = opened;
        outOnly.inputDeviceName = {};
        deviceManager.setAudioDeviceSetup(outOnly, true);
        setup.inputDeviceName = {};
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
    DeviceKeeper deviceKeeper(deviceManager, setup, audioCallback);
    OscControlServer oscServer(engine, deviceKeeper);
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
