juce_add_console_app(tone-studio-app
    PRODUCT_NAME "ToneStudioHeadless"
    VERSION "1.0.0"
)

target_compile_features(tone-studio-app PUBLIC cxx_std_20)

target_sources(tone-studio-app PRIVATE
    src/Main.cpp
)

# The release version (the root project's VERSION — the one the installer is named after), so
# the engine can tell the web app which build is installed and the install button can say
# "update" when a newer one is published.
target_compile_definitions(tone-studio-app PRIVATE
    TONE_STUDIO_VERSION="${CMAKE_PROJECT_VERSION}")

# ASIO on Windows. Professional interfaces (X32, Wing, M32, Dante, Focusrite, RME) expose all
# of their channels only through ASIO — through Windows Audio they often appear as stereo
# pairs, or as two channels in total, so a 32-input desk would open as a 2-input engine. The
# three SDK headers are vendored under the SDK's GPL v3 option; third_party/asiosdk/README.md
# says which version and why only those three. Windows Audio and DirectSound are untouched:
# ASIO is one more device type in --list-devices, never the default.
if(WIN32)
    target_compile_definitions(tone-studio-app PRIVATE JUCE_ASIO=1)
    target_include_directories(tone-studio-app PRIVATE
        ${CMAKE_SOURCE_DIR}/third_party/asiosdk/common)
endif()

target_link_libraries(tone-studio-app
    PRIVATE
        dsp_engine
        juce::juce_audio_devices
        juce::juce_audio_formats
        juce::juce_events
        juce::juce_osc
)
