juce_add_console_app(tone-studio-app
    PRODUCT_NAME "ToneStudioHeadless"
    VERSION "1.0.0"
)

target_compile_features(tone-studio-app PUBLIC cxx_std_20)

target_sources(tone-studio-app PRIVATE
    src/Main.cpp
)

target_link_libraries(tone-studio-app
    PRIVATE
        dsp_engine
        juce::juce_audio_devices
        juce::juce_audio_formats
        juce::juce_events
        juce::juce_osc
)
