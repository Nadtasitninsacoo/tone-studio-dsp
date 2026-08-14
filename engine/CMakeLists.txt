add_library(dsp_engine STATIC)

target_compile_features(dsp_engine PUBLIC cxx_std_20)

target_include_directories(dsp_engine 
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Sources will be added here
target_sources(dsp_engine PRIVATE
    src/FilterPrimitives.cpp
    src/ParametricEQ.cpp
    src/GateExpander.cpp
    src/Compressor.cpp
    src/DeEsser.cpp
    src/GraphicEQ.cpp
    src/FeedbackSuppressor.cpp
    src/Limiter.cpp
    src/Crossover.cpp
    src/DelayLine.cpp
    src/ReverbDelay.cpp
    src/Panner.cpp
    src/Metering.cpp
    src/MixingEngine.cpp
)

target_link_libraries(dsp_engine 
    PUBLIC 
        juce::juce_audio_basics
        juce::juce_dsp
)

# Add Unit Tests executable
enable_testing()
add_executable(dsp_engine_tests)

target_sources(dsp_engine_tests PRIVATE
    tests/MainTests.cpp
    tests/FilterPrimitivesTests.cpp
    tests/ParametricEQTests.cpp
    tests/GateExpanderTests.cpp
    tests/CompressorTests.cpp
    tests/DeEsserTests.cpp
    tests/GraphicEQTests.cpp
    tests/FeedbackSuppressorTests.cpp
    tests/LimiterTests.cpp
    tests/CrossoverTests.cpp
    tests/DelayLineTests.cpp
    tests/ReverbDelayTests.cpp
    tests/PannerTests.cpp
    tests/MeteringTests.cpp
    tests/MixingEngineTests.cpp
)

target_link_libraries(dsp_engine_tests 
    PRIVATE 
        dsp_engine
        juce::juce_audio_basics
)

add_test(NAME dsp_engine_tests COMMAND dsp_engine_tests)
