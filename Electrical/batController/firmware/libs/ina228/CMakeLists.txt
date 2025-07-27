add_library(ina228 STATIC
    ina228.c
)

target_include_directories(ina228 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ina228 pico_stdlib hardware_i2c)


