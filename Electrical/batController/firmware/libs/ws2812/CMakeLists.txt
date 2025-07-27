
add_library(ws2812 STATIC
    ws2812.pio.c
)

target_include_directories(ws2812 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ws2812 pico_stdlib hardware_pio)

