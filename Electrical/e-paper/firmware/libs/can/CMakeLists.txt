add_library(can2040 STATIC
    can2040.c
)

target_include_directories(can2040 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(can2040 pico_stdlib)
