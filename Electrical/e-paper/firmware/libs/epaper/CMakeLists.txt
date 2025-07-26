add_library(epaper STATIC
    hV_Board.cpp
    hV_Colours565.cpp
    hV_Font_Terminal.cpp
    hV_HAL_Peripherals.cpp
    hV_Screen_Buffer.cpp
    hV_Utilities_Common.cpp
    hV_Utilities_PDLS.cpp
    Screen_EPD_EXT3.cpp
)

target_include_directories(epaper PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(
                    epaper 
                    pico_stdlib
                    string_type_lib
                    hardware_spi
                    )
