//  Import in C mode to avoid C++ name mangling
extern "C" {
  #include "can2040.h"
}

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "PDLS_EXT3_Basic_Fast.h"
#include "hV_HAL_Peripherals.h"
#include "hV_Configuration.h"
#include "arduToPico.h"

Screen_EPD_EXT3_Fast myScreen(eScreen_EPD_266_PS_0C, boardRaspberryPiPico_RP2040);


// Determine if the target is an rp2350
#ifdef PICO_RP2350
  #include "RP2350.h"
#else
  #include "RP2040.h"
#endif

void performTest()
{
    uint32_t chrono = 0;

    myScreen.clear();
    myScreen.setOrientation(ORIENTATION_LANDSCAPE);

    uint16_t x = myScreen.screenSizeX();
    uint16_t y = myScreen.screenSizeY();
    uint16_t dx = 0;
    uint16_t dy = 0;
    uint16_t dz = y / 2;
    std::string text = "";

    myScreen.selectFont(Font_Terminal12x16);

    // 0
    dy = (dz - myScreen.characterSizeY()) / 2;
    text = myScreen.WhoAmI();
    printf("%c",text);
    dx = (x - myScreen.stringSizeX(text)) / 2;
    myScreen.gText(dx, dy, text);
    myScreen.dRectangle(0, dz * 0, x, dz, myColours.black);

    sleep_ms(200);
    myScreen.flush();
    sleep_ms(200);
    
    // 1
    dy += dz;
    // text = formatString("Global update= %i ms", chrono);
    text = formatString("Fast update= %i ms", chrono);
    printf("%c", text);
    dx = (x - myScreen.stringSizeX(text)) / 2;
    myScreen.gText(dx, dy, text);
    myScreen.dRectangle(0, dz * 1, x, dz, myColours.black);

    myScreen.flush();
}

void setup()
{
    delay(500);
    myScreen.begin();
    myScreen.clear();
    performTest();
    sleep_ms(8000);
    myScreen.regenerate();
}


int main(){
  //Needed for printf
  stdio_init_all();
  setup();

  setup();
  while (true) {
    // Main loop 
    myScreen.clear();
    performTest();  

    sleep_ms(2000);  // Sleep for 2 second
  }
 

  return 0;
}