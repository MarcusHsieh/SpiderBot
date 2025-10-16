#ifndef RGB_LED_CONTROL_H
#define RGB_LED_CONTROL_H

#include "PE_Types.h"
#include "GPIOC.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RGB_LED_OK = 0,
    RGB_LED_ERROR,
    RGB_LED_NOT_INITIALIZED
} RGBLed_Status;

typedef enum {
    RGB_LED_1 = 1,
    RGB_LED_2 = 2,
    RGB_LED_BOTH = 3
} RGBLed_Selection;

typedef enum {
    RGB_COLOR_OFF = 0,      //R=0, G=0, B=0 (All OFF)
    RGB_COLOR_RED,          //R=1, G=0, B=0 (Red ON)
    RGB_COLOR_GREEN,        //R=0, G=1, B=0 (Green ON)
    RGB_COLOR_BLUE,         //R=0, G=0, B=1 (Blue ON)
    RGB_COLOR_YELLOW,       //R=1, G=1, B=0 (Red+Green ON)
    RGB_COLOR_MAGENTA,      //R=1, G=0, B=1 (Red+Blue ON)
    RGB_COLOR_CYAN,         //R=0, G=1, B=1 (Green+Blue ON)
    RGB_COLOR_WHITE         //R=1, G=1, B=1 (All ON)
} RGBLed_Color;

//RGB Color Structure
typedef struct {
    uint8_t red;    //1 = ON, 0 = OFF
    uint8_t green;  //1 = ON, 0 = OFF
    uint8_t blue;   //1 = ON, 0 = OFF
} RGBLed_RGB;

RGBLed_Status RGBLed_Init(void);
RGBLed_Status RGBLed_SetColor(RGBLed_Selection led, RGBLed_Color color);
RGBLed_Status RGBLed_SetRGB(RGBLed_Selection led, RGBLed_RGB rgb);
RGBLed_Status RGBLed_TurnOff(RGBLed_Selection led);
RGBLed_Status RGBLed_GetRGB(RGBLed_Selection led, RGBLed_RGB *rgb);
RGBLed_Status RGBLed_ColorToRGB(RGBLed_Color color, RGBLed_RGB *rgb);

const char* RGBLed_GetColorName(RGBLed_Color color);

RGBLed_Status RGBLed_ProcessCommand(const char* command);
void RGBLed_PrintHelp(void);
RGBLed_Status RGBLed_TestSequence(uint16_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* RGB_LED_CONTROL_H */ 