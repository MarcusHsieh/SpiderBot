#include "Cpu.h"
#include "Events.h"
#include "Pins1.h"
#include "CI2C1.h"
#include "FX1.h"
#include "WAIT3.h"
#include "GI2C1.h"
#include "GPIOC.h"
#include "AD1.h"
#include "CsIO1.h"
#include "IO1.h"
#include "GPIO1.h"
#include "SM1.h"
#include "WAIT1.h"
#include "MCUC1.h"
#include "WAIT2.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "PDD_Includes.h"
#include "Init_Config.h"
#include "spi_servo_control.h"
#include "mpu6050.h"
#include "mahony.h"
#include "rgb_led_control.h"
#include "photoresistor.h"

static SpiServo_Handle servo_handle;

int main(void)
{
    /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
    PE_low_level_init();
    /*** End of Processor Expert internal initialization.                    ***/

    LDD_TDeviceData *gpio_device = GPIO1_Init(NULL);
    if (gpio_device != NULL) {
        GPIO1_SetFieldBits(gpio_device, LED_RED, 1);
    }

    IO1_Init(NULL);
    SpiServo_DebugPuts("\r\n--- SpiderBot K64F Controller ---\r\n");

    SpiServo_DebugPuts("Using GI2C1 component for I2C.\r\n");
    
    SpiServo_DebugPuts("Initializing IMU (MPU6050)...");
    if (mpu6050_init() != MPU_SUCCESS) {
    	SpiServo_DebugPuts("FAILED.\r\n");
        for (;;) {
            if (gpio_device != NULL) {
                GPIO1_ClearFieldBits(gpio_device, LED_RED, 1);
                WAIT1_Waitms(150);
                GPIO1_SetFieldBits(gpio_device, LED_RED, 1);
                WAIT1_Waitms(150);
            }
        }
    }
    
    uint8_t whoami = 0;
    if (mpu6050_read_who_am_i(&whoami) != MPU_SUCCESS || whoami != MPU6050_I2C_ADDR) {
        SpiServo_DebugPuts("WHO_AM_I check FAILED. Read: 0x");
        SpiServo_DebugPrintHex(whoami);
        SpiServo_DebugPuts("\r\n");
        for (;;) {}
    }

    SpiServo_DebugPuts("OK (WHO_AM_I=0x");
    SpiServo_DebugPrintHex(whoami);
    SpiServo_DebugPuts(").\r\n");
    
    SpiServo_DebugPuts("Initializing SPI Servo Controller...");
    SpiServo_Status status = SpiServo_Init(&servo_handle);
    if (status != SPI_SERVO_OK) {
        SpiServo_DebugPuts("FAILED.\r\n");
        for (int i = 0; i < 20; i++) {
            if (gpio_device != NULL) {
                GPIO1_ClearFieldBits(gpio_device, LED_RED, 1);
                WAIT1_Waitms(100);
                GPIO1_SetFieldBits(gpio_device, LED_RED, 1);
                WAIT1_Waitms(100);
            }
        }
        for (;;) {}
    }
    SpiServo_DebugPuts("OK.\r\n");

    SpiServo_DebugPuts("Initializing RGB LEDs...");
    RGBLed_Status rgb_status = RGBLed_Init();
    if (rgb_status != RGB_LED_OK) {
        SpiServo_DebugPuts("FAILED.\r\n");
        for (int i = 0; i < 10; i++) {
            if (gpio_device != NULL) {
                GPIO1_ClearFieldBits(gpio_device, LED_RED, 1);
                WAIT1_Waitms(200);
                GPIO1_SetFieldBits(gpio_device, LED_RED, 1);
                WAIT1_Waitms(200);
            }
        }
        for (;;) {}
    }
    SpiServo_DebugPuts("OK.\r\n");

    SpiServo_DebugPuts("Initializing Photo Resistor...");
    if (PhotoRes_Init() != 0) {
        SpiServo_DebugPuts("FAILED.\r\n");
        for (int i = 0; i < 15; i++) {
            if (gpio_device != NULL) {
                GPIO1_ClearFieldBits(gpio_device, LED_RED, 1);
                WAIT1_Waitms(150);
                GPIO1_SetFieldBits(gpio_device, LED_RED, 1);
                WAIT1_Waitms(150);
            }
        }
        for (;;) {}
    }
    SpiServo_DebugPuts("OK.\r\n");

    //SUCCESSFUL LAUNCH
    if (gpio_device != NULL) {
        GPIO1_ClearFieldBits(gpio_device, LED_RED, 1);
    }
    
    WAIT1_Waitms(1000);
    SpiServo_DebugPuts("\r\n=== SpiderBot Control ===\r\n");
    SpiServo_DebugPuts("Type 'help' for available commands\r\n");
    SpiServo_DebugPuts("SpiderBot> ");
    for (;;) {
        SpiServo_ProcessUartCommand(&servo_handle, NULL);

        WAIT1_Waitms(10);
    }

    /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
}
