#ifndef SPI_SERVO_CONTROL_H_
#define SPI_SERVO_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>
#include "PE_Types.h"

//constraints
#define SERVO_ANGLE_MIN     15
#define SERVO_ANGLE_MAX     158
#define SERVO_CHANNELS_MAX  16


typedef enum {
    SPI_SERVO_OK = 0,
    SPI_SERVO_ERROR,
    SPI_SERVO_INVALID_CHANNEL,
    SPI_SERVO_INVALID_ANGLE,
    SPI_SERVO_SPI_ERROR
} SpiServo_Status;

typedef struct {
    bool initialized;
    uint8_t current_angles[SERVO_CHANNELS_MAX]; //last angles
    LDD_TDeviceData *spi_device; //SPI
} SpiServo_Handle;


//initialize SPI master comms
SpiServo_Status SpiServo_Init(SpiServo_Handle *handle);

//specific angle + channel cmd
SpiServo_Status SpiServo_SetAngle(SpiServo_Handle *handle, uint8_t channel, uint8_t angle);


//get last angle for a channel
uint8_t SpiServo_GetAngle(SpiServo_Handle *handle, uint8_t channel);

SpiServo_Status SpiServo_TestSpiComm(SpiServo_Handle *handle);

SpiServo_Status SpiServo_Stand(SpiServo_Handle *handle);

SpiServo_Status SpiServo_StandStraight(SpiServo_Handle *handle);

SpiServo_Status SpiServo_Lay(SpiServo_Handle *handle);

SpiServo_Status SpiServo_Stop(SpiServo_Handle *handle);

SpiServo_Status SpiServo_StandSimultaneous(SpiServo_Handle *handle);

SpiServo_Status SpiServo_StandStraightSimultaneous(SpiServo_Handle *handle);

SpiServo_Status SpiServo_LaySimultaneous(SpiServo_Handle *handle);

SpiServo_Status SpiServo_StandMaxSimultaneous(SpiServo_Handle *handle);

SpiServo_Status SpiServo_Park(SpiServo_Handle *handle);

SpiServo_Status SpiServo_ProcessUartCommand(SpiServo_Handle *handle, void *walk_controller);

SpiServo_Status SpiServo_WalkForwardStep(SpiServo_Handle *handle);

SpiServo_Status SpiServo_RotateC(SpiServo_Handle *handle);

SpiServo_Status SpiServo_StopWalking(SpiServo_Handle *handle);

SpiServo_Status SpiServo_Rotate90C(SpiServo_Handle *handle);

void SpiServo_DebugPuts(const char* str);
void SpiServo_DebugPutChar(char ch);
void SpiServo_DebugPrintDec(uint32_t value);
void SpiServo_DebugPrintHex(uint8_t value);

extern volatile bool spi_tx_complete;
extern volatile bool spi_rx_complete;

#endif /* SPI_SERVO_CONTROL_H_ */
