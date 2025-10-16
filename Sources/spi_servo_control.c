#include <stdio.h>
#include <string.h>
#include "spi_servo_control.h"
#include "SM1.h"
#include "IO1.h"
#include "UART_PDD.h"
#include "WAIT1.h"
#include "IO_Map.h"
#include "mpu6050.h"
#include "mahony.h"
#include "rgb_led_control.h"
#include "photoresistor.h"
#include <math.h>

//mpu-6050
#define ACCEL_SENSITIVITY 16384.0f //lsb/g for +/- 2g range
#define GYRO_SENSITIVITY 131.0f   //lsb/dps for +/- 250dps range
#define PI 3.14159265359f

//global variables for spi sync
volatile bool spi_tx_complete = false;
volatile bool spi_rx_complete = false;

static void spi_delay(uint32_t delay_cycles) {
    volatile uint32_t i;
    for (i = 0; i < delay_cycles; i++) {
        __asm("nop");
    }
}

void SpiServo_DebugPutChar(char ch) {
    while (!UART_PDD_GetTxCompleteStatus(UART0_BASE_PTR)) {
        spi_delay(10);
    }
    
    UART_PDD_PutChar8(UART0_BASE_PTR, (uint8_t)ch);
    
    while (!UART_PDD_GetTxCompleteStatus(UART0_BASE_PTR)) {
        spi_delay(10);
    }
}

void SpiServo_DebugPuts(const char* str) {
    if (str == NULL) return;
    
    while (*str) {
        SpiServo_DebugPutChar(*str);
        str++;
    }
}

void SpiServo_DebugPrintDec(uint32_t value) {
    char buffer[12];
    char *p = buffer + sizeof(buffer) - 1;
    *p = '\0';
    
    if (value == 0) {
        *(--p) = '0';
    } else {
        while (value > 0) {
            *(--p) = '0' + (value % 10);
            value /= 10;
        }
    }
    
    SpiServo_DebugPuts(p);
}

void SpiServo_DebugPrintFloat(const char* label, float value) {
    char p_sign = (value < 0) ? '-' : ' ';
    float p_abs = fabsf(value);
    SpiServo_DebugPuts(label);
    SpiServo_DebugPuts(": ");
    SpiServo_DebugPutChar(p_sign);
    SpiServo_DebugPrintDec((int)p_abs);
    SpiServo_DebugPutChar('.');
    SpiServo_DebugPrintDec((int)((p_abs - (int)p_abs) * 100));
}

void SpiServo_DebugPrintHex(uint8_t value) {
    char buffer[4];
    const char hex_chars[] = "0123456789ABCDEF";
    
    buffer[0] = hex_chars[(value >> 4) & 0x0F];
    buffer[1] = hex_chars[value & 0x0F];
    buffer[2] = '\0';
    
    SpiServo_DebugPuts(buffer);
}

static uint8_t constrain_angle(uint8_t angle) {
    if (angle < SERVO_ANGLE_MIN) return SERVO_ANGLE_MIN;
    if (angle > SERVO_ANGLE_MAX) return SERVO_ANGLE_MAX;
    return angle;
}

//3-byte spi cmd [mode, channel, angle] + receive response
static SpiServo_Status send_spi_command(SpiServo_Handle *handle, uint8_t channel, uint8_t angle) {
    if (!handle || !handle->spi_device) {
        return SPI_SERVO_SPI_ERROR;
    }
    
    uint8_t command[3] = {0x4E, channel, angle}; //keeping at nonsync
    uint8_t response[3];
    
    SpiServo_DebugPuts("[SPI] Sending: CH");
    SpiServo_DebugPrintDec(channel);
    SpiServo_DebugPuts(" = ");
    SpiServo_DebugPrintDec(angle);
    SpiServo_DebugPuts("deg\r\n");
    
    SpiServo_DebugPuts("[SPI] Command sent: ");
    for (int i = 0; i < 3; i++) {
        SpiServo_DebugPrintHex(command[i]);
        if (i < 2) SpiServo_DebugPuts(" ");
    }
    SpiServo_DebugPuts("\r\n");

    spi_tx_complete = false;
    spi_rx_complete = false;
    
    LDD_TError rx_error = SM1_ReceiveBlock(handle->spi_device, response, 3);
    if (rx_error != ERR_OK) {
        SpiServo_DebugPuts("[SPI] ERROR: ReceiveBlock setup failed\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    WAIT1_Waitms(1);
    
    LDD_TError tx_error = SM1_SendBlock(handle->spi_device, command, 3);
    if (tx_error != ERR_OK) {
        SpiServo_DebugPuts("[SPI] ERROR: SendBlock failed\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    uint16_t tx_timeout = 100;
    while (!spi_tx_complete && tx_timeout > 0) {
        WAIT1_Waitms(1);
        tx_timeout--;
    }
    
    if (!spi_tx_complete) {
        SpiServo_DebugPuts("[SPI] ERROR: Transmission timeout\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    uint16_t rx_timeout = 100;
    while (!spi_rx_complete && rx_timeout > 0) {
        WAIT1_Waitms(1);
        rx_timeout--;
    }
    
    if (!spi_rx_complete) {
        SpiServo_DebugPuts("[SPI] ERROR: Reception timeout\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    SpiServo_DebugPuts("[SPI] Response received: ");
    for (int i = 0; i < 3; i++) {
        SpiServo_DebugPrintHex(response[i]);
        if (i < 2) SpiServo_DebugPuts(" ");
    }
    SpiServo_DebugPuts("\r\n");
    
    return SPI_SERVO_OK;
}

SpiServo_Status SpiServo_Init(SpiServo_Handle *handle) {
    if (!handle) return SPI_SERVO_ERROR;
    
    SpiServo_DebugPuts("\r\n=== SPI Servo Control Initialization ===\r\n");
    
    handle->spi_device = SM1_Init(NULL);
    if (!handle->spi_device) {
        SpiServo_DebugPuts("[SPI] Failed to initialize SPI master\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    SpiServo_DebugPuts("[SPI] SPI master initialized successfully\r\n");
    SpiServo_DebugPuts("[SPI] Pin mapping:\r\n");
    SpiServo_DebugPuts("      CS   (PTD0) - J2_6\r\n");
    SpiServo_DebugPuts("      SCK  (PTD1) - J2_12\r\n");
    SpiServo_DebugPuts("      MOSI (PTD2) - J2_8\r\n");
    SpiServo_DebugPuts("      MISO (PTD3) - J2_10\r\n");
    
    for (uint8_t i = 0; i < SERVO_CHANNELS_MAX; i++) {
        handle->current_angles[i] = 90;
    }
    
    handle->initialized = true;
    
    WAIT1_Waitms(2000);
    
    SpiServo_DebugPuts("[SPI] System ready\r\n");
    
    return SPI_SERVO_OK;
}

SpiServo_Status SpiServo_SetAngle(SpiServo_Handle *handle, uint8_t channel, uint8_t angle) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    if (channel >= SERVO_CHANNELS_MAX) {
        return SPI_SERVO_INVALID_CHANNEL;
    }
    
    angle = constrain_angle(angle);
    
    SpiServo_Status status = send_spi_command(handle, channel, angle);
    if (status == SPI_SERVO_OK) {
        handle->current_angles[channel] = angle;
    }
    
    return status;
}


uint8_t SpiServo_GetAngle(SpiServo_Handle *handle, uint8_t channel) {
    if (!handle || channel >= SERVO_CHANNELS_MAX) {
        return 0;
    }
    return handle->current_angles[channel];
}

SpiServo_Status SpiServo_TestSpiComm(SpiServo_Handle *handle) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    SpiServo_DebugPuts("\r\n=== SPI COMMS TEST ===\r\n");
    SpiServo_DebugPuts("K64F to ESP32\r\n\r\n");

    SpiServo_DebugPuts("=== TEST 1: Basic SPI Hardware Test ===\r\n");
    
    uint8_t test_cmd[3] = {0xAA, 0x55, 0xFF};
    uint8_t test_resp[3] = {0x00, 0x00, 0x00};

    spi_tx_complete = false;
    spi_rx_complete = false;

    LDD_TError rx_error = SM1_ReceiveBlock(handle->spi_device, test_resp, 3);
    if (rx_error != ERR_OK) {
        SpiServo_DebugPuts("FAIL: ReceiveBlock setup failed\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    WAIT1_Waitms(1);

    LDD_TError tx_error = SM1_SendBlock(handle->spi_device, test_cmd, 3);
    if (tx_error != ERR_OK) {
        SpiServo_DebugPuts("FAIL: SendBlock failed\r\n");
        return SPI_SERVO_SPI_ERROR;
    }

    uint16_t timeout = 500;
    while ((!spi_tx_complete || !spi_rx_complete) && timeout > 0) {
        WAIT1_Waitms(1);
        timeout--;
    }
    
    if (!spi_tx_complete || !spi_rx_complete) {
        SpiServo_DebugPuts("FAIL: SPI transaction timeout\r\n");
        SpiServo_DebugPuts("TX Complete: ");
        SpiServo_DebugPuts(spi_tx_complete ? "YES" : "NO");
        SpiServo_DebugPuts(", RX Complete: ");
        SpiServo_DebugPuts(spi_rx_complete ? "YES" : "NO");
        SpiServo_DebugPuts("\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    SpiServo_DebugPuts("PASS: SPI hardware transaction completed\r\n");
    SpiServo_DebugPuts("Sent: [");
    for (int i = 0; i < 3; i++) {
        SpiServo_DebugPrintHex(test_cmd[i]);
        if (i < 2) SpiServo_DebugPuts(", ");
    }
    SpiServo_DebugPuts("]\r\n");
    SpiServo_DebugPuts("Received: [");
    for (int i = 0; i < 3; i++) {
        SpiServo_DebugPrintHex(test_resp[i]);
        if (i < 2) SpiServo_DebugPuts(", ");
    }
    SpiServo_DebugPuts("]\r\n\r\n");

    SpiServo_DebugPuts("=== TEST 2: ESP32 Protocol Test ===\r\n");
    SpiServo_DebugPuts("Servo cmd\r\n");
    
    SpiServo_Status status = send_spi_command(handle, 0, 90);
    if (status == SPI_SERVO_OK) {
        SpiServo_DebugPuts("PASS: Servo command protocol test completed\r\n");
    } else {
        SpiServo_DebugPuts("FAIL: Servo command protocol test failed\r\n");
    }
    
    SpiServo_DebugPuts("=== TEST 3: Rapid Command Test ===\r\n");
    
    for (int i = 0; i < 3; i++) {
        SpiServo_DebugPuts("Command ");
        SpiServo_DebugPrintDec(i + 1);
        SpiServo_DebugPuts("/3: ");
        
        SpiServo_Status rapid_status = send_spi_command(handle, i, 45 + i * 45);
        if (rapid_status == SPI_SERVO_OK) {
            SpiServo_DebugPuts("PASS\r\n");
        } else {
            SpiServo_DebugPuts("FAIL\r\n");
        }
        WAIT1_Waitms(200);
    }

    return SPI_SERVO_OK;
}


//pose cmds
SpiServo_Status SpiServo_Stand(SpiServo_Handle *handle) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    SpiServo_DebugPuts("\r\n=== SpiderBot Stand Position ===\r\n");
    SpiServo_DebugPuts("Setting all legs perpendicular to octagon: yaw=90deg, pitch=45deg\r\n");

    for (uint8_t leg = 0; leg < 6; leg++) {
        uint8_t yaw_channel = leg * 2; //0,2,4,6,8,10
        uint8_t pitch_channel = leg * 2 + 1; //1,3,5,7,9,11
        
        SpiServo_DebugPuts("Leg ");
        SpiServo_DebugPrintDec(leg);
        SpiServo_DebugPuts(": YAW CH");
        SpiServo_DebugPrintDec(yaw_channel);
        SpiServo_DebugPuts("=90deg, PITCH CH");
        SpiServo_DebugPrintDec(pitch_channel);
        SpiServo_DebugPuts("=45deg\r\n");
        
        SpiServo_Status status = SpiServo_SetAngle(handle, yaw_channel, 90);
        if (status != SPI_SERVO_OK) return status;
        WAIT1_Waitms(100);
        
        status = SpiServo_SetAngle(handle, pitch_channel, 45);
        if (status != SPI_SERVO_OK) return status;
        WAIT1_Waitms(100);
    }
    
    SpiServo_DebugPuts("Stand position complete\r\n");
    return SPI_SERVO_OK;
}

SpiServo_Status SpiServo_StandSimultaneous(SpiServo_Handle *handle) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    SpiServo_DebugPuts("\r\n=== SpiderBot Stand Position (Simultaneous) ===\r\n");
    SpiServo_DebugPuts("Sending stand command...\r\n");
    
    //channel 251
    SpiServo_Status status = send_spi_command(handle, 251, 0);
    if (status == SPI_SERVO_OK) {
        for (uint8_t leg = 0; leg < 6; leg++) {
            handle->current_angles[leg * 2] = 90;//yaw ch
            handle->current_angles[leg * 2 + 1] = 45; //pitch ch
        }
        SpiServo_DebugPuts("Stand position command sent successfully\r\n");
    } else {
        SpiServo_DebugPuts("Failed to send stand command\r\n");
    }
    
    return status;
}

SpiServo_Status SpiServo_StandStraight(SpiServo_Handle *handle) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    SpiServo_DebugPuts("\r\n=== SpiderBot Straight Stand Position ===\r\n");
    SpiServo_DebugPuts("Setting legs parallel\r\n");

    uint8_t yaw_angles[6] = {135, 90, 45, 45, 90, 135};
    uint16_t leg_positions[6] = {45, 0, 315, 225, 180, 135};
    
    for (uint8_t leg = 0; leg < 6; leg++) {
        uint8_t yaw_channel = leg * 2;
        uint8_t pitch_channel = leg * 2 + 1;
        
        SpiServo_DebugPuts("Leg ");
        SpiServo_DebugPrintDec(leg);
        SpiServo_DebugPuts(" (@");
        SpiServo_DebugPrintDec(leg_positions[leg]);
        SpiServo_DebugPuts("deg): YAW CH");
        SpiServo_DebugPrintDec(yaw_channel);
        SpiServo_DebugPuts("=");
        SpiServo_DebugPrintDec(yaw_angles[leg]);
        SpiServo_DebugPuts("deg, PITCH CH");
        SpiServo_DebugPrintDec(pitch_channel);
        SpiServo_DebugPuts("=45deg\r\n");
        
        SpiServo_Status status = SpiServo_SetAngle(handle, yaw_channel, yaw_angles[leg]);
        if (status != SPI_SERVO_OK) return status;
        WAIT1_Waitms(100);
        
        status = SpiServo_SetAngle(handle, pitch_channel, 45);
        if (status != SPI_SERVO_OK) return status;
        WAIT1_Waitms(100);
    }
    
    SpiServo_DebugPuts("Straight stand position complete\r\n");
    return SPI_SERVO_OK;
}

SpiServo_Status SpiServo_StandStraightSimultaneous(SpiServo_Handle *handle) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    SpiServo_DebugPuts("\r\n=== SpiderBot Straight Stand Position ===\r\n");
    SpiServo_DebugPuts("Sending straight stand command...\r\n");
    
    //channel 252
    SpiServo_Status status = send_spi_command(handle, 252, 0);
    if (status == SPI_SERVO_OK) {
        uint8_t yaw_angles[6] = {135, 90, 45, 45, 90, 135};
        for (uint8_t leg = 0; leg < 6; leg++) {
            handle->current_angles[leg * 2] = yaw_angles[leg];
            handle->current_angles[leg * 2 + 1] = 45;
        }
        SpiServo_DebugPuts("Straight stand position command sent successfully\r\n");
    } else {
        SpiServo_DebugPuts("Failed to send straight stand command\r\n");
    }
    
    return status;
}

SpiServo_Status SpiServo_Lay(SpiServo_Handle *handle) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    SpiServo_DebugPuts("\r\n=== SpiderBot Lay Position ===\r\n");
    SpiServo_DebugPuts("Setting all legs to rest at base level: yaw=90deg, pitch=74deg\r\n");
    
    for (uint8_t leg = 0; leg < 6; leg++) {
        uint8_t yaw_channel = leg * 2;
        uint8_t pitch_channel = leg * 2 + 1;
        
        SpiServo_DebugPuts("Leg ");
        SpiServo_DebugPrintDec(leg);
        SpiServo_DebugPuts(": YAW CH");
        SpiServo_DebugPrintDec(yaw_channel);
        SpiServo_DebugPuts("=90deg, PITCH CH");
        SpiServo_DebugPrintDec(pitch_channel);
        SpiServo_DebugPuts("=74deg\r\n");
        
        SpiServo_Status status = SpiServo_SetAngle(handle, yaw_channel, 90);
        if (status != SPI_SERVO_OK) return status;
        WAIT1_Waitms(100);
        
        status = SpiServo_SetAngle(handle, pitch_channel, 74);
        if (status != SPI_SERVO_OK) return status;
        WAIT1_Waitms(100);
    }
    
    SpiServo_DebugPuts("Lay position complete\r\n");
    return SPI_SERVO_OK;
}

SpiServo_Status SpiServo_LaySimultaneous(SpiServo_Handle *handle) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    SpiServo_DebugPuts("\r\n=== SpiderBot Lay Position (Simultaneous) ===\r\n");
    SpiServo_DebugPuts("Sending simultaneous lay command...\r\n");
    
    //channel 253
    SpiServo_Status status = send_spi_command(handle, 253, 0);
    if (status == SPI_SERVO_OK) {
        for (uint8_t leg = 0; leg < 6; leg++) {
            handle->current_angles[leg * 2] = 90;
            handle->current_angles[leg * 2 + 1] = 74;
        }
        SpiServo_DebugPuts("Lay position command sent successfully\r\n");
    } else {
        SpiServo_DebugPuts("Failed to send lay command\r\n");
    }
      return status;
}

SpiServo_Status SpiServo_StandMaxSimultaneous(SpiServo_Handle *handle) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    SpiServo_DebugPuts("\r\n=== SpiderBot Stand Max Position (Simultaneous) ===\r\n");
    SpiServo_DebugPuts("Sending simultaneous stand max command...\r\n");
    
    //channel 254
    SpiServo_Status status = send_spi_command(handle, 254, 0);
    if (status == SPI_SERVO_OK) {
        for (uint8_t leg = 0; leg < 6; leg++) {
            handle->current_angles[leg * 2] = 90;
            handle->current_angles[leg * 2 + 1] = 35;
        }
        SpiServo_DebugPuts("Stand max position command sent successfully\r\n");
    } else {
        SpiServo_DebugPuts("Failed to send stand max command\r\n");
    }
    
    return status;
}

SpiServo_Status SpiServo_Stop(SpiServo_Handle *handle) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    SpiServo_DebugPuts("\r\n=== Emergency Stop ===\r\n");
    SpiServo_DebugPuts("Sending stop command...\r\n");
    
    //channel 250
    SpiServo_Status status = send_spi_command(handle, 250, 0);
    if (status == SPI_SERVO_OK) {
        SpiServo_DebugPuts("Emergency stop command sent successfully\r\n");
    } else {
        SpiServo_DebugPuts("Failed to send emergency stop command\r\n");
    }
      return status;
}

//to decrease the jitter in pitch stop sending PWM there
SpiServo_Status SpiServo_Park(SpiServo_Handle *handle) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    SpiServo_DebugPuts("\r\n=== Park YAW Servos ===\r\n");
    SpiServo_DebugPuts("Disabling YAW servo PWM to prevent oscillation (PITCH servos stay active)...\r\n");
    
    //channel 255
    SpiServo_Status status = send_spi_command(handle, 255, 0);
    if (status == SPI_SERVO_OK) {
        SpiServo_DebugPuts("Park command sent - YAW servos should stop oscillating\r\n");
    } else {
        SpiServo_DebugPuts("Failed to send park command\r\n");
    }
      return status;
}

static uint8_t parse_number(const char* str, uint8_t start_pos, uint8_t max_len) {
    uint8_t result = 0;
    uint8_t pos = start_pos;
    
    while (pos < max_len && str[pos] >= '0' && str[pos] <= '9') {
        result = result * 10 + (str[pos] - '0');
        pos++;
    }
    
    return result;
}

static bool read_uart_char(char* ch, uint32_t timeout_ms) {
    uint32_t timeout_count = timeout_ms * 1000;
    
    while (timeout_count > 0) {
        if (UART_PDD_ReadStatus1Reg(UART0_BASE_PTR) & UART_PDD_RX_DATA_FULL_FLAG) {
            *ch = (char)UART_PDD_GetChar8(UART0_BASE_PTR);
            return true;
        }
        spi_delay(100);
        timeout_count--;
    }
    
    return false;
}

SpiServo_Status SpiServo_ProcessUartCommand(SpiServo_Handle *handle, void *walk_controller_ptr) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    (void)walk_controller_ptr;
    
    static char command_buffer[32];
    static uint8_t buffer_pos = 0;
    char ch;
    
    if (!read_uart_char(&ch, 1)) {
        return SPI_SERVO_OK;
    }
    
    SpiServo_DebugPutChar(ch);
    
    if (ch == '\b' || ch == 127) {
        if (buffer_pos > 0) {
            buffer_pos--;
            SpiServo_DebugPuts(" \b");
        }
        return SPI_SERVO_OK;
    }
    
    if (ch == '\r' || ch == '\n') {
        SpiServo_DebugPuts("\r\n");
        
        if (buffer_pos == 0) {
            SpiServo_DebugPuts("SpiderBot> ");
            return SPI_SERVO_OK;
        }
        
        command_buffer[buffer_pos] = '\0';
        
        if (command_buffer[0] == 'c' && buffer_pos >= 3) {
            uint8_t channel = parse_number(command_buffer, 1, buffer_pos);
            uint8_t dash_pos = 0;
            for (uint8_t i = 1; i < buffer_pos; i++) {
                if (command_buffer[i] == '-') {
                    dash_pos = i;
                    break;
                }
            }
            if (dash_pos > 0 && dash_pos < buffer_pos - 1) {
                uint8_t angle = parse_number(command_buffer, dash_pos + 1, buffer_pos);
                if (channel <= 11 && angle >= 15 && angle <= 158) {
                    SpiServo_DebugPuts("Setting channel ");
                    SpiServo_DebugPrintDec(channel);
                    SpiServo_DebugPuts(" to ");
                    SpiServo_DebugPrintDec(angle);
                    SpiServo_DebugPuts("deg\r\n");
                    SpiServo_Status status = SpiServo_SetAngle(handle, channel, angle);
                    if (status == SPI_SERVO_OK) {
                        SpiServo_DebugPuts("Command executed successfully\r\n");
                    } else {
                        SpiServo_DebugPuts("Error executing command\r\n");
                    }
                } else {
                    SpiServo_DebugPuts("Invalid channel (0-11) or angle (15-158)\r\n");
                }
            } else {
                SpiServo_DebugPuts("Invalid format. Use: c#-angle (e.g., c0-90)\r\n");
            }
        }
        else if (buffer_pos == 5 && //stand
                 command_buffer[0] == 's' && command_buffer[1] == 't' && 
                 command_buffer[2] == 'a' && command_buffer[3] == 'n' && 
                 command_buffer[4] == 'd') {
            SpiServo_DebugPuts("Executing stand position...\r\n");
            SpiServo_StandSimultaneous(handle);
        }
        else if (buffer_pos == 8 && //standmax
                 command_buffer[0] == 's' && command_buffer[1] == 't' && 
                 command_buffer[2] == 'a' && command_buffer[3] == 'n' && 
                 command_buffer[4] == 'd' && command_buffer[5] == 'm' &&
                 command_buffer[6] == 'a' && command_buffer[7] == 'x') {
            SpiServo_DebugPuts("Executing stand max position...\r\n");
            SpiServo_StandMaxSimultaneous(handle);
        }
        else if (buffer_pos == 13 && //standstraight
                 command_buffer[0] == 's' && command_buffer[1] == 't' && 
                 command_buffer[2] == 'a' && command_buffer[3] == 'n' && 
                 command_buffer[4] == 'd' && command_buffer[5] == 's' &&
                 command_buffer[6] == 't' && command_buffer[7] == 'r' &&
                 command_buffer[8] == 'a' && command_buffer[9] == 'i' &&
                 command_buffer[10] == 'g' && command_buffer[11] == 'h' && 
                 command_buffer[12] == 't') {
            SpiServo_DebugPuts("Executing straight stand position...\r\n");
            SpiServo_StandStraightSimultaneous(handle);
        }
        else if (buffer_pos == 3 && //lay
                 command_buffer[0] == 'l' && command_buffer[1] == 'a' && 
                 command_buffer[2] == 'y') {
            SpiServo_DebugPuts("Executing lay position...\r\n");
            SpiServo_LaySimultaneous(handle);
        }
        else if (buffer_pos == 4 && //stop
                 command_buffer[0] == 's' && command_buffer[1] == 't' && 
                 command_buffer[2] == 'o' && command_buffer[3] == 'p') {
            SpiServo_DebugPuts("Executing emergency stop...\r\n");
            SpiServo_Stop(handle);
        }
        else if (buffer_pos == 4 && //park
                 command_buffer[0] == 'p' && command_buffer[1] == 'a' && 
                 command_buffer[2] == 'r' && command_buffer[3] == 'k') {
            SpiServo_DebugPuts("Parking YAW servos only (keeping PITCH servos active for support)...\r\n");
            SpiServo_Park(handle);
        }
        else if (buffer_pos == 7 && //walkfwd
                 command_buffer[0] == 'w' && command_buffer[1] == 'a' && 
                 command_buffer[2] == 'l' && command_buffer[3] == 'k' && 
                 command_buffer[4] == 'f' && command_buffer[5] == 'w' && 
                 command_buffer[6] == 'd') {
            SpiServo_DebugPuts("Walking forward step...\r\n");
            SpiServo_Status walk_status = SpiServo_WalkForwardStep(handle);
            if (walk_status != SPI_SERVO_OK) {
                SpiServo_DebugPuts("Failed to execute walk forward step\r\n");
            }
        }
        else if (buffer_pos == 7 && //rotatec
                 command_buffer[0] == 'r' && command_buffer[1] == 'o' &&
                 command_buffer[2] == 't' && command_buffer[3] == 'a' &&
                 command_buffer[4] == 't' && command_buffer[5] == 'e' &&
                 command_buffer[6] == 'c') {
            SpiServo_DebugPuts("Rotating clockwise...\r\n");
            SpiServo_Status rotate_status = SpiServo_RotateC(handle);
            if (rotate_status != SPI_SERVO_OK) {
                SpiServo_DebugPuts("Failed to execute rotate clockwise step\r\n");
            }
        }
        else if (buffer_pos == 10 && //rotateleft
                 command_buffer[0] == 'r' && command_buffer[1] == 'o' && 
                 command_buffer[2] == 't' && command_buffer[3] == 'a' && 
                 command_buffer[4] == 't' && command_buffer[5] == 'e' && 
                 command_buffer[6] == 'l' && command_buffer[7] == 'e' && 
                 command_buffer[8] == 'f' && command_buffer[9] == 't') {
            SpiServo_DebugPuts("Skipped\r\n");
        }
        else if (buffer_pos == 8 && //walkstop
                 command_buffer[0] == 'w' && command_buffer[1] == 'a' && 
                 command_buffer[2] == 'l' && command_buffer[3] == 'k' && 
                 command_buffer[4] == 's' && command_buffer[5] == 't' && 
                 command_buffer[6] == 'o' && command_buffer[7] == 'p') {
            SpiServo_DebugPuts("Stopping walk...\r\n");
            SpiServo_Status stop_status = SpiServo_StopWalking(handle);
            if (stop_status != SPI_SERVO_OK) {
                SpiServo_DebugPuts("Failed to execute stop walking\r\n");
            }
        }
        else if (buffer_pos == 9 && //rotate90c
                 command_buffer[0] == 'r' && command_buffer[1] == 'o' && 
                 command_buffer[2] == 't' && command_buffer[3] == 'a' && 
                 command_buffer[4] == 't' && command_buffer[5] == 'e' && 
                 command_buffer[6] == '9' && command_buffer[7] == '0' && 
                 command_buffer[8] == 'c') {
            SpiServo_DebugPuts("Executing 90-degree clockwise rotation...\r\n");
            SpiServo_Status rotate90_status = SpiServo_Rotate90C(handle);
            if (rotate90_status != SPI_SERVO_OK) {
                SpiServo_DebugPuts("Failed to send rotate90c command\r\n");
            }
        }
        else if (buffer_pos == 4 && //help
                 command_buffer[0] == 'h' && command_buffer[1] == 'e' && 
                 command_buffer[2] == 'l' && command_buffer[3] == 'p') {
            SpiServo_DebugPuts("\r\n=== SpiderBot Commands ===\r\n");
            SpiServo_DebugPuts("c#-angle    : Move channel # to angledeg (e.g., c0-90)\r\n");
            SpiServo_DebugPuts("             Channels 0-11, Angles 15-158\r\n");            
            SpiServo_DebugPuts("stand       : Normal stand (yaw=90deg, pitch=45deg)\r\n");
            SpiServo_DebugPuts("standmax    : Maximum safe stand (yaw=90deg, pitch=25deg)\r\n");            
            SpiServo_DebugPuts("standstraight: Straight stand (legs parallel)\r\n");
            SpiServo_DebugPuts("lay         : Lay position (yaw=90deg, pitch=74deg)\r\n");
            SpiServo_DebugPuts("stop        : Emergency stop all servos\r\n");
            SpiServo_DebugPuts("park        : Park YAW servos only (disable PWM, keep PITCH active)\r\n");
            SpiServo_DebugPuts("\r\n=== Walking Commands ===\r\n");
            SpiServo_DebugPuts("walkfwd     : Walk forward one step (tripod gait)\r\n");
            SpiServo_DebugPuts("rotatec     : Rotate clockwise one full sequence\r\n");
            SpiServo_DebugPuts("rotate90c   : Rotate 90 degrees clockwise\r\n");
            SpiServo_DebugPuts("walkstop    : Stop walking and return to center position\r\n");
            SpiServo_DebugPuts("\r\n=== Test Commands ===\r\n");
            SpiServo_DebugPuts("spitest     : Test SPI communication\r\n");
            SpiServo_DebugPuts("\r\n=== Sensor Commands ===\r\n");
            SpiServo_DebugPuts("readimu     : Stream IMU data (roll/pitch/yaw) for 10 seconds\r\n");
            SpiServo_DebugPuts("tofsensors  : Stream VL53L0X (2) ToF distance sensors\r\n");
            SpiServo_DebugPuts("time        : Get time of day based on light sensor (day/night/dawn)\r\n");
            SpiServo_DebugPuts("light       : Show detailed light sensor information\r\n");
            SpiServo_DebugPuts("\r\n=== RGB LED Commands ===\r\n");
            SpiServo_DebugPuts("rgb         : Show RGB LED help\r\n");
            SpiServo_DebugPuts("red/green/blue/yellow/magenta/white/off : Set both LEDs\r\n");
            SpiServo_DebugPuts(".cyan/.magenta : Set both LEDs \r\n");
            SpiServo_DebugPuts("rgb 1 <color> : Set LED 1 to color\r\n");
            SpiServo_DebugPuts("rgb 2 <color> : Set LED 2 to color\r\n");
            SpiServo_DebugPuts("rgb status  : Show current LED colors\r\n");
            SpiServo_DebugPuts("rgb test    : Run color test sequence\r\n");
            SpiServo_DebugPuts("\r\n=== Unsupported Commands ===\r\n");
            SpiServo_DebugPuts("walkback, walkleft, walkright, rotateleft : Not implemented\r\n");
            SpiServo_DebugPuts("Use 'rotatec' to change direction, then 'walkfwd'\r\n");
            SpiServo_DebugPuts("help        : Show this help\r\n");
            SpiServo_DebugPuts("\r\nLeg Layout (6 legs on octagon):\r\n");
            SpiServo_DebugPuts("CH0,1: 45deg   CH2,3: 0deg    CH4,5: 315deg\r\n");
            SpiServo_DebugPuts("CH10,11: 135deg [BOT] CH6,7: 225deg\r\n");
            SpiServo_DebugPuts("                CH8,9: 180deg\r\n");
            SpiServo_DebugPuts("(Even channels=YAW, Odd channels=PITCH)\r\n");
        }
        else if (strcmp(command_buffer, "rot90c") == 0) {
            SpiServo_Rotate90C(handle);
        }
        //spitest
        else if (buffer_pos == 7 && 
                 command_buffer[0] == 's' && command_buffer[1] == 'p' && 
                 command_buffer[2] == 'i' && command_buffer[3] == 't' && 
                 command_buffer[4] == 'e' && command_buffer[5] == 's' && 
                 command_buffer[6] == 't') {
            SpiServo_DebugPuts("Testing SPI comms...\r\n");
            SpiServo_Status test_status = SpiServo_TestSpiComm(handle);
            if (test_status == SPI_SERVO_OK) {
                SpiServo_DebugPuts("SPI test completed successfully\r\n");
            } else {
                SpiServo_DebugPuts("SPI test failed\r\n");
            }
        }
        else if (strcmp(command_buffer, "readimu") == 0) {
            SpiServo_DebugPuts("\r\n--- Reading IMU for 10 seconds ---\r\n");

            SensorDataRaw_t sensorData;
            float ax, ay, az, gx, gy, gz;
            float roll, pitch, yaw;

            Mahony_init();

            for (int i = 0; i < 100; i++) {
                mpu6050_read_raw_data(&sensorData);

                ax = (float)sensorData.accel.x / ACCEL_SENSITIVITY;
                ay = (float)sensorData.accel.y / ACCEL_SENSITIVITY;
                az = (float)sensorData.accel.z / ACCEL_SENSITIVITY;
                gx = (float)sensorData.gyro.x / GYRO_SENSITIVITY;
                gy = (float)sensorData.gyro.y / GYRO_SENSITIVITY;
                gz = (float)sensorData.gyro.z / GYRO_SENSITIVITY;

                Mahony_update(gx * (PI/180.0f), gy * (PI/180.0f), gz * (PI/180.0f), ax, ay, az);

                Mahony_getRollPitchYaw(&roll, &pitch, &yaw);

                roll = roll * (180.0f / PI);
                pitch = pitch * (180.0f / PI);
                yaw = yaw * (180.0f / PI);

                SpiServo_DebugPuts("\r                                                              \r");

                SpiServo_DebugPrintFloat("Roll", roll);
                SpiServo_DebugPuts(", ");
                SpiServo_DebugPrintFloat("Pitch", pitch);
                SpiServo_DebugPuts(", ");
                SpiServo_DebugPrintFloat("Yaw", yaw);

                WAIT1_Waitms(100);
            }
            SpiServo_DebugPuts("\r\n--- IMU Reading Complete ---\r\n");
        }
        //tofsensors
        else if (buffer_pos == 10 && 
                 command_buffer[0] == 't' && command_buffer[1] == 'o' && 
                 command_buffer[2] == 'f' && command_buffer[3] == 's' && 
                 command_buffer[4] == 'e' && command_buffer[5] == 'n' && 
                 command_buffer[6] == 's' && command_buffer[7] == 'o' && 
                 command_buffer[8] == 'r' && command_buffer[9] == 's') {
            SpiServo_DebugPuts("\r\n=== VL53L0X ToF Sensors ===\r\n");

            SpiServo_DebugPuts("Checking sensor status...\r\n");
            uint8_t status_cmd[3] = {0x02, 0x02, 0x00};
            uint8_t status_response[3];

            spi_tx_complete = false;
            spi_rx_complete = false;

            LDD_TError rx_error = SM1_ReceiveBlock(handle->spi_device, status_response, 3);
            if (rx_error != ERR_OK) {
                SpiServo_DebugPuts("[ERROR] Failed to setup receive buffer\r\n");
            } else {
                WAIT1_Waitms(1);
                
                LDD_TError tx_error = SM1_SendBlock(handle->spi_device, status_cmd, 3);
                if (tx_error != ERR_OK) {
                    SpiServo_DebugPuts("[ERROR] Failed to send status command\r\n");
                } else {
                    uint16_t timeout = 100;
                    while ((!spi_tx_complete || !spi_rx_complete) && timeout > 0) {
                        WAIT1_Waitms(1);
                        timeout--;
                    }
                    
                    if (spi_tx_complete && spi_rx_complete) {
                        SpiServo_DebugPuts("Status Response: [");
                        SpiServo_DebugPrintHex(status_response[0]);
                        SpiServo_DebugPuts(", ");
                        SpiServo_DebugPrintHex(status_response[1]);
                        SpiServo_DebugPuts(", ");
                        SpiServo_DebugPrintHex(status_response[2]);
                        SpiServo_DebugPuts("]\r\n");
                        
                        if (status_response[1] == 0x01) {
                            SpiServo_DebugPuts("ToF sensors are ENABLED\r\n");
                            SpiServo_DebugPuts("Number of sensors: ");
                            SpiServo_DebugPrintDec(status_response[2]);
                            SpiServo_DebugPuts("\r\n");
                            
                            SpiServo_DebugPuts("\r\nReading distances for 5 seconds...\r\n");
                            if (status_response[2] >= 2) {
                                SpiServo_DebugPuts("Testing BOTH sensors:\r\n");
                            } else {
                                SpiServo_DebugPuts("Testing sensor 1 only:\r\n");
                            }
                            
                            for (int i = 0; i < 50; i++) {
                                uint8_t dist1_cmd[3] = {0x02, 0x00, 0x00};
                                uint8_t dist1_response[3];
                                
                                spi_tx_complete = false;
                                spi_rx_complete = false;
                                
                                rx_error = SM1_ReceiveBlock(handle->spi_device, dist1_response, 3);
                                if (rx_error == ERR_OK) {
                                    WAIT1_Waitms(1);
                                    tx_error = SM1_SendBlock(handle->spi_device, dist1_cmd, 3);
                                    if (tx_error == ERR_OK) {
                                        timeout = 50;
                                        while ((!spi_tx_complete || !spi_rx_complete) && timeout > 0) {
                                            WAIT1_Waitms(1);
                                            timeout--;
                                        }
                                        
                                        if (spi_tx_complete && spi_rx_complete) {
                                            uint16_t distance1 = (dist1_response[1] << 8) | dist1_response[2];
                                            SpiServo_DebugPuts("\rS1:");
                                            SpiServo_DebugPrintDec(distance1);
                                            SpiServo_DebugPuts("mm");

                                            if (status_response[2] >= 2) {
                                                uint8_t dist2_cmd[3] = {0x02, 0x01, 0x00};
                                                uint8_t dist2_response[3];
                                                
                                                spi_tx_complete = false;
                                                spi_rx_complete = false;
                                                
                                                WAIT1_Waitms(5);
                                                
                                                rx_error = SM1_ReceiveBlock(handle->spi_device, dist2_response, 3);
                                                if (rx_error == ERR_OK) {
                                                    WAIT1_Waitms(1);
                                                    tx_error = SM1_SendBlock(handle->spi_device, dist2_cmd, 3);
                                                    if (tx_error == ERR_OK) {
                                                        timeout = 50;
                                                        while ((!spi_tx_complete || !spi_rx_complete) && timeout > 0) {
                                                            WAIT1_Waitms(1);
                                                            timeout--;
                                                        }
                                                        
                                                        if (spi_tx_complete && spi_rx_complete) {
                                                            uint16_t distance2 = (dist2_response[1] << 8) | dist2_response[2];
                                                            SpiServo_DebugPuts(" S2:");
                                                            SpiServo_DebugPrintDec(distance2);
                                                            SpiServo_DebugPuts("mm");
                                                        }
                                                    }
                                                }
                                            }
                                            SpiServo_DebugPuts("    ");
                                        }
                                    }
                                }
                                WAIT1_Waitms(100);
                            }
                            SpiServo_DebugPuts("\r\n");
                        } else {
                            SpiServo_DebugPuts("ToF sensors are DISABLED or not responding !!\r\n");
                        }
                    } else {
                        SpiServo_DebugPuts("[ERROR] Status command timeout\r\n");
                    }
                }
            }
            SpiServo_DebugPuts("=== ToF Sensor Test Complete ===\r\n");
        }
        //time
        else if (buffer_pos == 4 && 
                 command_buffer[0] == 't' && command_buffer[1] == 'i' && 
                 command_buffer[2] == 'm' && command_buffer[3] == 'e') {
            SpiServo_DebugPuts("\r\n--- Reading Light Sensor for 5 seconds ---\r\n");
            SpiServo_DebugPuts("Raw ADC | Percent | Level    | Time\r\n");
            SpiServo_DebugPuts("--------|---------|----------|------\r\n");
            
            for (int i = 0; i < 50; i++) {
                uint16_t raw = PhotoRes_ReadRaw();
                uint8_t percentage = PhotoRes_GetLightPercentage();
                PhotoRes_LightLevel level = PhotoRes_GetLightLevel();
                const char* time_str = PhotoRes_GetTimeString();
                
                SpiServo_DebugPuts("\r");
                
                //raw ADC
                if (raw < 10000) SpiServo_DebugPuts(" ");
                if (raw < 1000) SpiServo_DebugPuts(" ");
                if (raw < 100) SpiServo_DebugPuts(" ");
                if (raw < 10) SpiServo_DebugPuts(" ");
                SpiServo_DebugPrintDec(raw);
                SpiServo_DebugPuts(" | ");
                
                //%
                if (percentage < 100) SpiServo_DebugPuts(" ");
                if (percentage < 10) SpiServo_DebugPuts(" ");
                SpiServo_DebugPrintDec(percentage);
                SpiServo_DebugPuts("%   | ");
                
                //light
                switch (level) {
                    case LIGHT_LEVEL_DARK:
                        SpiServo_DebugPuts("DARK    ");
                        break;
                    case LIGHT_LEVEL_DIM:
                        SpiServo_DebugPuts("DIM     ");
                        break;
                    case LIGHT_LEVEL_NORMAL:
                        SpiServo_DebugPuts("NORMAL  ");
                        break;
                    case LIGHT_LEVEL_BRIGHT:
                        SpiServo_DebugPuts("BRIGHT  ");
                        break;
                }
                SpiServo_DebugPuts(" | ");
                
                //time
                SpiServo_DebugPuts(time_str);
                
                WAIT1_Waitms(100);
            }
            SpiServo_DebugPuts("\r\n--- Light Sensor Reading Complete ---\r\n");
            
            const char* final_time = PhotoRes_GetTimeString();
            SpiServo_DebugPuts("Current time of day: ");
            SpiServo_DebugPuts(final_time);
            SpiServo_DebugPuts("\r\n");
        }
        //light
        else if (buffer_pos == 5 && 
                 command_buffer[0] == 'l' && command_buffer[1] == 'i' && 
                 command_buffer[2] == 'g' && command_buffer[3] == 'h' && 
                 command_buffer[4] == 't') {
            PhotoRes_PrintDebug();
        }
        //rgb
        else if (buffer_pos >= 3 && 
                 command_buffer[0] == 'r' && command_buffer[1] == 'g' && 
                 command_buffer[2] == 'b') {
            if (buffer_pos == 3) {
                RGBLed_PrintHelp();
            } else {
                //string parse
                char rgb_command[32];
                uint8_t rgb_start = 3;
                
                if (rgb_start < buffer_pos && command_buffer[rgb_start] == ' ') {
                    rgb_start++;
                }
                
                uint8_t rgb_pos = 0;
                for (uint8_t i = rgb_start; i < buffer_pos && rgb_pos < sizeof(rgb_command) - 1; i++) {
                    rgb_command[rgb_pos++] = command_buffer[i];
                }
                rgb_command[rgb_pos] = '\0';
                
                RGBLed_Status rgb_status = RGBLed_ProcessCommand(rgb_command);
                if (rgb_status != RGB_LED_OK) {
                    SpiServo_DebugPuts("RGB command failed\r\n");
                }
            }
        }
        else if (command_buffer[0] == '.' ||
                 (buffer_pos == 3 && 
                  command_buffer[0] == 'r' && command_buffer[1] == 'e' && 
                  command_buffer[2] == 'd') ||
                 (buffer_pos == 5 && 
                  command_buffer[0] == 'g' && command_buffer[1] == 'r' && 
                  command_buffer[2] == 'e' && command_buffer[3] == 'e' && 
                  command_buffer[4] == 'n') ||
                 (buffer_pos == 4 && 
                  command_buffer[0] == 'b' && command_buffer[1] == 'l' && 
                  command_buffer[2] == 'u' && command_buffer[3] == 'e') ||
                 (buffer_pos == 6 && 
                  command_buffer[0] == 'y' && command_buffer[1] == 'e' && 
                  command_buffer[2] == 'l' && command_buffer[3] == 'l' && 
                  command_buffer[4] == 'o' && command_buffer[5] == 'w') ||
                 (buffer_pos == 7 && 
                  command_buffer[0] == 'm' && command_buffer[1] == 'a' && 
                  command_buffer[2] == 'g' && command_buffer[3] == 'e' && 
                  command_buffer[4] == 'n' && command_buffer[5] == 't' && 
                  command_buffer[6] == 'a') ||
                 (buffer_pos == 5 && 
                  command_buffer[0] == 'w' && command_buffer[1] == 'h' && 
                  command_buffer[2] == 'i' && command_buffer[3] == 't' && 
                  command_buffer[4] == 'e') ||
                 (buffer_pos == 3 && 
                  command_buffer[0] == 'o' && command_buffer[1] == 'f' && 
                  command_buffer[2] == 'f')) {
            char color_command[16];
            strncpy(color_command, command_buffer, buffer_pos);
            color_command[buffer_pos] = '\0';
            
            RGBLed_Status rgb_status = RGBLed_ProcessCommand(color_command);
            if (rgb_status != RGB_LED_OK) {
                SpiServo_DebugPuts("RGB color command failed\r\n");
            }
        } else {
            SpiServo_DebugPuts("Unknown command. Type 'help' for commands.\r\n");
        }
        
        buffer_pos = 0;
        SpiServo_DebugPuts("SpiderBot> ");
        
        return SPI_SERVO_OK;
    }
    
    if (buffer_pos < sizeof(command_buffer) - 1) {
        command_buffer[buffer_pos++] = ch;
    } else {
        SpiServo_DebugPuts("\r\nCommand too long! Max 31 chars.\r\n");
        buffer_pos = 0;
        SpiServo_DebugPuts("SpiderBot> ");
    }
    
    return SPI_SERVO_OK;
}

SpiServo_Status SpiServo_WalkForwardStep(SpiServo_Handle *handle) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    SpiServo_DebugPuts("Walking forward step (ESP32-based)...\r\n");
    
    uint8_t command[3] = {0x01, 0x00, 0x00};
    uint8_t response[3];
    
    spi_tx_complete = false;
    spi_rx_complete = false;
    
    LDD_TError rx_error = SM1_ReceiveBlock(handle->spi_device, response, 3);
    if (rx_error != ERR_OK) {
        SpiServo_DebugPuts("[SPI] ERROR: ReceiveBlock setup failed\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    WAIT1_Waitms(1);
    
    LDD_TError tx_error = SM1_SendBlock(handle->spi_device, command, 3);
    if (tx_error != ERR_OK) {
        SpiServo_DebugPuts("[SPI] ERROR: SendBlock failed\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    uint16_t timeout = 100;
    while ((!spi_tx_complete || !spi_rx_complete) && timeout > 0) {
        WAIT1_Waitms(1);
        timeout--;
    }
    
    if (!spi_tx_complete || !spi_rx_complete) {
        SpiServo_DebugPuts("[SPI] ERROR: Transaction timeout\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    SpiServo_DebugPuts("Walk forward step command sent successfully\r\n");
    return SPI_SERVO_OK;
}

SpiServo_Status SpiServo_RotateC(SpiServo_Handle *handle) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    SpiServo_DebugPuts("Rotating clockwise (ESP32-based)...\r\n");
    
    uint8_t command[3] = {0x01, 0x01, 0x00};
    uint8_t response[3];
    
    spi_tx_complete = false;
    spi_rx_complete = false;
    
    LDD_TError rx_error = SM1_ReceiveBlock(handle->spi_device, response, 3);
    if (rx_error != ERR_OK) {
        SpiServo_DebugPuts("[SPI] ERROR: ReceiveBlock setup failed\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    WAIT1_Waitms(1);
    
    LDD_TError tx_error = SM1_SendBlock(handle->spi_device, command, 3);
    if (tx_error != ERR_OK) {
        SpiServo_DebugPuts("[SPI] ERROR: SendBlock failed\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    uint16_t timeout = 100;
    while ((!spi_tx_complete || !spi_rx_complete) && timeout > 0) {
        WAIT1_Waitms(1);
        timeout--;
    }
    
    if (!spi_tx_complete || !spi_rx_complete) {
        SpiServo_DebugPuts("[SPI] ERROR: Transaction timeout\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    SpiServo_DebugPuts("Rotate clockwise step command sent successfully\r\n");
    return SPI_SERVO_OK;
}

SpiServo_Status SpiServo_StopWalking(SpiServo_Handle *handle) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    SpiServo_DebugPuts("Stopping walking (ESP32-based)...\r\n");
    
    uint8_t command[3] = {0x01, 0x02, 0x00};
    uint8_t response[3];
    
    spi_tx_complete = false;
    spi_rx_complete = false;
    
    LDD_TError rx_error = SM1_ReceiveBlock(handle->spi_device, response, 3);
    if (rx_error != ERR_OK) {
        SpiServo_DebugPuts("[SPI] ERROR: ReceiveBlock setup failed\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    WAIT1_Waitms(1);
    
    LDD_TError tx_error = SM1_SendBlock(handle->spi_device, command, 3);
    if (tx_error != ERR_OK) {
        SpiServo_DebugPuts("[SPI] ERROR: SendBlock failed\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    uint16_t timeout = 100;
    while ((!spi_tx_complete || !spi_rx_complete) && timeout > 0) {
        WAIT1_Waitms(1);
        timeout--;
    }
    
    if (!spi_tx_complete || !spi_rx_complete) {
        SpiServo_DebugPuts("[SPI] ERROR: Transaction timeout\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    for (uint8_t leg = 0; leg < 6; leg++) {
        handle->current_angles[leg * 2] = 90;
    }
    SpiServo_DebugPuts("Stop walking command sent successfully\r\n");
    return SPI_SERVO_OK;
}

SpiServo_Status SpiServo_Rotate90C(SpiServo_Handle *handle) {
    if (!handle || !handle->initialized) {
        return SPI_SERVO_ERROR;
    }
    
    SpiServo_DebugPuts("Rotating 90deg clockwise (ESP32-based)...\r\n");
    
    uint8_t command[3] = {0x01, 0x04, 0x00};
    uint8_t response[3];
    
    spi_tx_complete = false;
    spi_rx_complete = false;
    
    LDD_TError rx_error = SM1_ReceiveBlock(handle->spi_device, response, 3);
    if (rx_error != ERR_OK) {
        SpiServo_DebugPuts("[SPI] ERROR: ReceiveBlock setup failed\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    WAIT1_Waitms(1);
    
    LDD_TError tx_error = SM1_SendBlock(handle->spi_device, command, 3);
    if (tx_error != ERR_OK) {
        SpiServo_DebugPuts("[SPI] ERROR: SendBlock failed\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    uint16_t timeout = 100;
    while ((!spi_tx_complete || !spi_rx_complete) && timeout > 0) {
        WAIT1_Waitms(1);
        timeout--;
    }
    
    if (!spi_tx_complete || !spi_rx_complete) {
        SpiServo_DebugPuts("[SPI] ERROR: Transaction timeout\r\n");
        return SPI_SERVO_SPI_ERROR;
    }
    
    SpiServo_DebugPuts("Rotate 90c command sent successfully. ESP32 is executing the sequence.\r\n");
    return SPI_SERVO_OK;
}
