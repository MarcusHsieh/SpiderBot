# SpiderBot

## Overview

SpiderBot is a hexapod robot featuring a distributed dual-microcontroller architecture. The system combines the NXP K64F FRDM board for high-level control and sensor processing with an ESP32 for real-time servo management and walking algorithms.

## Key Features

- **12-DOF Locomotion**: 6 legs with 2 servos each (yaw + pitch joints)
- **Distributed Processing**: K64F handles sensors, ESP32 manages servo control
- **Sensor Suite**:
  - MPU6050 6-DOF IMU for orientation feedback
  - 2× VL53L0X Time-of-Flight distance sensors
  - Photoresistor for ambient light detection
- **High-Precision Control**: PCA9685 16-channel PWM driver with 12-bit resolution
- **SPI Communication**: 1MHz inter-processor communication protocol
- **Real-Time Feedback**: RGB LED status indicators and orientation-based behaviors

## Hardware Components

| Component | Purpose | Interface |
|-----------|---------|-----------|
| K64F FRDM Board | Main controller, sensor processing, user interface | I2C (MPU6050), ADC (photoresistor), GPIO (LEDs) |
| ESP32 | Servo controller, walking algorithms, distance sensing | SPI Slave, I2C Master |
| PCA9685 | 16-channel PWM servo driver | I2C (0x40) |
| MPU6050 | 6-DOF IMU (accelerometer + gyroscope) | I2C (0x68) |
| VL53L0X (×2) | Time-of-Flight distance sensors | I2C (0x30, 0x31) |
| 12× Servos | Leg actuation (yaw/pitch per leg) | PWM (50Hz) |

## Communication Protocol

### K64F ↔ ESP32 SPI Protocol
- **Format**: 3-byte packets at ~1MHz (SPI Mode 0)
- **Commands**:
  - Gait control (walk forward/backward/left/right, turn, stop)
  - Individual servo positioning (12 channels)
  - Distance sensor requests
- **Response**: Status + sensor data (3 bytes)

### K64F Sensor Processing
- **IMU**: Mahony filter for orientation estimation (100Hz update)
- **Photoresistor**: 16-bit ADC sampling for ambient light
- **RGB LEDs**: Visual feedback based on orientation and system state

## Project Structure

```
SpiderBot/
├── Sources/                      # K64F application source code
│   ├── main.c                    # Main application logic
│   ├── spi_servo_control.c/h     # SPI communication with ESP32
│   ├── mpu6050.c/h               # IMU driver and interface
│   ├── mahony.c/h                # Orientation filter algorithm
│   ├── photoresistor.c/h         # Light sensor driver
│   └── rgb_led_control.c/h       # LED control functions
├── ESP32_ServoControl/           # ESP32 PlatformIO project
│   ├── platformio.ini            # Build configuration
│   └── src/
│       └── ESP32_ServoControl.ino # Main servo control firmware
├── 3D Models/                    # Mechanical design files
│   ├── Print Models/             # STL files for 3D printing
│   └── Solidworks Models/        # Source CAD files
├── Generated_Code/               # Processor Expert auto-generated code
├── Static_Code/                  # K64F peripheral drivers (PDD)
├── SDK/                          # K64F CMSIS and device headers
├── Project_Settings/             # Linker scripts and debugger configs
├── ProcessorExpert.pe            # Processor Expert configuration
├── SpiderBot_Architecture_Documentation.md  # Detailed architecture
├── Implementation_Details_Section.md        # Implementation notes
├── System_Diagrams_Readable.md              # System diagrams
└── Testing_Evaluation_Section.md            # Testing results
```

## Documentation

- **[SpiderBot_Architecture_Documentation.md](SpiderBot_Architecture_Documentation.md)** - Hardware and software architecture, pin mappings, protocols

## Building the Project

### K64F Firmware (Kinetis Design Studio)

1. Open Kinetis Design Studio
2. Import project: `File → Import → Existing Projects → SpiderBot`
3. Select build configuration: `Debug` or `Release`
4. Build: `Project → Build Project`
5. Flash: Connect FRDM-K64F via USB, use debugger configuration in `Project_Settings/Debugger/`

### ESP32 Firmware (PlatformIO)

```bash
cd ESP32_ServoControl
pio run                    # Build firmware
pio run --target upload    # Upload to ESP32
pio device monitor         # View serial output (115200 baud)
```

**Dependencies** (auto-installed via platformio.ini):
- Adafruit PWM Servo Driver Library
- VL53L0X by Pololu
- Wire (I2C)
- SPI

## Hardware Setup

1. **Power Supply**:
   - 5-6V @ 10A for servos
   - 3.3V regulated for logic (K64F, ESP32, sensors)

2. **K64F Connections**:
   - SPI0: PTD0-PTD3 → ESP32 VSPI
   - I2C0: PTE24-PTE25 → MPU6050
   - ADC0: PTE20 → Photoresistor
   - GPIO: PTC0-PTC5 → RGB LEDs

3. **ESP32 Connections**:
   - VSPI: GPIO5,18,19,23 → K64F SPI0
   - I2C: GPIO21-22 → PCA9685, VL53L0X sensors
   - GPIO16-17 → VL53L0X XSHUT pins

4. **I2C Devices**:
   - MPU6050: 0x68 (on K64F I2C bus)
   - PCA9685: 0x40 (on ESP32 I2C bus)
   - VL53L0X #1: 0x30 (on ESP32 I2C bus)
   - VL53L0X #2: 0x31 (on ESP32 I2C bus)

## Usage

1. Power on the system
2. K64F initializes and performs sensor calibration
3. ESP32 initializes servo controller and distance sensors
4. System enters ready state (indicated by RGB LEDs)
5. Send gait commands via K64F SPI interface
6. Monitor orientation and light levels via K64F sensors
7. ESP32 executes walking algorithms and provides distance feedback

## Technologies Used

- **MCUs**: NXP K64F (ARM Cortex-M4), Espressif ESP32 (Dual-core Xtensa)
- **IDE**: Kinetis Design Studio (K64F), PlatformIO (ESP32)
- **Framework**: Processor Expert, Arduino (ESP32)
- **Communication**: SPI, I2C
- **Sensors**: I2C-based IMU and ToF sensors
- **Control**: PWM servo control via PCA9685
