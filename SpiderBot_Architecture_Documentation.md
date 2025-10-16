# Architecture Documentation

### System Components
- **K64F FRDM Board**: Main controller, sensor processing, user interface
- **ESP32**: Servo controller, walking algorithms, distance sensing  
- **PCA9685**: 16-channel PWM servo driver
- **12 Servo Motors**: 6 legs × 2 servos (yaw + pitch)
- **MPU6050**: 6-DOF IMU for orientation feedback
- **VL53L0X ToF Sensors**: 2× distance sensors for obstacle detection
- **RGB LEDs**: 2× status indication
- **Photoresistor**: Ambient light sensing

## Hardware Architecture

### Power System Design
- **Logic Power**: 3.3V regulated supply for microcontrollers and sensors
- **Servo Power**: 5.0-6.0V high-current supply (10A capacity)
- **Power Distribution**: Separate analog/digital ground planes
- **Protection**: Fused power rails with overcurrent protection

### K64F FRDM Board Interfaces

#### SPI Master (SPI0) - Primary Communication
| K64F Pin | Function | Direction | ESP32 Pin | Purpose |
|----------|----------|-----------|-----------|---------|
| PTD0 | SPI0_PCS0 | Output | GPIO5 | Chip Select |
| PTD1 | SPI0_SCK | Output | GPIO18 | Serial Clock (~1MHz) |
| PTD2 | SPI0_SOUT | Output | GPIO23 | Master Out Slave In |
| PTD3 | SPI0_SIN | Input | GPIO19 | Master In Slave Out |

**Configuration**: Mode 0, 8-bit frames, 3-byte protocol

#### I2C Master (I2C0) - Sensor Interface  
| K64F Pin | Function | Pull-up | Device |
|----------|----------|---------|--------|
| PTE25 | I2C0_SDA | 4.7kΩ | MPU6050 |
| PTE24 | I2C0_SCL | 4.7kΩ | MPU6050 |

**Configuration**: 100kHz standard mode, 7-bit addressing

#### ADC System (ADC0)
| Channel | Pin | Sensor | Range | Resolution |
|---------|-----|--------|-------|------------|
| 0 | PTE20 | Photoresistor | 0-3.3V | 16-bit |

**Configuration**: SAR ADC, single-ended, interrupt-driven

#### GPIO System
| Pin Group | Pins | Function | Configuration |
|-----------|------|----------|---------------|
| RGB LED 1 | PTC0-PTC2 | R,G,B | Output, Push-Pull |
| RGB LED 2 | PTC3-PTC5 | R,G,B | Output, Push-Pull |
| Status LED | PTB22 | System Status | Output, Active Low |

### ESP32 Interfaces

#### VSPI Slave - Command Reception
| ESP32 Pin | Function | Configuration |
|-----------|----------|---------------|
| GPIO5 | VSPI_SS | Input, Active Low |
| GPIO18 | VSPI_CLK | Input, Rising Edge |
| GPIO23 | VSPI_MOSI | Input |
| GPIO19 | VSPI_MISO | Output |

#### I2C Master - Device Control
| ESP32 Pin | Function | Connected Devices |
|-----------|----------|-------------------|
| GPIO21 | SDA | PCA9685, VL53L0X×2 |
| GPIO22 | SCL | PCA9685, VL53L0X×2 |

#### Sensor Control
| ESP32 Pin | Function | Purpose |
|-----------|----------|---------|
| GPIO16 | XSHUT_1 | VL53L0X Sensor 1 Enable |
| GPIO17 | XSHUT_2 | VL53L0X Sensor 2 Enable |

### Servo System Design

#### PCA9685 PWM Controller
- **Interface**: I2C slave at address 0x40
- **Resolution**: 12-bit PWM (4096 levels)  
- **Frequency**: 50Hz for servo compatibility
- **Channels**: 16 available, 12 used

#### Servo Mapping
| Channel | Leg | Joint | Position | Offset | Range |
|---------|-----|-------|----------|--------|-------|
| 0,1 | 0 | Yaw,Pitch | 45° | 0°,0° | 15°-158° |
| 2,3 | 1 | Yaw,Pitch | 0° | -5°,0° | 15°-158° |
| 4,5 | 2 | Yaw,Pitch | 315° | +2°,-10° | 15°-158° |
| 6,7 | 3 | Yaw,Pitch | 225° | -5°,-6° | 15°-158° |
| 8,9 | 4 | Yaw,Pitch | 180° | 0°,-20° | 15°-158° |
| 10,11 | 5 | Yaw,Pitch | 135° | -10°,+15° | 15°-158° |

#### PWM Signal Characteristics
- **Period**: 20ms (50Hz)
- **Pulse Width**: 1-2ms (5%-10% duty cycle)
- **Angle Mapping**: Linear 15°→1.0ms, 158°→2.0ms
- **PWM Values**: 102-512 (12-bit) 

## Software Architecture

### K64F Software Stack

#### Application Layer
- **main.c**: System coordinator and initialization
- **spi_servo_control.c**: SPI master and servo control
- **mpu6050.c/mahony.c**: IMU processing with AHRS filter
- **rgb_led_control.c**: Status indication system
- **photoresistor.c**: Light sensor management

#### Processor Expert Components
- **SM1**: SPI Master hardware abstraction
- **GI2C1**: I2C Generic for sensor communication
- **IO1**: UART debug interface
- **AD1**: ADC for analog sensing
- **GPIOC**: GPIO control for LEDs
- **WAIT1**: Timing delays and timeouts

#### Hardware Abstraction Layer
- **SPI0**: Master mode peripheral
- **I2C0**: Standard mode peripheral  
- **UART0**: 115200 baud communication
- **ADC0**: 16-bit SAR converter
- **GPIO**: Multiple port control

### ESP32 Software Stack

#### Application Layer
- **ESP32_ServoControl.ino**: Main application
- **Command Queue System**: Asynchronous processing
- **Servo Control Engine**: Position management
- **Walking Algorithm**: Tripod gait implementation
- **ToF Sensor Manager**: Distance measurement
- **Pose Controller**: Predefined positions

#### Arduino Framework
- **SPI Slave Handler**: Command reception
- **Adafruit PWMServoDriver**: PCA9685 interface
- **Wire Library**: I2C communication
- **VL53L0X Library**: ToF sensor control

## Communication Protocol

### SPI Command Protocol (3-byte packets)

#### Command Structure
| Byte 0 | Byte 1 | Byte 2 | Description |
|--------|--------|--------|-------------|
| Mode | Parameter 1 | Parameter 2 | Command Format |

#### Command Modes

**Mode 0x4E - Servo Control**
- Channels 0-11: Direct servo control (channel, angle)
- Channel 250: Emergency stop
- Channel 251: Stand position  
- Channel 252: Straight stand
- Channel 253: Lay position
- Channel 254: Stand max
- Channel 255: Park yaw servos

**Mode 0x01 - Walking Commands**
- 0x00: Walk forward step
- 0x01: Rotate clockwise  
- 0x02: Stop walking
- 0x04: Rotate 90° clockwise

**Mode 0x02 - Sensor Commands**
- 0x00: Get ToF sensor 1 distance
- 0x01: Get ToF sensor 2 distance
- 0x02: Get sensor status

#### Response Format
Standard response: `[0xE5, data_high, data_low]`
- Distance: millimeters as 16-bit value
- Status: enabled flag and sensor count
- Command echo with status code

### UART Command Interface

#### Key Commands
| Command | Function |
|---------|----------|
| `c#-angle` | Set servo channel to angle |
| `stand` | Normal stand position |
| `standmax` | Maximum extension stand |
| `lay` | Resting position |
| `stop` | Emergency stop |
| `park` | Disable yaw servos |
| `walkfwd` | Walk forward step |
| `rotatec` | Rotate clockwise |
| `readimu` | Stream IMU data |
| `tofsensors` | Stream ToF data |
| `rgb [args]` | RGB LED control |
| `help` | Show command list |

## Sensor Systems

### MPU6050 IMU Sensor
- **Interface**: I2C at address 0x68
- **Accelerometer**: ±2g range, 16384 LSB/g
- **Gyroscope**: ±250°/s range, 131 LSB/(°/s)
- **Sample Rate**: 125Hz configurable
- **Processing**: Mahony AHRS filter, 10Hz output

### VL53L0X ToF Distance Sensors
- **Interface**: I2C with dynamic addressing
- **Sensor 1**: Address 0x30, GPIO16 XSHUT
- **Sensor 2**: Address 0x31, GPIO17 XSHUT  
- **Range**: 30mm - 2000mm
- **Accuracy**: ±3% typical
- **Update Rate**: 10Hz

### Photoresistor Light Sensor
- **Interface**: ADC0 Channel 0 (PTE20)
- **Circuit**: Voltage divider with 10kΩ resistor
- **Processing**: 16-bit resolution, percentage conversion
- **Classification**: Night/Dawn/Day based on thresholds 

## Walking Algorithm Design

### Tripod Gait Implementation
Hexapod uses a tripod gait where legs are divided into two groups:
- **Group A**: Legs 0, 2, 4 (triangular stability)
- **Group B**: Legs 1, 3, 5 (triangular stability)

#### Walking Sequence
1. **Phase 1**: Lift Group A, advance Group B ground contact
2. **Phase 2**: Lower Group A, retract Group B  
3. **Phase 3**: Lift Group B, advance Group A ground contact
4. **Phase 4**: Lower Group B, retract Group A

#### Servo Movements per Step
- **Lift**: Pitch servo to 35° (leg up)
- **Advance**: Yaw servo ±15° from center (forward/back)
- **Lower**: Pitch servo to 45° (ground contact)
- **Retract**: Yaw servo return to center position

### Rotation Algorithm
4-step clockwise rotation sequence:
1. Lift legs 0,2,4 → rotate yaw to 105°
2. Push with legs 1,3,5 → yaw to 75° 
3. Lift legs 1,3,5 → rotate yaw to 105°
4. Push with legs 0,2,4 → yaw to 75°

## Advanced Features

### Servo Calibration System
Individual offset calibration stored in ESP32 PROGMEM:
```cpp
const int8_t servoOffsets[12] PROGMEM = {
   0,  0,  // CH0,1: Leg @ 45°   
   -5,  0,  // CH2,3: Leg @ 0°
   2, -10,  // CH4,5: Leg @ 315°
   -5,  -6,  // CH6,7: Leg @ 135°
   0, -20,  // CH8,9: Leg @ 180°
   -10, 15  // CH10,11: Leg @ 225°
};
```

### Command Queue System
ESP32 asynchronous processing:
- 10-command circular buffer
- ISR-safe queuing from SPI callback
- Main loop execution for complex operations
- Prevents blocking during servo movements

### Auto-Park Feature
Automatic yaw servo disabling:
- Triggered after pose commands (stand, standmax)
- Prevents oscillation while maintaining position
- Pitch servos remain active for gravity support

## Performance Specifications

### Timing Characteristics
- **SPI Transaction**: ~3ms per command
- **Servo Response**: 200-500ms position dependent
- **Walking Step**: 800ms configurable
- **IMU Rate**: 10Hz (100ms interval)
- **ToF Rate**: 10Hz (100ms interval)
- **Command Latency**: <10ms

### Power Consumption
- **K64F**: ~200mA @ 3.3V
- **ESP32**: ~150mA @ 3.3V active
- **Sensors**: ~50mA @ 3.3V total
- **Servos**: ~2A peak @ 5V all active
- **Total**: ~2.5A peak, ~1.5A typical

### Communication Performance
- **SPI Speed**: 1MHz clock
- **I2C Speed**: 100kHz standard
- **UART Speed**: 115200 baud
- **Throughput**: ~300 commands/second max 

## Debugging and Diagnostics

### Debug Interfaces
1. **K64F UART Debug**: Primary system diagnostics at 115200 baud
2. **ESP32 Serial Monitor**: Servo system and command processing logs
3. **SPI Transaction Logging**: Real-time command/response tracing
4. **Sensor Data Streaming**: Live sensor value monitoring

### Diagnostic Commands

#### System Health Check
```
SpiderBot> help              # Show all available commands
SpiderBot> spitest           # Test SPI communication integrity
SpiderBot> readimu           # Stream IMU data for 10 seconds
SpiderBot> tofsensors        # Stream ToF distance data
SpiderBot> light             # Check photoresistor readings
SpiderBot> rgb status        # Show current LED states
```

#### Servo System Testing
```
SpiderBot> c0-90             # Test individual servo movement
SpiderBot> stand             # Test coordinated pose
SpiderBot> walkfwd           # Test walking algorithm
SpiderBot> stop              # Emergency stop all servos
SpiderBot> park              # Test auto-park feature
```

#### Advanced Diagnostics
```
SpiderBot> time              # Light-based time detection
SpiderBot> rgb test          # RGB LED sequence test
SpiderBot> rotate90c         # Test rotation algorithm
SpiderBot> standmax          # Test maximum extension pose
```

### Development Environment Notes

#### Recommended Development Setup
1. **K64F Development**: 
   - Kinetis Design Studio (KDS) v3.2.0 or newer
   - Processor Expert enabled
   - GNU ARM toolchain

2. **ESP32 Development**:
   - PlatformIO with VSCode (preferred)
   - Arduino IDE 2.x with ESP32 board package (alternative)
   - VL53L0X library v1.3.0+
   - Adafruit PWM Servo Driver library v2.4.0+

