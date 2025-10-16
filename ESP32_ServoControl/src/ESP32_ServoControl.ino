#include "driver/spi_slave.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>

#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS   5

#define I2C_SDA 21
#define I2C_SCL 22
#define PCA9685_ADDR 0x40

#define TOF_XSHUT_1 17
#define TOF_XSHUT_2 16
#define TOF_ADDR_1 0x30
#define TOF_ADDR_2 0x31
#define TOF_COUNT 2

#define SERVO_FREQ 50
#define SERVO_MIN 102
#define SERVO_MAX 512
#define SERVO_CHANNELS 12

//servo offsets
const int8_t servoOffsets[12] PROGMEM = {
   0,  0,  //CH0,1: Leg @ 45deg   
   -5,  0,  //CH2,3: Leg @ 0deg
   2, -10,  //CH4,5: Leg @ 315deg 
  -5,  -6,  //CH6,7: Leg @ 135deg 
   0,-20,  //CH8,9: Leg @ 180deg 
 -10,15   //CH10,11: Leg @ 225deg 
};

//spi comms
#define CMD_BUFFER_SIZE 3
#define RESPONSE_BUFFER_SIZE 3
static uint32_t spi_command_count = 0;
static uint32_t spi_valid_command_count = 0;

static WORD_ALIGNED_ATTR uint8_t recvbuf[CMD_BUFFER_SIZE];
static WORD_ALIGNED_ATTR uint8_t sendbuf[RESPONSE_BUFFER_SIZE];
static spi_slave_transaction_t t;

//servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

//tof sensors
VL53L0X tof1;
VL53L0X tof2;

//servo control
uint8_t currentAngles[SERVO_CHANNELS];
bool servosEnabled[SERVO_CHANNELS];

//walking state
int walkStep = 0;
int rotateStep = 0;
bool walkingMode = false;

//tof sensor
bool tofSensorsEnabled = false;
uint16_t tofDistance1 = 0;
uint16_t tofDistance2 = 0;
unsigned long lastTofRead = 0;
#define TOF_READ_INTERVAL 100

//leg defs (6 legs, each w yaw and pitch servo)
struct Leg {
  uint8_t yawChannel;
  uint8_t pitchChannel;
  uint16_t position;
};

Leg legs[6] = {
  {0, 1, 45},
  {2, 3, 0},
  {4, 5, 315},
  {6, 7, 225},
  {8, 9, 180},
  {10, 11, 135}
};

#define COMMAND_QUEUE_SIZE 10

struct Command {
  uint8_t mode;
  uint8_t param1;
  uint8_t param2;
  bool valid;
};

static Command commandQueue[COMMAND_QUEUE_SIZE];
static volatile int queueHead = 0;
static volatile int queueTail = 0;
static volatile bool newCommand = false;

void process_command();
void my_post_trans_cb(spi_slave_transaction_t *trans);
void setServoAngle(uint8_t channel, uint8_t angle);
void disableServo(uint8_t channel);
void enableServo(uint8_t channel);
void executeStandPose();
void executeStraightStandPose();
void executeLayPose();
void executeStandMaxPose();
void executeStopCommand();
void executeParkCommand();
void executeWalkForwardStep();
void executeRotateClockwise();
void executeStopWalking();
void executeRotate90Clockwise();
void initializeServos();
void printCommand(uint8_t mode, uint8_t param1, uint8_t param2);
void execute_queued_command(const Command& cmd);
void queue_command(uint8_t mode, uint8_t param1, uint8_t param2);
bool get_next_command(Command& cmd);
bool initializeTofSensors();
void readTofSensors();
void getTofDistances(uint16_t* dist1, uint16_t* dist2);
void execute_command(uint8_t mode, uint8_t param1, uint8_t param2);

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== ESP32 SpiderBot ===");
    
  //init i2c
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("[I2C] Initializing on pins SDA=" + String(I2C_SDA) + ", SCL=" + String(I2C_SCL));
  
  //init pca9685
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  Serial.println("[PCA9685] Initialized at " + String(SERVO_FREQ) + "Hz");
  
  //init servo states
  initializeServos();
  
  //init tof sensors
  Serial.println("[TOF] Attempting initial sensor initialization...");
  tofSensorsEnabled = initializeTofSensors();
  
  //retry
  if (!tofSensorsEnabled) {
    Serial.println("[TOF] Initial init failed, retrying after delay...");
    delay(1000);
    tofSensorsEnabled = initializeTofSensors();
  }
  
  Serial.printf("[SPI_DIAG] ToF sensors final status: %s\n", tofSensorsEnabled ? "ENABLED" : "DISABLED");
  
  //spi slave config
  spi_bus_config_t buscfg = {
    .mosi_io_num = GPIO_MOSI,
    .miso_io_num = GPIO_MISO,
    .sclk_io_num = GPIO_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
  };

  spi_slave_interface_config_t slvcfg = {
    .spics_io_num = GPIO_CS,
    .flags = 0,
    .queue_size = 1,
    .mode = 0,
    .post_setup_cb = NULL,
    .post_trans_cb = my_post_trans_cb
  };

  esp_err_t ret = spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
  assert(ret == ESP_OK);
  
  if (ret == ESP_OK) {
    Serial.println("[SPI_DIAG] + SPI slave initialization SUCCESS");
  } else {
    Serial.printf("[SPI_DIAG] - SPI slave initialization FAILED: %d\n", ret);
  }

  sendbuf[0] = 0xE5;
  sendbuf[1] = 0x32;
  sendbuf[2] = 0x00;
  
  Serial.printf("[SPI_DIAG] Initial response buffer: [0x%02X, 0x%02X, 0x%02X]\n", sendbuf[0], sendbuf[1], sendbuf[2]);
  
  //q first transaction
  memset(&t, 0, sizeof(t));
  t.length = CMD_BUFFER_SIZE * 8;
  t.tx_buffer = sendbuf;
  t.rx_buffer = recvbuf;
  
  ret = spi_slave_queue_trans(VSPI_HOST, &t, portMAX_DELAY);
  assert(ret == ESP_OK);
  
  if (ret == ESP_OK) {
    Serial.println("[SPI_DIAG] + First SPI transaction queued successfully");
  } else {
    Serial.printf("[SPI_DIAG] - Failed to queue SPI transaction: %d\n", ret);
  }
}

void loop() {
  static unsigned long lastHeartbeat = 0;
  unsigned long now = millis();
  
  //process q'ed cmds
  Command cmd;
  while (get_next_command(cmd)) {
    execute_queued_command(cmd);
  }
  
  //read tof sensors every once in a while
  // if (tofSensorsEnabled && (now - lastTofRead > TOF_READ_INTERVAL)) {
  //   readTofSensors();
  //   lastTofRead = now;
  // }
  
  //heartbeat
  if (now - lastHeartbeat > 5000) {
    Serial.printf("[HEARTBEAT] ESP32 running, uptime: %lu ms\n", now);
    lastHeartbeat = now;
  }
  
  delay(100);
}

void my_post_trans_cb(spi_slave_transaction_t *trans) {
  process_command();
  
  //q next transaction
  memset(&t, 0, sizeof(t));
  t.length = CMD_BUFFER_SIZE * 8;
  t.tx_buffer = sendbuf;
  t.rx_buffer = recvbuf;
  spi_slave_queue_trans(VSPI_HOST, &t, portMAX_DELAY);
}

void process_command() {
  uint8_t mode = recvbuf[0];
  uint8_t param1 = recvbuf[1];
  uint8_t param2 = recvbuf[2];
  
  //inc cmd counter
  spi_command_count++;
  
  //q cmd to exec
  queue_command(mode, param1, param2);

  sendbuf[0] = 0xE5;
  sendbuf[1] = mode;
  sendbuf[2] = 0x00;
  
  if (mode == 0x4E) {
    //servo control mode || special cmds
    if (param1 <= 11) {
      //normal servo cmd: channel 0-11
      sendbuf[1] = param1;
      sendbuf[2] = param2;
    }
    else {
      //special cmds
      switch (param1) {
        case 250: sendbuf[2] = 0x01; break;  //STOP
        case 251: sendbuf[2] = 0x02; break;  //STAND
        case 252: sendbuf[2] = 0x03; break;  //STRAIGHT
        case 253: sendbuf[2] = 0x04; break;  //LAY
        case 254: sendbuf[2] = 0x05; break;  //STANDMAX
        case 255: sendbuf[2] = 0x06; break;  //PARK
        default:
          sendbuf[2] = 0xFF; //unknown
          break;
      }
    }
  }
  else if (mode == 0x01) {
    //walking ctrl mode
    switch (param1) {
      case 0x00: sendbuf[2] = 0x10; break;  //WALK
      case 0x01: sendbuf[2] = 0x11; break;  //ROTATE
      case 0x02: sendbuf[2] = 0x12; break;  //STOP WALK
      case 0x04: sendbuf[2] = 0x13; break;  //ROT90
      default:
        sendbuf[2] = 0xFF; //unknown
        break;
    }
  }
  else if (mode == 0x02) {
    //sensor data mode
    switch (param1) {
      case 0x00: //sensor 1 dist
        if (tofSensorsEnabled) {
          sendbuf[1] = (tofDistance1 >> 8) & 0xFF;
          sendbuf[2] = tofDistance1 & 0xFF;
        } else {
          sendbuf[1] = 0xFF;
          sendbuf[2] = 0xFF;
        }
        break;
      case 0x01: //sensor 2 dist
        if (tofSensorsEnabled && TOF_COUNT > 1) {
          sendbuf[1] = (tofDistance2 >> 8) & 0xFF;
          sendbuf[2] = tofDistance2 & 0xFF;
        } else {
          sendbuf[1] = 0xFF;
          sendbuf[2] = 0xFF;
        }
        break;
      case 0x02: //sensor status
        //if disabled, try reinit
        if (!tofSensorsEnabled) {
          queue_command(0xFE, 0x01, 0x00);
        }
        
        sendbuf[1] = tofSensorsEnabled ? 0x01 : 0x00;
        sendbuf[2] = tofSensorsEnabled ? TOF_COUNT : 0x00;
        break;
      default:
        sendbuf[1] = 0xFF;
        sendbuf[2] = 0xFF;
        break;
    }
  }
  else {
    //unknown
    sendbuf[1] = 0xFF;
    sendbuf[2] = 0xFF;
  }
}

void printCommand(uint8_t mode, uint8_t param1, uint8_t param2) {
  Serial.printf("[CMD] Received: [0x%02X, 0x%02X, 0x%02X] -> ", mode, param1, param2);
  
  if (mode == 0x4E) {
    if (param1 <= 11) {
      Serial.printf("Set CH%d = %ddeg\n", param1, param2);
    } else {
      const char* cmdName = "UNKNOWN";
      switch (param1) {
        case 250: cmdName = "STOP"; break;
        case 251: cmdName = "STAND"; break;
        case 252: cmdName = "STRAIGHT STAND"; break;
        case 253: cmdName = "LAY"; break;
        case 254: cmdName = "STAND MAX"; break;
        case 255: cmdName = "PARK YAW"; break;
      }
      Serial.printf("Special: %s\n", cmdName);
    }
  }
  else if (mode == 0x01) {
    const char* walkCmd = "UNKNOWN";
    switch (param1) {
      case 0x00: walkCmd = "WALK FORWARD"; break;
      case 0x01: walkCmd = "ROTATE CLOCKWISE"; break;
      case 0x02: walkCmd = "STOP WALKING"; break;
      case 0x04: walkCmd = "ROTATE 90deg CW"; break;
    }
    Serial.printf("Walk: %s\n", walkCmd);
  }
  else if (mode == 0x02) {
    const char* sensorCmd = "UNKNOWN";
    switch (param1) {
      case 0x00: sensorCmd = "GET TOF1 DISTANCE"; break;
      case 0x01: sensorCmd = "GET TOF2 DISTANCE"; break;
      case 0x02: sensorCmd = "GET SENSOR STATUS"; break;
    }
    Serial.printf("Sensor: %s\n", sensorCmd);
  }
  else {
    Serial.printf("Unknown mode\n");
  }
}

void setServoAngle(uint8_t channel, uint8_t angle) {
  if (channel >= SERVO_CHANNELS) {
    Serial.printf("[SERVO] ERROR: Invalid channel %d\n", channel);
    return;
  }
  
  //servo offset (!!)
  int8_t offset = pgm_read_byte(&servoOffsets[channel]);
  int16_t adjustedAngle = angle + offset;
  
  //constraints
  if (adjustedAngle < 15) adjustedAngle = 15;
  if (adjustedAngle > 158) adjustedAngle = 158;
  
  //angle -> pwm value
  uint16_t pwmValue = map(adjustedAngle, 15, 158, SERVO_MIN, SERVO_MAX);
  
  try {
    pwm.setPWM(channel, 0, pwmValue);
    currentAngles[channel] = angle;//keep old angle
    servosEnabled[channel] = true;
    Serial.printf("[SERVO] CH%d set to %ddeg (offset %+d, actual %ddeg, PWM=%d)\n", 
                  channel, angle, offset, adjustedAngle, pwmValue);
  } catch (...) {
    Serial.printf("[SERVO] ERROR: Failed to set CH%d to %ddeg\n", channel, angle);
  }
}

void disableServo(uint8_t channel) {
  if (channel >= SERVO_CHANNELS) return;
  
  pwm.setPWM(channel, 0, 0);
  servosEnabled[channel] = false;
  
  Serial.printf("[SERVO] CH%d disabled\n", channel);
}

void enableServo(uint8_t channel) {
  if (channel >= SERVO_CHANNELS) return;
  
  setServoAngle(channel, currentAngles[channel]);
}

void initializeServos() {
  Serial.println("[SERVO] Initializing all servos to center position (90deg)");
  
  for (int i = 0; i < SERVO_CHANNELS; i++) {
    currentAngles[i] = 90;
    servosEnabled[i] = true;
    setServoAngle(i, 90);
  }
  
  Serial.println("[SERVO] All servos initialized");
}

void executeStandPose() {
  Serial.println("[POSE] Executing stand position: yaw=90deg, pitch=45deg");
  
  for (int leg = 0; leg < 6; leg++) {
    setServoAngle(legs[leg].yawChannel, 90);
    setServoAngle(legs[leg].pitchChannel, 45);
  }
}

void executeStraightStandPose() {
  Serial.println("[POSE] Executing straight stand position: legs parallel");
  
  uint8_t yawAngles[6] = {135, 90, 45, 45, 90, 135};
  
  for (int leg = 0; leg < 6; leg++) {
    setServoAngle(legs[leg].yawChannel, yawAngles[leg]);
    setServoAngle(legs[leg].pitchChannel, 45);
  }
  
  Serial.println("[AUTO] Auto-parking YAW servos after straight stand pose");
  delay(500);
  executeParkCommand();
}

void executeLayPose() {
  Serial.println("[POSE] Executing lay position: yaw=90deg, pitch=74deg");
  
  for (int leg = 0; leg < 6; leg++) {
    setServoAngle(legs[leg].yawChannel, 90);
    setServoAngle(legs[leg].pitchChannel, 74);
  }
}

void executeStandMaxPose() {
  Serial.println("[POSE] Executing stand max position: yaw=90deg, pitch=35deg");
  
  for (int leg = 0; leg < 6; leg++) {
    setServoAngle(legs[leg].yawChannel, 90);
    setServoAngle(legs[leg].pitchChannel, 35);
  }
  
  Serial.println("[AUTO] Auto-parking YAW servos after stand max pose");
  delay(500);
  executeParkCommand();
}

void executeStopCommand() {
  Serial.println("[EMERGENCY] Stopping all servos");
  
  for (int i = 0; i < SERVO_CHANNELS; i++) {
    disableServo(i);
  }
  
  walkingMode = false;
}

void executeParkCommand() {
  Serial.println("[PARK] Disabling YAW servos (even channels), keeping PITCH active");
  
  for (int leg = 0; leg < 6; leg++) {
    disableServo(legs[leg].yawChannel);
  }
}

void executeWalkForwardStep() {
  Serial.println("[WALK] Executing forward step (tripod gait)");
  
  walkingMode = true;
  
  if (walkStep % 2 == 0) {
    setServoAngle(legs[0].pitchChannel, 65);
    setServoAngle(legs[2].pitchChannel, 65);
    setServoAngle(legs[4].pitchChannel, 65);
    delay(200);
    
    setServoAngle(legs[0].yawChannel, 105);
    setServoAngle(legs[2].yawChannel, 105);
    setServoAngle(legs[4].yawChannel, 105);
    delay(200);
    
    setServoAngle(legs[0].pitchChannel, 30);
    setServoAngle(legs[2].pitchChannel, 30);
    setServoAngle(legs[4].pitchChannel, 30);
    delay(200);
    
    setServoAngle(legs[1].yawChannel, 75);
    setServoAngle(legs[3].yawChannel, 75);
    setServoAngle(legs[5].yawChannel, 75);
  } else {
    //lift group b, move group a
    setServoAngle(legs[1].pitchChannel, 65);
    setServoAngle(legs[3].pitchChannel, 65);
    setServoAngle(legs[5].pitchChannel, 65);
    delay(200);
    
    setServoAngle(legs[1].yawChannel, 105);
    setServoAngle(legs[3].yawChannel, 105);
    setServoAngle(legs[5].yawChannel, 105);
    delay(200);
    
    setServoAngle(legs[1].pitchChannel, 30);
    setServoAngle(legs[3].pitchChannel, 30);
    setServoAngle(legs[5].pitchChannel, 30);
    delay(200);
    
    setServoAngle(legs[0].yawChannel, 75);
    setServoAngle(legs[2].yawChannel, 75);
    setServoAngle(legs[4].yawChannel, 75);
  }
  
  walkStep++;
}

void executeRotateClockwise() {
  Serial.println("[ROTATE] Executing clockwise rotation step");
  
  switch (rotateStep % 4) {
    case 0:
      setServoAngle(legs[0].pitchChannel, 65);
      setServoAngle(legs[2].pitchChannel, 65);
      setServoAngle(legs[4].pitchChannel, 65);
      delay(200);
      
      setServoAngle(legs[0].yawChannel, 105);
      setServoAngle(legs[2].yawChannel, 105);
      setServoAngle(legs[4].yawChannel, 105);
      delay(200);
      
      setServoAngle(legs[0].pitchChannel, 30);
      setServoAngle(legs[2].pitchChannel, 30);
      setServoAngle(legs[4].pitchChannel, 30);
      break;
      
    case 1:
      setServoAngle(legs[1].yawChannel, 75);
      setServoAngle(legs[3].yawChannel, 75);
      setServoAngle(legs[5].yawChannel, 75);
      break;
      
    case 2:
      setServoAngle(legs[1].pitchChannel, 65);
      setServoAngle(legs[3].pitchChannel, 65);
      setServoAngle(legs[5].pitchChannel, 65);
      delay(200);
      
      setServoAngle(legs[1].yawChannel, 105);
      setServoAngle(legs[3].yawChannel, 105);
      setServoAngle(legs[5].yawChannel, 105);
      delay(200);
      
      setServoAngle(legs[1].pitchChannel, 30);
      setServoAngle(legs[3].pitchChannel, 30);
      setServoAngle(legs[5].pitchChannel, 30);
      break;
      
    case 3:
      setServoAngle(legs[0].yawChannel, 75);
      setServoAngle(legs[2].yawChannel, 75);
      setServoAngle(legs[4].yawChannel, 75);
      break;
  }
  
  rotateStep++;
}

void executeStopWalking() {
  Serial.println("[WALK] Stopping and returning to center position");
  
  walkingMode = false;
  walkStep = 0;
  rotateStep = 0;
  
  for (int leg = 0; leg < 6; leg++) {
    setServoAngle(legs[leg].yawChannel, 90);
    delay(100);
  }
}

void executeRotate90Clockwise() {
  Serial.println("[ROTATE] Executing 90deg clockwise rotation (5-step sequence)");
  
  for (int i = 0; i < 5; i++) {
    executeRotateClockwise();
    delay(500);
  }
}

void queue_command(uint8_t mode, uint8_t param1, uint8_t param2) {
  int nextHead = (queueHead + 1) % COMMAND_QUEUE_SIZE;
  if (nextHead != queueTail) {
    commandQueue[queueHead].mode = mode;
    commandQueue[queueHead].param1 = param1;
    commandQueue[queueHead].param2 = param2;
    commandQueue[queueHead].valid = true;
    queueHead = nextHead;
    newCommand = true;
  }
}

bool get_next_command(Command& cmd) {
  if (queueTail == queueHead) {
    return false;
  }
  
  cmd = commandQueue[queueTail];
  queueTail = (queueTail + 1) % COMMAND_QUEUE_SIZE;
  return cmd.valid;
}

void execute_queued_command(const Command& cmd) {
  uint8_t mode = cmd.mode;
  uint8_t param1 = cmd.param1;
  uint8_t param2 = cmd.param2;
  
  printCommand(mode, param1, param2);
  Serial.printf("[QUEUE] Executing: [0x%02X, 0x%02X, 0x%02X]\n", mode, param1, param2);
  
  if (mode == 0x4E) {
    if (param1 <= 11) {
      setServoAngle(param1, param2);
    }
    else {
      switch (param1) {
        case 250: executeStopCommand(); break;
        case 251: executeStandPose(); break;
        case 252: executeStraightStandPose(); break;
        case 253: executeLayPose(); break;
        case 254: executeStandMaxPose(); break;
        case 255: executeParkCommand(); break;
        default:
          Serial.printf("[ERROR] Unknown special command: %d\n", param1);
          break;
      }
    }
  }
  else if (mode == 0x01) {
    switch (param1) {
      case 0x00: executeWalkForwardStep(); break;
      case 0x01: executeRotateClockwise(); break;
      case 0x02: executeStopWalking(); break;
      case 0x04: executeRotate90Clockwise(); break;
      default:
        Serial.printf("[ERROR] Unknown walk command: %d\n", param1);
        break;
    }
  }
  else if (mode == 0xFE) {
    if (param1 == 0x01) {
      Serial.println("[SENSOR] Re-initializing ToF sensors due to status check...");
      bool newStatus = initializeTofSensors();
      if (newStatus && !tofSensorsEnabled) {
        tofSensorsEnabled = true;
        Serial.println("[SENSOR] + Re-initialization successful!");
      } else if (!newStatus) {
        Serial.println("[SENSOR] - Re-initialization failed");
      }
    }
  }
  else if (mode == 0x02) {
    Serial.printf("[SENSOR] ToF sensor command processed (param1=0x%02X)\n", param1);
    Serial.printf("[SENSOR] Status response: enabled=%s, count=%d\n", 
                 tofSensorsEnabled ? "true" : "false", tofSensorsEnabled ? TOF_COUNT : 0);
  }
  else {
    Serial.printf("[ERROR] Unknown mode: 0x%02X\n", mode);
  }
}

bool initializeTofSensors() {
  Serial.println("[TOF] ========================================");
  Serial.println("[TOF] Initializing VL53L0X ToF sensors...");
  if (TOF_COUNT > 1) {
    Serial.printf(", Sensor2=%d", TOF_XSHUT_2);
  }
  Serial.println();
  
  pinMode(TOF_XSHUT_1, OUTPUT);
  if (TOF_COUNT > 1) {
    pinMode(TOF_XSHUT_2, OUTPUT);
  }
 
  Serial.println("[TOF] Resetting all sensors...");
  digitalWrite(TOF_XSHUT_1, LOW);
  if (TOF_COUNT > 1) {
    digitalWrite(TOF_XSHUT_2, LOW);
  }
  delay(100);
  
  //init sensor 1
  Serial.println("[TOF] ----------------------------------------");
  Serial.println("[TOF] Initializing sensor 1...");
  digitalWrite(TOF_XSHUT_1, HIGH);
  delay(200);
  
  tof1.setTimeout(1000);
  
  bool sensor1_ok = false;
  for (int retry = 0; retry < 3; retry++) {
    Serial.printf("[TOF] Sensor 1 init attempt %d/3...\n", retry + 1);
    
    if (tof1.init()) {
      //custom @ (!!)
      tof1.setAddress(TOF_ADDR_1);
      delay(10);

      uint16_t testReading = tof1.readRangeSingleMillimeters();
      if (!tof1.timeoutOccurred()) {
        sensor1_ok = true;
        Serial.printf("[TOF] + Sensor 1 initialized successfully at address 0x%02X\n", TOF_ADDR_1);
        Serial.printf("[TOF] + Test reading: %dmm\n", testReading);
        break;
      } else {
        Serial.println("[TOF] - Sensor 1 init OK but test reading failed");
      }
    } else {
      Serial.printf("[TOF] - Sensor 1 init attempt %d failed\n", retry + 1);
    }
    
    if (retry < 2) {
      delay(100);
    }
  }
  
  if (!sensor1_ok) {
    Serial.println("[TOF] - ERROR: Failed to initialize sensor 1 after 3 attempts!");
    return false;
  }
  
  //init sensor 2
  bool sensor2_ok = true;
  if (TOF_COUNT > 1) {
    Serial.println("[TOF] ----------------------------------------");
    Serial.println("[TOF] Initializing sensor 2...");
    digitalWrite(TOF_XSHUT_2, HIGH);
    delay(50);
    
    tof2.setTimeout(1000);
    sensor2_ok = false;
    
    for (int retry = 0; retry < 3; retry++) {
      Serial.printf("[TOF] Sensor 2 init attempt %d/3...\n", retry + 1);
      
      if (tof2.init()) {
        tof2.setAddress(TOF_ADDR_2); //custom @ (!!)
        delay(10);
        
        uint16_t testReading = tof2.readRangeSingleMillimeters();
        if (!tof2.timeoutOccurred()) {
          sensor2_ok = true;
          Serial.printf("[TOF] + Sensor 2 initialized successfully at address 0x%02X\n", TOF_ADDR_2);
          Serial.printf("[TOF] + Test reading: %dmm\n", testReading);
          break;
        } else {
          Serial.println("[TOF] - Sensor 2 init OK but test reading failed");
        }
      } else {
        Serial.printf("[TOF] - Sensor 2 init attempt %d failed\n", retry + 1);
      }
      
      if (retry < 2) {
        delay(100);
      }
    }
    
    if (!sensor2_ok) {
      Serial.println("[TOF] - ERROR: Failed to initialize sensor 2 after 3 attempts!");
    }
  }
  
  bool overall_success = sensor1_ok && sensor2_ok;
  
  Serial.println("[TOF] ========================================");
  if (overall_success) {
    Serial.printf("[TOF] + SUCCESS: Initialized %d ToF sensor(s)\n", TOF_COUNT);
    Serial.println("[TOF] Sensors ready for distance measurements");
  } else {
    Serial.println("[TOF] - PARTIAL/FAILED: Some sensors failed to initialize");
    if (sensor1_ok) {
      Serial.println("[TOF] Sensor 1 is working");
    }
    if (TOF_COUNT > 1 && !sensor2_ok) {
      Serial.println("[TOF] Sensor 2 failed - continuing with sensor 1 only");
    }
  }
  Serial.println("[TOF] ========================================");
  
  return overall_success;
}

void readTofSensors() {
  if (!tofSensorsEnabled) {
    return;
  }
  
  //read sensor 1
  tofDistance1 = tof1.readRangeSingleMillimeters();
  if (tof1.timeoutOccurred()) {
    tofDistance1 = 9999;
    Serial.println("[TOF] WARNING: Sensor 1 timeout");
  } else if (tofDistance1 > 2000) {
    tofDistance1 = 9999;
  }
  
  //read sensor 2
  if (TOF_COUNT > 1) {
    tofDistance2 = tof2.readRangeSingleMillimeters();
    if (tof2.timeoutOccurred()) {
      tofDistance2 = 9999;
      Serial.println("[TOF] WARNING: Sensor 2 timeout");
    } else if (tofDistance2 > 2000) {
      tofDistance2 = 9999;
    }
  } else {
    tofDistance2 = 9999;
  }
  
  static int debugCounter = 0;
  if (++debugCounter >= 10) {
    if (TOF_COUNT > 1) {
      Serial.printf("[TOF] Distance: Sensor1=%dmm, Sensor2=%dmm\n", tofDistance1, tofDistance2);
    } else {
      Serial.printf("[TOF] Distance: Sensor1=%dmm\n", tofDistance1);
    }
    debugCounter = 0;
  }
}

void getTofDistances(uint16_t* dist1, uint16_t* dist2) {
  if (dist1) *dist1 = tofDistance1;
  if (dist2) *dist2 = tofDistance2;
}

void execute_command(uint8_t mode, uint8_t param1, uint8_t param2) {
  Serial.printf("[CMD] Received: [0x%02X, 0x%02X, 0x%02X] -> ", mode, param1, param2);
  
  if (mode == 0x4E) {
    if (param1 <= 11) {
      Serial.printf("Servo CH%d=%ddeg\n", param1, param2);
      setServoAngle(param1, param2);
    }
    else {
      switch (param1) {
        case 250:
          Serial.println("STOP command");
          for (int i = 0; i < 12; i++) {
            pwm.setPWM(i, 0, 0);
          }
          break;
        case 251:
          Serial.println("STAND command");
          executeStandPose();
          delay(100);
          executeParkCommand();
          break;
        case 252:
          Serial.println("STRAIGHT STAND command");
          executeStraightStandPose();
          delay(100);
          executeParkCommand();
          break;
        case 253:
          Serial.println("LAY command");
          executeLayPose();
          break;
        case 254:
          Serial.println("STAND MAX command");
          executeStandMaxPose();
          delay(100);
          executeParkCommand();
          break;
        case 255:
          Serial.println("PARK command");
          executeParkCommand();
          break;
        default:
          Serial.printf("Unknown special command %d\n", param1);
          break;
      }
    }
  }
  else if (mode == 0x01) {
    switch (param1) {
      case 0x00:
        Serial.println("Walk forward");
        executeWalkForwardStep();
        break;
      case 0x01:
        Serial.println("Rotate clockwise");
        executeRotateClockwise();
        break;
      case 0x02:
        Serial.println("Stop walking");
        executeStopWalking();
        break;
      case 0x04:
        Serial.println("Rotate 90deg clockwise");
        executeRotate90Clockwise();
        break;
      default:
        Serial.printf("Unknown walk command %d\n", param1);
        break;
    }
  }
  else if (mode == 0xFE) {
    if (param1 == 0x01) {
      Serial.println("Re-initializing ToF sensors");
      initializeTofSensors();
    }
  }
  else {
    Serial.printf("Unknown mode: 0x%02X\n", mode);
  }
} 