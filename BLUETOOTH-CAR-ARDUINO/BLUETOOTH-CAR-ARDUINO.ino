#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
#include <QMC5883LCompass.h>
#include <MPU6050.h>

//========================= PIN DEFINITION ========================
#define ARDUINO_A0 A0
#define ARDUINO_A1 A1
#define ARDUINO_A2 A2
#define ARDUINO_A3 A3
#define ARDUINO_SDA A4
#define ARDUINO_SCL A5

#define L298N_ENA 10
#define L298N_IN1 6
#define L298N_IN2 7
#define L298N_IN3 8
#define L298N_IN4 9
#define L298N_ENB 5

#define SIG_SERVO 12

#define GAS_SENSOR 11
#define ETHANOL_SENSOR 4

#define HY_SRF_ECHO A0
#define HY_SRF_TRIGGER A1
//========================= BLUETOOTH ===========================
#define START_BYTE_1 0xBB
#define START_BYTE_2 0xCC
#define END_BYTE 0xFF

#define DEBUG_BT true  // Bật/Tắt debug qua Serial

#define RING_BUFFER_SIZE 32

uint8_t ringBuffer[RING_BUFFER_SIZE];
uint16_t head = 0;
uint16_t tail = 0;


//========================= CONST VALUES ===========================
const int MAX_DISTANCE = 30;


//========================= GLOBAL VARIABLES =======================
SoftwareSerial bluetooth(ARDUINO_A3, ARDUINO_A2);  // RX, TX
QMC5883LCompass qmc5883lSensor;
Adafruit_BMP280 bmp280Sensor;
MPU6050 mpu6050;


float temperature, pressure, hight;
float heading;
int gas_value = 1;
int ethanol_value = 1;
long distance;



float ax_g, ay_g, az_g;
float gx_dps, gy_dps, gz_dps;

struct Timer {
  unsigned long last_time;
  int interval;
};

Timer messageTimer = { 0, 100 };
Timer distanceTimer = { 0, 150 };
Timer environmentalTimer = { 0, 2000 };
Timer compassTimer = { 0, 150 };
Timer airQualityTimer = { 0, 2000 };
Timer mpuTimer = { 0, 120 };


String move_command;



//========================= SETUP ===========================
void setup() {

  Serial.begin(9600);

  bluetooth.begin(115200);

  pinMode(L298N_ENA, OUTPUT);
  pinMode(L298N_ENB, OUTPUT);
  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);
  pinMode(L298N_IN3, OUTPUT);
  pinMode(L298N_IN4, OUTPUT);

  pinMode(HY_SRF_ECHO, INPUT);
  pinMode(HY_SRF_TRIGGER, OUTPUT);

  pinMode(ETHANOL_SENSOR, INPUT);
  pinMode(GAS_SENSOR, INPUT);

  qmc5883lSensor.init();

  bmp280Sensor.begin(0x76);
  bmp280Sensor.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_500);

  //==============================================================
  mpu6050.initialize();



  if (!mpu6050.testConnection()) {
    Serial.println("MPU6050 connection failed!");
  } else {
    Serial.println("MPU6050 connected.");
  }

  while (bluetooth.available()) {
    bluetooth.read();
  }

  Serial.println("Setup Complete");
  Serial.println("---------------------------------------------------------------");
}

//========================= LOOP ===========================
void loop() {


  readBluetoothToRingBuffer();  // Đọc dữ liệu từ Bluetooth vào ring buffer

  parseBluetoothRingBuffer();  // Phân tích và xử lý gói tin

  if (millis() - messageTimer.last_time >= messageTimer.interval) {
    messageTimer.last_time = millis();
    sendCarSensorPacket();
  }

  if (millis() - mpuTimer.last_time >= mpuTimer.interval) {
    mpuTimer.last_time = millis();
    readMpuSensor();
  }

  if (millis() - distanceTimer.last_time >= distanceTimer.interval) {
    distanceTimer.last_time = millis();
    readDistanceSensor();
  }

  if (millis() - environmentalTimer.last_time >= environmentalTimer.interval) {
    environmentalTimer.last_time = millis();
    readEnvironmentalSensor();
  }

  if (millis() - compassTimer.last_time >= compassTimer.interval) {
    compassTimer.last_time = millis();
    readCompassHeading();
  }

  if (millis() - airQualityTimer.last_time >= airQualityTimer.interval) {
    airQualityTimer.last_time = millis();
    readAirQualitySensors();
  }
}

//========================= SENSOR FUNCTIONS ===========================

void readMpuSensor() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  // Đọc dữ liệu raw
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Scale factor theo datasheet:
  // Gia tốc: ±2g → 1g = 16384
  // Gyro: ±250 deg/s → 1 deg/s = 131
  ax_g = ax / 16384.0;
  ay_g = ay / 16384.0;
  az_g = az / 16384.0;

  gx_dps = gx / 131.0;
  gy_dps = gy / 131.0;
  gz_dps = gz / 131.0;
}

void readEnvironmentalSensor() {
  temperature = bmp280Sensor.readTemperature();
  pressure = bmp280Sensor.readPressure();
  hight = bmp280Sensor.readAltitude(1013.25);
  //Serial.print("temperature: "); Serial.println(temperature);
  //Serial.print("pressure: "); Serial.println(pressure);
  //Serial.print("hight: "); Serial.println(hight);
}

void readCompassHeading() {
  qmc5883lSensor.read();
  heading = fmod(qmc5883lSensor.getAzimuth() + 360.0, 360.0);
  //Serial.print("heading: "); Serial.println(heading);
}

void readAirQualitySensors() {

  int gasDetected = digitalRead(GAS_SENSOR);
  int ethanolDetected = digitalRead(ETHANOL_SENSOR);

  if (gasDetected == LOW) {
    gas_value = 1;
  } else {
    gas_value = 0;
  }

  if (ethanolDetected == LOW) {
    ethanol_value = 1;
  } else {
    ethanol_value = 0;
  }

  Serial.print("gas_value: ");
  Serial.println(gas_value);
  Serial.print("ethanol_value: ");
  Serial.println(ethanol_value);
}

void readDistanceSensor() {
  digitalWrite(HY_SRF_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(HY_SRF_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(HY_SRF_TRIGGER, LOW);
  distance = pulseIn(HY_SRF_ECHO, HIGH) / 58.0;
}


//========================= BLUETOOTH ===========================


// Add a byte to the ring buffer
void ringWrite(uint8_t data) {
  uint16_t next = (head + 1) % RING_BUFFER_SIZE;
  if (next != tail) {  // prevent overflow
    ringBuffer[head] = data;
    head = next;
  }
}

// Read a byte from the ring buffer
bool ringRead(uint8_t* data) {
  if (head == tail) return false;
  *data = ringBuffer[tail];
  tail = (tail + 1) % RING_BUFFER_SIZE;
  return true;
}

// Đọc từ bluetooth và nạp vào buffer
void readBluetoothToRingBuffer() {
  while (bluetooth.available()) {
    ringWrite(bluetooth.read());
  }
}


void parseBluetoothRingBuffer() {
  static enum {
    WAIT_START1,
    WAIT_START2,
    WAIT_ID,
    WAIT_LEN,
    READ_DATA,
    READ_CHECKSUM,
    WAIT_END
  } state = WAIT_START1;

  static uint8_t id = 0;
  static uint8_t length = 0;
  static uint8_t index = 0;
  static uint8_t checksum = 0;
  static uint8_t buffer[RING_BUFFER_SIZE];

  uint8_t b;
  while (ringRead(&b)) {
    switch (state) {
      case WAIT_START1:
        if (b == START_BYTE_1) state = WAIT_START2;
        break;

      case WAIT_START2:
        if (b == START_BYTE_2) state = WAIT_ID;
        else state = WAIT_START1;
        break;

      case WAIT_ID:
        id = b;
        state = WAIT_LEN;
        break;

      case WAIT_LEN:
        length = b;
        if (length == 0 || length > RING_BUFFER_SIZE) {
          if (DEBUG_BT) Serial.println("Invalid length");
          state = WAIT_START1;
        } else {
          index = 0;
          checksum = 0;
          state = READ_DATA;
        }
        break;

      case READ_DATA:
        if (index < RING_BUFFER_SIZE) {
          buffer[index++] = b;
          checksum ^= b;
        } else {
          if (DEBUG_BT) Serial.println("Buffer overflow!");
          state = WAIT_START1;
        }
        if (index >= length) state = READ_CHECKSUM;
        break;

      case READ_CHECKSUM:
        if (b == checksum) {
          state = WAIT_END;
        } else {
          if (DEBUG_BT) Serial.println("Checksum error");
          state = (b == START_BYTE_1) ? WAIT_START2 : WAIT_START1;
        }
        break;

      case WAIT_END:
        if (b == END_BYTE) {
          handleParsedPacket(id, buffer, length);  // xử lý theo ID
        } else {
          if (DEBUG_BT) Serial.println("Missing end byte");
        }
        state = WAIT_START1;
        break;
    }
  }
}


void handleParsedPacket(uint8_t id, byte* data, byte length) {

  switch (id) {
    case 0x02:  // Gói điều khiển xe
      if (length >= 3) {
        byte mode = data[0];
        char direction = (char)data[1];
        byte speed = data[2];

        // if (DEBUG_BT) {
        //   Serial.print("Mode: "); Serial.println(mode);
        //   Serial.print("Dir: "); Serial.println(direction);
        //   Serial.print("Speed: "); Serial.println(speed);
        // }

        switch (direction) {
          case 'f': moveForward(speed); break;
          case 'b': moveBackward(speed); break;
          case 'l': turnLeft(speed); break;
          case 'r': turnRight(speed); break;
          case 'p': stopMotors(); break;
          default: stopMotors(); break;
        }
      }
      break;

    case 0x01:

      break;

    default:
      if (DEBUG_BT) {
        Serial.print("Unknown packet ID: ");
        Serial.println(id, HEX);
      }
      break;
  }
}




//---------------------------------------------------------------------------------
void sendCarSensorPacket() {
  int16_t data[13] = {
    (int16_t)(temperature * 100),
    (int16_t)(pressure / 10),  // Giảm độ phân giải để vừa int16
    (int16_t)(hight * 100),
    (int16_t)(heading * 100),
    (int16_t)(distance * 100),
    (int16_t)(gas_value * 100),
    (int16_t)(ethanol_value * 100),
    (int16_t)(gx_dps * 100),
    (int16_t)(gy_dps * 100),
    (int16_t)(gz_dps * 100),
    (int16_t)(ax_g * 100),
    (int16_t)(ay_g * 100),
    (int16_t)(az_g * 100)
  };

  byte* ptr = (byte*)data;
  uint16_t payloadSize = sizeof(int16_t) * 13;

  byte checksum = 0;
  for (int i = 0; i < payloadSize; i++) {
    checksum ^= ptr[i];
  }

  bluetooth.write(0xAA);
  bluetooth.write(0x55);
  bluetooth.write((byte)(payloadSize >> 8));
  bluetooth.write((byte)payloadSize);
  bluetooth.write(ptr, payloadSize);
  bluetooth.write(checksum);
  bluetooth.write(0xFF);
}


//========================= MOTOR FUNCTIONS ===========================
void moveMotor(int in1, int in2, int in3, int in4, int ena, int enb) {
  digitalWrite(L298N_IN1, in1);
  digitalWrite(L298N_IN2, in2);
  digitalWrite(L298N_IN3, in3);
  digitalWrite(L298N_IN4, in4);
  analogWrite(L298N_ENA, ena);
  analogWrite(L298N_ENB, enb);
}

void moveForward(int speed) {
  moveMotor(LOW, HIGH, HIGH, LOW, speed, speed);
}

void moveBackward(int speed) {
  moveMotor(HIGH, LOW, LOW, HIGH, speed, speed);
}

void stopMotors() {
  moveMotor(LOW, LOW, LOW, LOW, 0, 0);
}

void turnLeft(int speed) {
  moveMotor(HIGH, LOW, HIGH, LOW, speed, speed);
}

void turnRight(int speed) {
  moveMotor(LOW, HIGH, LOW, HIGH, speed, speed);
}