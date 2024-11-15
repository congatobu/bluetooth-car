#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <Adafruit_BMP085.h>
#include <QMC5883LCompass.h>
#include <Wire.h>
#include "Directions.h"
//--------------------------------------------
#define ARDUINO_A0 A0
#define ARDUINO_A1 A1
#define ARDUINO_A2 A2
#define ARDUINO_A3 A3
#define ARDUINO_SDA A4
#define ARDUINO_SCL A5
//--------------------------------------------
#define L298N_ENA 10
#define L298N_IN1 6
#define L298N_IN2 7
#define L298N_IN3 8
#define L298N_IN4 9
#define L298N_ENB 5

#define GAS_SENSOR 12

#define ETHANOL_SENSOR 13


#define MAX_DISTANCE 30
#define WHEEL_SPEED 150
#define SERVO_INTERVL = 20

//--------------------------------------------
#define HY_SRF_TRIGGER A2
#define HY_SRF_ECHO A3
long distance;

//--------------------------------------------------------------------------
// CAU HINH GUI MESSAGE QUA BLUETOOTH
SoftwareSerial bluetooth(ARDUINO_A0, ARDUINO_A1);  //(RX, TX) of HC-05/ HC-06
unsigned long message_time = 0;
int message_period = 100;
unsigned message_sequance = 0;
//String bluetooth_receiver = "";

//--------------------------------------------------------------------------
// CAU HINH THOI GIAN DO KHOANG CACH BANG CAM BIEN SIEU AM
unsigned long ultrasonic_time = 0;
int ultrasonic_period = 200;

//--------------------------------------------------------------------------
// CAU HINH THOI GIAN DOC DU LIEU TU CAM BIEN NHIET DO
unsigned long bmp_180_time = 0;
int bmp_180_period = 1000;

//--------------------------------------------------------------------------
// CAU HINH THOI GIAN DOC TOA DO TU QMC5883L
unsigned long qmc_5883l_time = 0;
int qmc_5883l_period = 250;

//--------------------------------------------------------------------------
// CAU HINH THOI GIAN TU DONG QUAY CUA XE
unsigned long turn_time = 0;
int turn_period = 900;

//--------------------------------------------------------------------------
// CAU HINH THOI GIAN DOC CAM BIEN GAS
unsigned long gas_time = 0;
int gas_period = 1000;
int gas_value = 1;
int ethanol_value = 1;

//--------------------------------------------------------------------------
QMC5883LCompass qmc_5883l_sensor;
int16_t mx, my, mz;
float heading;

//---------------------------------------------------------------------------
Adafruit_BMP085 bmp_180_sensor;
float temperature;
float pressure;
float hight;


//---------------------------------------------------------------------------
// bien dieu khien
State state = STATE_STOP;
Driver driver = DRIVER_BLUETOOTH;
JsonDocument move_command;

void setup() {
  // begin:
  Wire.begin();
  Serial.begin(9600);

  bluetooth.begin(38400);


  // cai dat cho L298
  pinMode(L298N_ENA, OUTPUT);
  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);
  pinMode(L298N_IN3, OUTPUT);
  pinMode(L298N_IN4, OUTPUT);
  pinMode(L298N_ENB, OUTPUT);

  // cai dat cho ultra sonic hy srf05
  pinMode(HY_SRF_ECHO, INPUT);
  pinMode(HY_SRF_TRIGGER, OUTPUT);

  // ==================== QMC5883L ============================
  qmc_5883l_sensor.init();
  
  
  // ==================== BMP180 ============================
  bmp_180_sensor.begin();

  Serial.println("Setup Complete");

  state = STATE_MOVE_FORWARD;

  driver = DRIVER_BLUETOOTH;

  pinMode(ETHANOL_SENSOR, INPUT);
  pinMode(GAS_SENSOR, INPUT);
}

void loop() {


  bluetooth_controll();

  if (millis() >= message_time + message_period) {
    message_time += message_period;
    send_data_to_phone();
  }

  if (millis() >= ultrasonic_time + ultrasonic_period) {
    ultrasonic_time += ultrasonic_period;
    read_hy_srf05_sensor();
    
  }

  if (millis() >= bmp_180_time + bmp_180_period) {
    bmp_180_time += bmp_180_period;
    read_bmp180_sensor();
  }

  if (millis() >= qmc_5883l_time + qmc_5883l_period) {
    qmc_5883l_time += qmc_5883l_period;
    read_qmc_5883l_sensor();
  }

  //----------------------------------------------------------------

  if (millis() >= gas_time + gas_period) {
    gas_time += gas_period;
    read_gas_sensor();
  }

  if (driver == DRIVER_AUTO) {
    move_auto();
  } else {
    move_by_bluetooth(move_command);
  }
}



//------------------------------------------------------------------

void send_data_to_phone() {


  // long start_time = millis();

  JsonDocument doc;

  doc["d"] = distance;

  doc["a"] = heading;

  doc["t"] = temperature;

  doc["p"] = pressure;

  doc["h"] = hight;

  doc["g"] = gas_value;

  doc["e"] = ethanol_value;

  String out;

  serializeJson(doc, out);

  bluetooth.println(out);

  bluetooth.flush();
  
 // Serial.print("out ");
//  Serial.println(out);

  // long end_time = millis();
  // long run_time = end_time - start_time;
  //Serial.println(run_time);
}

//------------------------------------------------------------------

void bluetooth_controll() {
  if (bluetooth.available()) {
    // long start_time = millis();
    String bluetooth_receiver = bluetooth.readStringUntil('\n');
    JsonDocument document;
    DeserializationError error = deserializeJson(document, bluetooth_receiver);


    if (error) {
      Serial.println(bluetooth_receiver);
      return;
    }

    String command = document["c"];

    // Serial.print("nhan du lieu: ");
    // Serial.println(bluetooth_receiver);

    if (command == "M") {
      move_command = NULL;
      move_command = document;
    }

    if (command == "D") {
      String dri = document["m"];
      if (dri == "m") {
        driver = DRIVER_BLUETOOTH;
      }
      if (dri == "a") {
        driver = DRIVER_AUTO;
      }
    }
    //delay(50);
    
    // long end_time = millis();
    // long run_time = end_time - start_time;
    // Serial.println(run_time);
  }

  
}
//------------------------------------------------------------------

long read_hy_srf05_sensor() {

  long duration;

  digitalWrite(HY_SRF_TRIGGER, LOW);

  delayMicroseconds(2);

  digitalWrite(HY_SRF_TRIGGER, HIGH);

  delayMicroseconds(10);

  digitalWrite(HY_SRF_TRIGGER, LOW);

  duration = pulseIn(HY_SRF_ECHO, HIGH);

  distance = duration * 0.034 / 2;

  // Serial.println(distance);
  return distance;
}

void read_qmc_5883l_sensor() {

  qmc_5883l_sensor.read();
  heading = qmc_5883l_sensor.getAzimuth();
  // Serial.println();
  // Serial.print("heading ");
  // Serial.println(heading);
}

void read_bmp180_sensor() {

  temperature = bmp_180_sensor.readTemperature();

  pressure = bmp_180_sensor.readPressure();

  hight = bmp_180_sensor.readAltitude();

  // Serial.print("temperature ");
  // Serial.print(temperature);
  // Serial.print(" pressure ");
  // Serial.print(pressure);
  // Serial.print(" hight ");
  // Serial.print(hight);
}

void read_gas_sensor() {

  gas_value = digitalRead(GAS_SENSOR);
  ethanol_value = digitalRead(ETHANOL_SENSOR);

  // Serial.println();
  // Serial.print("gas value ");
  // Serial.print(gas_value);

  // Serial.print(" ethanol value ");
  // Serial.println(ethanol_value);
}

//------------------------------------------------------------------
// Move according to the instructions via Bluetooth.
void move_by_bluetooth(JsonDocument document) {

  if (document == NULL) {
    return;
  }


  // Serial.println("move to xx");
  float x = document["x"].as<float>();
  float y = document["y"].as<float>();


  // Serial.print(x);
  // Serial.println(y);

  if (x >= 0.5) {
    
    turn_left();
  }


  if (x <= -0.5) {
    turn_rigt();
    
  }

  if (x == 0 && y == 0) {
    left_stop();
    right_stop();
  }

  if (y >= 0.5) {
    forward();
  }

  if (y <= -0.5) {
    backward();
  }
}
//------------------------------------------------------------------
// di chuyen tu dong

void move_auto() {

  //  Serial.println(distance);
  if (state == STATE_MOVE_FORWARD && distance > 1 && distance < MAX_DISTANCE) {

    // Serial.print(distance);
    // Serial.println(" - doi trang thai sang quay trai, phai");
    state = STATE_TURN;
    turn_time = millis();
  }

  if (state != STATE_MOVE_FORWARD && millis() >= turn_time + turn_period) {
    state = STATE_MOVE_FORWARD;
  }

  if (state == STATE_TURN) {
    int random = rand() % 2;

    if (random == 0) {
      state = STATE_TURN_LEFT;
      // Serial.println("chay sang trai");
      
      turn_rigt();
    }

    if (random == 1) {
      state = STATE_TURN_RIGHT;
      // Serial.println("chay sang phai");
      turn_left();
    }
  }

  if (state == STATE_MOVE_FORWARD) {
    // Serial.println("chay thang");
    forward();
  }
}
//------------------------------------------------------------------

void forward() {
  left_forward();
  right_forward();
}

void backward() {
  left_backward();
  right_backwoard();
}

void turn_left() {
  right_forward();
  left_backward();
}

void turn_rigt() {

  left_forward();
  right_backwoard();

  
}

//------------------------------------------------------------------

void left_forward() {
  analogWrite(L298N_ENB, WHEEL_SPEED);
  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, HIGH);
}

void left_backward() {
  analogWrite(L298N_ENB, WHEEL_SPEED);
  digitalWrite(L298N_IN1, HIGH);
  digitalWrite(L298N_IN2, LOW);
}


void left_stop() {
  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, LOW);
}

//------------------------------------------------------------------

void right_forward() {
  analogWrite(L298N_ENA, WHEEL_SPEED);
  digitalWrite(L298N_IN3, HIGH);
  digitalWrite(L298N_IN4, LOW);
}

void right_backwoard() {
  analogWrite(L298N_ENA, WHEEL_SPEED);
  digitalWrite(L298N_IN3, LOW);
  digitalWrite(L298N_IN4, HIGH);
}

void right_stop() {
  digitalWrite(L298N_IN3, LOW);
  digitalWrite(L298N_IN4, LOW);
}

//------------------------------------------------------------------
