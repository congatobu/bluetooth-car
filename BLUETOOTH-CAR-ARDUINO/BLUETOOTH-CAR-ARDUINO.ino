#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <Adafruit_BMP085.h>
#include "QMC5883L.h"
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


#define MAX_DISTANCE 30
#define WHEEL_SPEED 150
#define SERVO_INTERVL = 20

//--------------------------------------------
#define HY_SRF_TRIGGER A0
#define HY_SRF_ECHO A1
long distance;
//--------------------------------------------------------------------------
// CAU HINH GUI MESSAGE QUA BLUETOOTH
SoftwareSerial bluetooth(3, 4);  //(RX, TX) of HC-05/ HC-06
unsigned long message_time = 0;
int message_period = 1000;
unsigned message_sequance = 0;
String bluetooth_receiver = "";
//--------------------------------------------------------------------------
// CAU HINH THOI GIAN DO KHOANG CACH BANG CAM BIEN SIEU AM
unsigned long ultrasonic_time = 0;
int ultrasonic_period = 100;

// CAU HINH THOI GIAN DOC DU LIEU TU CAM BIEN NHIET DO
unsigned long bmp_180_time = 0;
int bmp_180_period = 1000;

// CAU HINH THOI GIAN DOC TOA DO TU QMC5883L
unsigned long qmc_5883l_time = 0;
int qmc_5883l_period = 200;

// CAU HINH THOI GIAN TU DONG QUAY CUA XE
unsigned long turn_time = 0;
int turn_period = 900;

//--------------------------------------------------------------------------
QMC5883L qmc_5883l_sensor;
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
Driver driver = DRIVER_AUTO;
JsonDocument move_command;

void setup() {
  // begin:
  Wire.begin();
  Serial.begin(9600);
  bluetooth.begin(9600);


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
  qmc_5883l_sensor.setSamplingRate(50);

  // ==================== BMP180 ============================
  bmp_180_sensor.begin();

  Serial.println("Setup Complete");

  state = STATE_MOVE_FORWARD;
  driver = DRIVER_AUTO;
}

void loop() {

  bluetooth_controll();

  if (millis() >= ultrasonic_time + ultrasonic_period) {
    ultrasonic_time += ultrasonic_period;
    //read_hy_srf05_sensor();
  }


  if (millis() >= qmc_5883l_time + qmc_5883l_period) {
    qmc_5883l_time += qmc_5883l_period;
    // read_qmc_5883l_sensor();
  }

  if (millis() >= bmp_180_time + bmp_180_period) {
    bmp_180_time += bmp_180_period;
    // read_bmp180_sensor();
  }


  if (millis() >= message_time + message_period) {
    message_time += message_period;
    // send_data_to_phone();
  }

  //----------------------------------------------------------------



  if (driver == DRIVER_AUTO) {
    move_auto();
    //delay(500);
  } else {
    move_by_bluetooth(move_command);
  }
}



//------------------------------------------------------------------

void send_data_to_phone() {


  JsonDocument doc;

  doc["cmd"] = "DATA";

  message_sequance += 1;

  doc["seq"] = message_sequance;

  doc["dist"] = distance;


  doc["heading"] = heading;

  doc["temp"] = temperature;

  doc["press"] = pressure;

  doc["hight"] = hight;

  String out;

  serializeJson(doc, out);

  Serial.println(out);

  bluetooth.println(out);

  bluetooth.flush();
}

//------------------------------------------------------------------

void bluetooth_controll() {

  if (Serial.available()) {
    String command = Serial.readString();
    bluetooth.println(command);
    Serial.print("BLUETOOTH SEND >> " + command);
  }


  static String input_chars;

  while (bluetooth.available()) {

    char input_char = (char)bluetooth.read();

    if (input_char == '\n') {

      bluetooth_receiver = input_chars;

      input_chars = "";

    } else {
      input_chars += input_char;
    }
  }

  if (bluetooth_receiver == "") {
    return;
  }


  JsonDocument document;
  DeserializationError error = deserializeJson(document, bluetooth_receiver);

  if (error) {
    Serial.print(bluetooth_receiver);
    //   Serial.print("deserializeJson() failed: ");
    //   Serial.println(error.f_str());
    return;
  }

  String command = document["cmd"];
  
  Serial.print("nhan du lieu: ");Serial.println(bluetooth_receiver);

  if (command == "MOVE") {

    move_command = document;
  }

  if (command == "DRIVE") {
    String dri = document["mode"];
    if (dri == "manual") {
      driver = DRIVER_BLUETOOTH;
    }
    if (dri == "auto") {
      driver = DRIVER_AUTO;
    }
  }

  bluetooth_receiver = "";
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

  //Serial.println(distance);
  return distance;
}

void read_qmc_5883l_sensor() {

  heading = qmc_5883l_sensor.readHeading();

  // Serial.print("heading ");Serial.println(heading);
}

void read_bmp180_sensor() {

  temperature = bmp_180_sensor.readTemperature();

  pressure = bmp_180_sensor.readPressure();

  hight = bmp_180_sensor.readAltitude();

  // Serial.print("temperature ");Serial.println(temperature);
  // Serial.print("pressure ");Serial.println(pressure);
  // Serial.print("hight ");Serial.println(hight);
}



//------------------------------------------------------------------
// di chuyen theo chi dan bang bluetooth
void move_by_bluetooth(JsonDocument document) {

  // Serial.println("move to xx");
  float x = document["x"].as<float>();
  float y = document["y"].as<float>();


  // Serial.print(x);
  // Serial.println(y);

  if (x >= 0.5) {
    //Serial.println("chay sang trai");
    turn_left();
  }


  if (x <= -0.5) {
    // Serial.println("chay sang phai");
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
      turn_left();
    }

    if (random == 1) {
      state = STATE_TURN_RIGHT;
      // Serial.println("chay sang phai");
      turn_rigt();
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
  left_forward();
  right_backwoard();
}

void turn_rigt() {
  right_forward();
  left_backward();
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