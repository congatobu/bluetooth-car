#include "WString.h"
#include "HardwareSerial.h"

class Logger {
public:
  void Log(String TAG, String message) {
    Serial.print(TAG);

    Serial.print(" : ");

    Serial.println(message);
  }
};

void Debug(String TAG, ...) {
  Serial.print(TAG);
  Serial.print(" : ");
  va_list argments;
  va_start(argments, TAG);
}