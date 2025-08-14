#include<Arduino.h>

// L298N control using only INA + INB (PWM on direction pin)

int INA = 14; // Motor direction/speed A
int INB = 13; // Motor direction/speed B

void setup() {
  Serial.begin(115200);
  
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);

  // Setup PWM channels
  ledcAttachPin(INA, 0);    // Channel 0 for INA
  ledcAttachPin(INB, 1);    // Channel 1 for INB
  ledcSetup(0, 1000, 8);    // 1 kHz, 8-bit PWM for INA
  ledcSetup(1, 1000, 8);    // 1 kHz, 8-bit PWM for INB
}

void loop() {
  // Forward - half speed
  Serial.println("Forward - Half Speed");
  ledcWrite(0, 0); // INA = 50% duty
  ledcWrite(1, 50);   // INB = 0
  delay(500);

    // Stop
  Serial.println("Stop");
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  delay(2000);

  // Reverse - full speed
  Serial.println("Reverse - Full Speed");
  ledcWrite(0, 50);
  ledcWrite(1, 0); // INB = 100% duty
  delay(500);

  // Stop
  Serial.println("Stop");
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  delay(2000);
}
