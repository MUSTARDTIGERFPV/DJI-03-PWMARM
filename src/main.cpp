#include <Arduino.h>
#include <MSP.h>
#include "types.h"

// MSP Settings
#define MSP_BAUD 115200
// Pin definitions
#define MSP_TX 17
#define MSP_RX 16
#define PWM_INPUT 4

#define MSP_STATUS_EX            150   //out message		 For OSD ‘Fly mode', For OSD ‘Disarmed’

MSP msp;

msp_api_version_t api;
msp_ident_t identReply;
msp_fc_variant_t variant;

HardwareSerial SerialMSP(2);


uint32_t flightModeFlags = 0x00000002;
msp_status_DJI_t status_DJI = { 0 };

volatile long lastPWMRiseTime = 0;
volatile uint16_t pwmValue = 988;

void IRAM_ATTR onPwmPinChange() {
  long now = micros();
  bool status = digitalRead(PWM_INPUT);
  if (status) {
    lastPWMRiseTime = now;
  } else {
    long value = (now - lastPWMRiseTime);
    if (value >= 988 && value <= 2012) {
      pwmValue = value;
    }
  }
}

void setup() {
  //  Start USB Serial
  Serial.begin(115200);

  //  Start MSP Serial
  SerialMSP.begin(MSP_BAUD, SERIAL_8N1, MSP_RX, MSP_TX);

  // Fire interrupt when pin changes
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT), onPwmPinChange, CHANGE);

  //  Allocate stream and timeout (default timeout = 500)
  msp.begin(SerialMSP);

  //  Original MSP message response
  identReply.multiWiiVersion = 0;
  identReply.multiType = TYPE_QUADX;
  identReply.mspVersion = MSP_PROTOCOL_VERSION;
  identReply.capability = MSP_FEATURE_VBAT;

  pinMode(LED_BUILTIN, OUTPUT);

  // Betaflight message variant
  const char identifier[] = {'P', 'W', 'M', 'X'};
  strcpy(variant.flightControlIdentifier, identifier);
  Serial.println("Initialized");
}

void loop() {
  // poll rcInput
  Serial.print(pwmValue);

  if (pwmValue > 1500) {
    flightModeFlags = 0x00000003;    // armed 
    Serial.println(": Armed");
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on 
  } else {
    flightModeFlags = 0x00000002;    // disarmed
    Serial.println(": Disarmed");
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off 
  }

  status_DJI.flightModeFlags = flightModeFlags;
  status_DJI.armingFlags = 0x0303;
  msp.send(MSP_STATUS_EX, &status_DJI, sizeof(status_DJI));
  status_DJI.armingFlags = 0x0000;
  msp.send(MSP_STATUS, &status_DJI, sizeof(status_DJI));  

  delay(100);
}