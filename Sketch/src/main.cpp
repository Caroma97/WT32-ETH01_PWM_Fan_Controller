/*
  Analoge Signale haben noch den Bereich vom Arduino Nano

*/

#include <Arduino.h>
#include <Seeed_BME280.h>
#include <Wire.h>

const byte PIN_fan1_pwm   =                  15; //  D5;
const byte PIN_fan1_tacho =                  39; //  A7;
const byte PIN_fan2_pwm   =                  14; //  D6;
const byte PIN_fan2_tacho =                  36; //  A6;
const byte PIN_Poti       =                  35; //  A0;
const byte PIN_Button     =                  32; //  D2;
const byte PIN_SDA        =                   2; //  A4;
const byte PIN_SCL        =                   4; //  A5;


#define interrupt_trigger_type            RISING // interrupt  triggered on a RISING input
volatile  bool interrupt_process_status = false;  // start with no switch press pending,  ie false
bool initialisation_complete            = false;  // inhibit any interrupts until initialisation is complete
const byte debounce                     =    10;  // time to wait in milli secs
byte mode                               =     0;
const byte maxMode                      =     1;

BME280 bme280;

class FAN_CONTROLLER {
  public:
    FAN_CONTROLLER(byte pin_pwm, byte pin_tacho);
    void setSpeed(int newSpeed);
    int rpm();
    int speed = 0;
    //void toggleMode();
    //byte mode = 0;

  private:
    byte _pin_pwm;
    byte _pin_tacho;
    int _speed_calculated;
    byte _maxMode = 0;
};

FAN_CONTROLLER::FAN_CONTROLLER(byte pin_pwm, byte pin_tacho) {
  pinMode(pin_pwm, OUTPUT);
  pinMode(pin_tacho, INPUT);
  analogWrite(_pin_pwm, 0);
  _pin_pwm = pin_pwm;
  _pin_tacho = pin_tacho;
}

void FAN_CONTROLLER::setSpeed(int newSpeed) {
  if (speed != newSpeed) {
    if (newSpeed <= 0) newSpeed = 0;
    else if (newSpeed >= 100) newSpeed = 100;
    else if (newSpeed == speed + 1 || newSpeed == speed - 1 ) return;
    speed = newSpeed;
    _speed_calculated= map(newSpeed, 0, 100, 0, 255);

    Serial.print(F("Setze Lüfter auf "));
    Serial.print(speed);
    Serial.print(F("% / "));
    Serial.println(_speed_calculated);

    analogWrite(_pin_pwm, _speed_calculated);
  }
}

int FAN_CONTROLLER::rpm() {
  delay(100);
  return round(0.5* analogRead(_pin_tacho));    // x0,5 da ein normaler PC Lüfter 2 Impulse pro Umdrehung ausgibt
}
/*
void FAN_CONTROLLER::toggleMode() {
  mode++;
  if (mode > _maxMode) mode = 0;
  return;    // x0,5 da ein normaler PC Lüfter 2 Impulse pro Umdrehung ausgibt
}
*/
FAN_CONTROLLER fan1(PIN_fan1_pwm, PIN_fan1_tacho);


int Mode_Auto() {
  //Serial.println(F("Reached Mode_Auto()"));
  int value = map(bme280.getTemperature(), 25, 80, 0, 100);
  if (value <= 0) return 0;
  return value;
}


int Mode_Manu() {
  //Serial.println(F("Reached Mode_Manu()"));
  int value = analogRead(PIN_Poti);
  //Serial.print(F("Analog Value: "));
  //Serial.println(value);
  return map(value, 0, 1023, 0, 100);
}




void setup() {
  delay(200);
  Serial.begin(9600);
  Serial.println(F("Starting Fan PWM Controller V0.1"));
  pinMode(PIN_Poti, INPUT);
  initialisation_complete = true; // open interrupt processing for business
  if (!bme280.init()) Serial.println(F("Device error!"));
  
}


void loop() {
  int fanValue = 0;
  switch (mode) {
    case 0:
      fanValue = Mode_Manu();
      break;
    case 1:
      fanValue = Mode_Auto();
      break;
    default:
      Serial.print(F("Unknown Mode: "));
      Serial.println(mode);
  }
  fan1.setSpeed(fanValue);
}

