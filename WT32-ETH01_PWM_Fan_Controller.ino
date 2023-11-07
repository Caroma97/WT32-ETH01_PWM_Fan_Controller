/*
BUGS:
  - 1. Tastendruck wird nicht erkannt.(Variabeln fehler??) -> Mit Serial.print Funktionen verfolgen (Später Interrup vermeide!!)

*/

#include <Arduino.h>
#include <Seeed_BME280.h>
#include <Wire.h>
/*
const byte PIN_fan1_pwm   =                  15; //  D5;
const byte PIN_fan1_tacho =                  39; //  A7;
const byte PIN_fan2_pwm   =                  14; //  D6;
const byte PIN_fan2_tacho =                  36; //  A6;
const byte PIN_Poti       =                  35; //  A0;
const byte PIN_Button     =                  32; //  D2;
const byte PIN_SDA        =                   2; //  A4;
const byte PIN_SCL        =                   4; //  A5;
*/

const byte PIN_fan1_pwm   =                  5;
const byte PIN_fan1_tacho =                  A7;
const byte PIN_fan2_pwm   =                  6;
const byte PIN_fan2_tacho =                  A6;
const byte PIN_Poti       =                  A0;
const byte PIN_Button     =                  3;
const byte PIN_SDA        =                  A4;
const byte PIN_SCL        =                  A5;

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


/*
class INTERRUPT_HANDLER {
  public:
    INTERRUPT_HANDLER(byte pin);
    bool read_button();

  private:
    void button_interrupt_handler();
    volatile  bool _interrupt_process_status;
    bool _initialisation_complete;
    bool _switching_pending;
    unsigned long _elapse_timer;
    byte _pin;
};

INTERRUPT_HANDLER::INTERRUPT_HANDLER(byte pin) {
  _pin = pin;
  pinMode(_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(_pin), button_interrupt_handler, interrupt_trigger_type);
  _initialisation_complete = true; // open interrupt processing for business
}

void  INTERRUPT_HANDLER::button_interrupt_handler() {
  if (_initialisation_complete == true) {  //  Trigger vor Inizialisierung verhindern
    if (!_interrupt_process_status) {
      if (digitalRead(_pin) == HIGH) {
        _interrupt_process_status  = true; 
      }
    }
  }
}

bool INTERRUPT_HANDLER::read_button() {
  int button_reading;
  if (_interrupt_process_status == true) {
    button_reading = digitalRead(_pin);
    if (button_reading == HIGH)  {
      // switch is pressed, so start/restart wait for button relealse, plus  end of debounce process
      _switching_pending = true;
      _elapse_timer  = millis(); // start elapse timing for debounce checking
    }
    if (_switching_pending  && button_reading == LOW) {
      // switch was pressed, now released, so check  if debounce time elapsed
      if (millis() - _elapse_timer >= debounce) {
        // dounce time elapsed, so switch press cycle complete
        _switching_pending  = false;             // reset for next button press interrupt cycle
        _interrupt_process_status  = false;      // reopen ISR for business now button on/off/debounce cycle complete
        return true;                            // advise that switch has been pressed
      }
    }
  }
  return false; // either no press request or debounce  period not elapsed
} // end of read_button function

INTERRUPT_HANDLER button(PIN_Button);
*/

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

/*

int Mode_Auto() {
  //Serial.println(F("Reached Mode_Auto()"));
  for(int i = columns; i--; i < 0) {
    if (fan1.temperatur > config[0][i]) {
      fan1.temperatur -= 1.32;
      Serial.println(F("Temperatur liegt über Grenzwert!"));
      Serial.print(config[0][i]);
      Serial.print(F(" : "));
      Serial.println(fan1.temperatur);
      return config[1][i];
    }
  }
  return 0;
}

void setFan(int percentage) {
  Serial.println(F("Reached setFan()"));
  if (fan1.speed != percentage) {
    fan1.speed = percentage;
    int temp = map(percentage, 0, 100, 0, 255);
    Serial.print(F("Setze Lüfter auf "));
    Serial.print(percentage);
    Serial.print(F("% / "));
    Serial.println(temp);
    analogWrite(PIN_fan1_pwm, temp);
  }
}
*/

//
// ISR for  handling interrupt triggers arising from associated button switch
//
void  button_interrupt_handler() {
  if (initialisation_complete == true) {  //  Trigger vor Inizialisierung verhindern
    if (!interrupt_process_status) {
      if (digitalRead(PIN_Button) == HIGH) {
        interrupt_process_status  = true; 
      }
    }
  }
}

bool read_button() {
  int button_reading;
  // static variables because we need to retain old values  between function calls
  static bool     switching_pending = false;
  static long int elapse_timer;
  if (interrupt_process_status == true) {
    button_reading = digitalRead(PIN_Button);
    if (button_reading == HIGH)  {
      // switch is pressed, so start/restart wait for button relealse, plus  end of debounce process
      switching_pending = true;
      elapse_timer  = millis(); // start elapse timing for debounce checking
    }
    if (switching_pending  && button_reading == LOW) {
      // switch was pressed, now released, so check  if debounce time elapsed
      if (millis() - elapse_timer >= debounce) {
        // dounce time elapsed, so switch press cycle complete
        switching_pending  = false;             // reset for next button press interrupt cycle
        interrupt_process_status  = false;      // reopen ISR for business now button on/off/debounce cycle complete
        return true;                            // advise that switch has been pressed
      }
    }
  }
  return false; // either no press request or debounce  period not elapsed
} // end of read_button function


void SensorWerteBeispiel() {
    float pressure;

    //get and print temperatures
    Serial.print(F("Temp: "));
    Serial.print(bme280.getTemperature());
    Serial.println(F("C"));//The unit for  Celsius because original arduino don't support special symbols

    //get and print atmospheric pressure data
    Serial.print(F("Pressure: "));
    Serial.print(pressure = bme280.getPressure());
    Serial.println(F("Pa"));

    //get and print altitude data
    Serial.print(F("Altitude: "));
    Serial.print(bme280.calcAltitude(pressure));
    Serial.println(F("m"));

    //get and print humidity data
    Serial.print(F("Humidity: "));
    Serial.print(bme280.getHumidity());
    Serial.println(F("%"));

    delay(1000);
}




void setup() {
  delay(200);
  Serial.begin(9600);
  Serial.println(F("Starting Fan PWM Controller V0.1"));
  pinMode(PIN_Poti, INPUT);
  pinMode(PIN_Button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_Button), button_interrupt_handler, interrupt_trigger_type);
  initialisation_complete = true; // open interrupt processing for business
  if (!bme280.init()) Serial.println(F("Device error!"));
  
}


void loop() {
  //if (button.read_button()) {
  if (read_button()) {
    Serial.println(F("Button pressed!!"));
    mode++;
    if (mode > maxMode) mode = 0;
    Serial.print(F("Switch to Mode "));
    Serial.println(mode);
  } 

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

