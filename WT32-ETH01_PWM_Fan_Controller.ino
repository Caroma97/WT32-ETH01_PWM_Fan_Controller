#include <Arduino.h>
#include <Seeed_BME280.h>                       // Libary einfügen
#include <Wire.h>


const byte PIN_fan1_pwm = 5;                    // Pins am Nano im Code zugeordnet
const byte PIN_fan1_tacho = A7;
const byte PIN_fan2_pwm = 6;
const byte PIN_fan2_tacho = A6;
const byte PIN_Poti = A0;
const byte PIN_Button = 3;
const byte PIN_SDA = A4;
const byte PIN_SCL = A5;


#define interrupt_trigger_type RISING            // interrupt bei steigender Flanke
volatile bool interrupt_process_status = false;
bool initialisation_complete = true;             // keine Interrupts solange die Initialisierung nicht abgeschlossen ist
const byte debounce = 10;                        // Warten in milli secs, wegen Prellen des Scahlters
byte mode = 0;
const byte maxMode = 1;

BME280 bme280;                                   // Temperatur Sensor initialisieren; I2C wir nehmen die Strandard Pins 

class FAN_CONTROLLER {                           // Klasse Fan Controller als Vorgabe, erleichtert das einfügen weiterer Lüfter um einiges
public:
  FAN_CONTROLLER(byte pin_pwm, byte pin_tacho);  // Public Funktionen und Variablen
  void setSpeed(int newSpeed);
  int speed = 0;

private:                                         // Private Variablen innerhalb der class
  byte _pin_pwm;
  byte _pin_tacho;
  int _speed_calculated;
  byte _maxMode = 0;
};  

FAN_CONTROLLER::FAN_CONTROLLER(byte pin_pwm, byte pin_tacho) {  // :: Funktion wird der Klasse zugeordnet
  pinMode(pin_pwm, OUTPUT);
  pinMode(pin_tacho, INPUT);
  analogWrite(pin_pwm, 0);
  _pin_pwm = pin_pwm;
  _pin_tacho = pin_tacho;                                       // Klasse Fan_Controller Ende
}

void FAN_CONTROLLER::setSpeed(int newSpeed) {
  if (speed != newSpeed) {                             
    if (newSpeed <= 0) newSpeed = 0;                                       //Speed kleiner als 0 = 0
    else if (newSpeed >= 100) newSpeed = 100;                              //Speed größer als 100 = 100
    else if (newSpeed == speed + 1 || newSpeed == speed - 1) return;       // +- 1 ist keine Änderung
    speed = newSpeed;
    _speed_calculated = map(newSpeed, 0, 100, 0, 255);                     // Umrechen der 256 Bits in Geschwindigkeit 100%

    Serial.print(F("Setze Lüfter auf "));                                  // Makro, das den Text nicht in den RAM, sondern in den Flashspeicher legt
    Serial.print(speed);
    Serial.print(F("% / "));
    Serial.println(_speed_calculated);

    analogWrite(_pin_pwm, _speed_calculated);                              // Speed an die Lüfter übergeben
  }
}


FAN_CONTROLLER fan1(PIN_fan1_pwm, PIN_fan1_tacho);                         // Lüfter initialisieren
FAN_CONTROLLER fan2(PIN_fan2_pwm, PIN_fan2_tacho);


int Mode_Auto() {
  int value = map(bme280.getTemperature(), 25, 80, 0, 100);                // 25 Grad ist Lüfter aus, 80 Grad ist Lüfter 100%
  if (value <= 0) return 0;
  return value;
}


int Mode_Manu() {
  int value = analogRead(PIN_Poti);
  return map(value, 0, 1023, 0, 100);                  // Stand des Potis umgerechnet in die Drezahl in Prozent
}

void button_interrupt_handler() {
  if (initialisation_complete == true) {               // Trigger vor Initsialisierung verhindern, erst dann können Interupts funktionieren
    if (!interrupt_process_status) {
      if (digitalRead(PIN_Button) == HIGH) {
        interrupt_process_status = true;
      }
    }
  }
}

bool read_button() {
  int button_reading;                                  // statische Variablen, da wir alte Werte zwischen Funktionsaufrufen beibehalten müssen
  static bool switching_pending = false;
  static long int elapse_timer;
  if (interrupt_process_status == true) {
    button_reading = digitalRead(PIN_Button);
    if (button_reading == HIGH) {                      // Der Schalter ist gedrückt, also Start/Neustart, warten auf das Prellen des Tasters
      switching_pending = true;
      elapse_timer = millis();                         // Zeit nach dem Prellen
    }
    if (switching_pending && button_reading == LOW) {  // Der Schalter wurde gedrückt und jetzt losgelassen. Timer für Prellzeit abwarten
      if (millis() - elapse_timer >= debounce) {       // Nach der Prellzeit ist der Cycle abgeschlossen
        switching_pending = false;                     // Reset für den nächsten Tastdruck interrupt cycle
        interrupt_process_status = false;              // ISR wieder verfügbar, on/off/prell Cycle abgeschlossen
        return true;                                   // Schalter wurde gedrückt
      }
    }
  }
  return false;                                        // Entweder kein Knopfdruck oder der Cycle ist nicht abgeschlossen
}


void setup() {
  delay(200);
  Serial.begin(9600);
  Serial.println(F("Starting Fan PWM Controller V0.1"));
  pinMode(PIN_Poti, INPUT);
  pinMode(PIN_Button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_Button), button_interrupt_handler, interrupt_trigger_type);
  initialisation_complete = true;                                                                            // ISR ab jetzt zulassen
  if (!bme280.init()) Serial.println(F("Device error!"));                                                    // Sensor Test
}


void loop() {
  if (read_button()) {  
    Serial.println(F("Button pressed!!"));        
    mode++;
    if (mode > maxMode) mode = 0;                       // Nur Modus 0 und 1 nach 1 wieder 0.
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
  fan2.setSpeed(fanValue);
}
