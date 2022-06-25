#include <Arduino.h>

// Econode SamrtTrap Gen3 Pin map 
// https://github.com/arduino/ArduinoCore-samd/blob/master/variants/arduino_mzero/variant.cpp
#define PIN_LED_YELLOW 11 // SAMD port PA16
#define PIN_LED_GREEN 13 // SAMD port PA17
#define PIN_LED_RED 2 // SAMD port PA14
// #define PIN_SENSORA 14 // SAMD port PA02
#define PIN_SENSORB 15 // SAMD port PB08
#define PIN_SENSOR_TILT 39 // SAMD port PA21


/*
GPS port
+-----+----------+------------+--------+---------+
| PIN | Function | GPS Module | SAMD21 | Arduino |
+-----+----------+------------+--------+---------+
|  1  | N/C      | PPS        |        |         |
|  2  | 3V3      | VIN        |        |         |
|  3  | GND      | GND        |        |         |
|  4  | GPS-RX   | Serial RX  |        | Serial1 |
|  5  | GPS-TX   | Serial TX  |        | Serial1 |
|  6  | GPS_IO-1 | GPS FIX    | PA08   | 4       |
|  7  | N/C      | V-Bat      |        |         |
|  8  | GPS-IO-2 | Enable     | PA09   | 3       |
|  9  | N/C      | 3.3v (out) |        |         |
+-----+----------+------------+--------+---------+
*/

#define PIN_GPS_ENABLE 3
#define PIN_GPS_FIX 4

/* Gen3 Rev 4

Grove standard is;
1 - GND
2 - 3V3
3 - SDA (PB08) / SensorB
4 - SCL (PB09) / SensorA
*/
#define PIN_SENSORA 16 // SAMD port PB09
#define PIN_SENSOR_TEST_MODE 14 // SAMD port PA02

// External interrupt handler mapping
#define EIC_WAKEUP_SENSORA EIC_WAKEUP_WAKEUPEN9 // SensorA = pin 16 / EIC 9
#define EIC_WAKEUP_SENSORB EIC_WAKEUP_WAKEUPEN8 // SensorB = pin 15 / EIC 8
#define EIC_WAKEUP_SENSOR_TEST_MODE EIC_WAKEUP_WAKEUPEN2 // Sensor Test mode = pin 14 / EIC 2
// EIC->WAKEUP.reg = EIC_WAKEUP_SENSORB | EIC_WAKEUP_SENSORA | EIC_WAKEUP_SENSOR_TEST_MODE;

// Battery Monitor
#define PIN_BATTERY_SENSOR_ENABLE 36 // PA18
#define PIN_BATTERY_SENSOR 17 // PA04
// Voltage divider for battery
// GND -> 10K -> Sensor -> 27K = Ratio of 3.7
// 3.3V Ref * 3.7 = 12.21V
#define BATTERY_VOLTAGE_DIVIDER_RATIO 12.22

/*
 Econode SAMD21 / RFM95 Pin definitions.
     +--------+--------+---------+
     | RFM95x | SAMD21 | Arduino |
     +--------+--------+---------+
     | NSS    | PA06   | 8       |
     | Reset  | PB23   | 31      |
     | DIO 0  | PB22   | 30      |
     | DIO 1  | PA15   | 5       |
     | DIO 2  | PA20   | 6       |
     +--------+--------+---------+
*/
#define PIN_LORA_RADIO_NSS 8
#define PIN_LORA_RADIO_RXEN 0
#define PIN_LORA_RADIO_TXEN 0
#define PIN_LORA_RADIO_RST 31
// #define PIN_LORA_RADIO_DIO0 30
#define PIN_LORA_RADIO_DIO0 38
#define PIN_LORA_RADIO_DIO1 5
#define PIN_LORA_RADIO_DIO2 6

#define PIN_USB_SENSOR 18 // PA05
#define PIN_EEPROM_SDA 20
#define PIN_EEPROM_SCL 21

#define PIN_RFM9X_MOSI 23
#define PIN_RFM9X_SCK 24
#define PIN_RFM9X_MISO 22

// Read battery volts in milli volts.
uint32_t enReadBatteryMilliVolts(){
  uint32_t _batteryVolts;
  pinMode(PIN_BATTERY_SENSOR_ENABLE, OUTPUT);
  pinMode(PIN_BATTERY_SENSOR,INPUT);
  digitalWrite(PIN_BATTERY_SENSOR_ENABLE,HIGH);
  delay(10);
  _batteryVolts = analogRead(PIN_BATTERY_SENSOR) * BATTERY_VOLTAGE_DIVIDER_RATIO;
  digitalWrite(PIN_BATTERY_SENSOR_ENABLE,LOW);
  return _batteryVolts;
}

void setup() {
  // Setup LEDs
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_YELLOW, LOW);
  // Setup USB/CDC Serial port
  Serial.begin(115200);
}

int ledIndex=0;
void loop(){
  Serial.println("Hello from USB Serial");
  Serial.print("Battery volts");
  Serial.println( enReadBatteryMilliVolts() );

  if( ++ledIndex > 2 ) ledIndex=0;
  digitalWrite(PIN_LED_GREEN, ledIndex==0 );
  digitalWrite(PIN_LED_YELLOW, ledIndex==1 );
  digitalWrite(PIN_LED_RED, ledIndex==2 );
  delay(1000);
}