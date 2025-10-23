#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <PID_v1.h>
#include <EEPROM.h>

#define DHTPIN 2
#define DHTTYPE DHT11
#define ONE_WIRE_BUS 3
#define DS1307_I2C_ADDRESS 0x68

const byte soilMoisturePins[] = {A3, A4};
const int NUM_SOIL_SENSORS = sizeof(soilMoisturePins) / sizeof(soilMoisturePins[0]);

#define EEPROM_ADDR_LOW  0
#define EEPROM_ADDR_HIGH 2

bool DEBUG = true;

struct Relay {
  byte pin;
  const __FlashStringHelper* name;
};
Relay relays[] = {
  {5, F("light")},
  {4, F("fan_heater")},
  {9, F("air_heater")},
  {11, F("soil_pump")},
  {10, F("fan_in")},
  {6, F("fan_out")},
  {7, F("air_pump")},
  {8, F("soil_heater")}
};

DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

struct SensorData {
  double airTemp;
  double airHum;
  double soilTemp;
  double soilHum;
};

struct Time {
  byte hour;
  byte minute;
  byte second;
};

byte bcdToDec(byte val) { return ((val / 16 * 10) + (val % 16)); }

Time readTime() {
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(DS1307_I2C_ADDRESS, 7);
  Time t;
  t.second = bcdToDec(Wire.read() & 0x7F);
  t.minute = bcdToDec(Wire.read());
  t.hour = bcdToDec(Wire.read() & 0x3F);
  return t;
}

void setupRelays() {
  for (auto &r : relays) {
    pinMode(r.pin, OUTPUT);
    digitalWrite(r.pin, HIGH);
  }
}

inline void setRelay(byte pin, bool on) {
  digitalWrite(pin, on ? LOW : HIGH);
}

int map_low = 1000;
int map_high = 200;

void loadSoilCalibration() {
  EEPROM.get(EEPROM_ADDR_LOW, map_low);
  EEPROM.get(EEPROM_ADDR_HIGH, map_high);
  if (map_low < 200 || map_low > 1000 || map_high < 100 || map_high > 1000) {
    map_low = 1000;
    map_high = 200;
  }
  if (DEBUG) {
    Serial.print(F("Soil calibration: dry="));
    Serial.print(map_low);
    Serial.print(F(" wet="));
    Serial.println(map_high);
  }
}

double readSoilMoisture() {
  int sum = 0;
  for (int i = 0; i < NUM_SOIL_SENSORS; i++) sum += analogRead(soilMoisturePins[i]);
  double avg = sum / (double)NUM_SOIL_SENSORS;
  double val = map(avg, map_low, map_high, 0, 100);
  return constrain(val, 0, 100);
}

SensorData readSensors() {
  SensorData data;
  data.airTemp = dht.readTemperature();
  data.airHum = dht.readHumidity();
  sensors.requestTemperatures();
  data.soilTemp = sensors.getTempCByIndex(0);
  data.soilHum = readSoilMoisture();
  if (isnan(data.airTemp)) data.airTemp = 25;
  if (isnan(data.airHum)) data.airHum = 60;
  if (data.soilTemp < -10 || data.soilTemp > 80) data.soilTemp = 25;
  return data;
}

double airTempInput, airTempOutput, airTempSetpoint = 24.0;
double soilTempInput, soilTempOutput, soilTempSetpoint = 25.0;

PID airPID(&airTempInput, &airTempOutput, &airTempSetpoint, 2.0, 5.0, 1.0, DIRECT);
PID soilPID(&soilTempInput, &soilTempOutput, &soilTempSetpoint, 2.5, 4.0, 1.0, DIRECT);

const unsigned long TIME_INTERVAL = 10000;
unsigned long airCycleStart = 0;
unsigned long soilCycleStart = 0;

void controlLight(int hour) {
  setRelay(relays[0].pin, (hour >= 6 && hour <= 23));
}

void controlAirHumidity(double hum) {
  if (hum > 90) {
    setRelay(relays[4].pin, true);
    setRelay(relays[5].pin, true);
  } else if (hum < 80) {
    setRelay(relays[4].pin, true);
    setRelay(relays[6].pin, true);
  } else {
    setRelay(relays[4].pin, false);
    setRelay(relays[5].pin, false);
    setRelay(relays[6].pin, false);
  }
}

void controlSoilHumidity(double hum) {
  setRelay(relays[3].pin, hum < 85);
}

void controlAirTemperature(double currentTemp) {
  airTempInput = currentTemp;
  airPID.Compute();
  unsigned long now = millis();
  if (now - airCycleStart >= TIME_INTERVAL) airCycleStart = now;
  int onTime = (int)(airTempOutput / 255.0 * TIME_INTERVAL);
  setRelay(relays[2].pin, (now - airCycleStart) < onTime);
  if (currentTemp > airTempSetpoint + 2) {
    setRelay(relays[4].pin, true);
    setRelay(relays[5].pin, true);
  } else if (currentTemp < airTempSetpoint - 1) {
    setRelay(relays[4].pin, false);
    setRelay(relays[5].pin, false);
  }
}

void controlSoilTemperature(double currentTemp) {
  soilTempInput = currentTemp;
  soilPID.Compute();
  unsigned long now = millis();
  if (now - soilCycleStart >= TIME_INTERVAL) soilCycleStart = now;
  int onTime = (int)(soilTempOutput / 255.0 * TIME_INTERVAL);
  setRelay(relays[7].pin, (now - soilCycleStart) < onTime);
}

void printData(const Time &t, const SensorData &data) {
  if (!DEBUG) return;
  Serial.print(F("["));
  if (t.hour < 10) Serial.print('0');
  Serial.print(t.hour); Serial.print(':');
  if (t.minute < 10) Serial.print('0');
  Serial.print(t.minute); Serial.print(':');
  if (t.second < 10) Serial.print('0');
  Serial.print(t.second); Serial.print(F("] "));
  Serial.print(F("Air "));
  Serial.print(data.airTemp); Serial.print(F("C "));
  Serial.print(data.airHum); Serial.print(F("% | Soil "));
  Serial.print(data.soilTemp); Serial.print(F("C "));
  Serial.print(data.soilHum); Serial.println(F("%"));
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupRelays();
  dht.begin();
  sensors.begin();
  loadSoilCalibration();
  airPID.SetMode(AUTOMATIC);
  soilPID.SetMode(AUTOMATIC);
  airPID.SetOutputLimits(0, 255);
  soilPID.SetOutputLimits(0, 255);
  if (DEBUG) Serial.println(F("Greenhouse Controller Initialized"));
}

void loop() {
  const Time t = readTime();
  const SensorData data = readSensors();
  printData(t, data);
  controlLight(t.hour);
  controlAirHumidity(data.airHum);
  controlSoilHumidity(data.soilHum);
  controlAirTemperature(data.airTemp);
  controlSoilTemperature(data.soilTemp);
}
