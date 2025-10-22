#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <PID_v1.h>
#include <EEPROM.h>

// ============================================================================
// ======================= HARDWARE CONFIGURATION =============================
// ============================================================================

// DHT11 sensor pin and type
#define DHTPIN 2
#define DHTTYPE DHT11

// DS18B20 OneWire bus
#define ONE_WIRE_BUS 3

// DS1307 I2C address
#define DS1307_I2C_ADDRESS 0x68

// Analog inputs for soil moisture sensors
const byte soilMoisturePins[] = {A3, A4};
const int NUM_SOIL_SENSORS = sizeof(soilMoisturePins) / sizeof(soilMoisturePins[0]);

// EEPROM addresses for soil calibration values
#define EEPROM_ADDR_LOW  0
#define EEPROM_ADDR_HIGH 2

// Relay configuration (active LOW logic)
struct Relay {
  byte pin;
  const char* name;
};
Relay relays[] = {
  {5, "light"},
  {4, "fan_heater"},
  {9, "air_heater"},
  {11, "soil_pump"},
  {10, "fan_in"},
  {6, "fan_out"},
  {7, "air_pump"},
  {8, "soil_heater"}
};

// ============================================================================
// ============================= SENSOR OBJECTS ===============================
// ============================================================================

DHT dht(DHTPIN, DHTTYPE);            // Air temperature & humidity sensor
OneWire oneWire(ONE_WIRE_BUS);       // OneWire bus for DS18B20
DallasTemperature sensors(&oneWire); // Soil temperature sensor

// ============================================================================
// ============================= DATA STRUCTURES ==============================
// ============================================================================

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

// ============================================================================
// ============================= RTC FUNCTIONS ================================
// ============================================================================

byte decToBcd(byte val) { return ((val / 10 * 16) + (val % 10)); }

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

// ============================================================================
// ============================ RELAY FUNCTIONS ===============================
// ============================================================================

void setupRelays() {
  for (auto &r : relays) {
    pinMode(r.pin, OUTPUT);
    digitalWrite(r.pin, HIGH);
  }
}

void setRelay(byte pin, bool on) {
  digitalWrite(pin, on ? LOW : HIGH);
}

// ============================================================================
// ========================= SOIL MOISTURE CALIBRATION ========================
// ============================================================================

int map_low = 1000;  // Default dry calibration
int map_high = 200;  // Default wet calibration

void loadSoilCalibration() {
  EEPROM.get(EEPROM_ADDR_LOW, map_low);
  EEPROM.get(EEPROM_ADDR_HIGH, map_high);
  if (map_low < 200 || map_low > 1000 || map_high < 100 || map_high > 1000) {
    map_low = 1000;
    map_high = 200;
  }
  Serial.print("[INFO] Soil calibration loaded. Dry=");
  Serial.print(map_low);
  Serial.print(" Wet=");
  Serial.println(map_high);
}

void calibrateSoilMoisture() {
  Serial.println("=== Soil Calibration Mode ===");
  Serial.println("Step 1: Keep sensors DRY and wait 5 seconds...");
  delay(5000);
  int drySum = 0;
  for (int i = 0; i < NUM_SOIL_SENSORS; i++) drySum += analogRead(soilMoisturePins[i]);
  map_low = drySum / NUM_SOIL_SENSORS;
  Serial.print("Dry average: "); Serial.println(map_low);

  Serial.println("Step 2: Place sensors in WATER and wait 5 seconds...");
  delay(5000);
  int wetSum = 0;
  for (int i = 0; i < NUM_SOIL_SENSORS; i++) wetSum += analogRead(soilMoisturePins[i]);
  map_high = wetSum / NUM_SOIL_SENSORS;
  Serial.print("Wet average: "); Serial.println(map_high);

  EEPROM.put(EEPROM_ADDR_LOW, map_low);
  EEPROM.put(EEPROM_ADDR_HIGH, map_high);
  Serial.println("[OK] Calibration saved to EEPROM.");
}

// ============================================================================
// ============================ SENSOR READING ================================
// ============================================================================

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

  // Fallback values in case of sensor errors
  if (isnan(data.airTemp)) data.airTemp = 25;
  if (isnan(data.airHum)) data.airHum = 60;
  if (data.soilTemp < -10 || data.soilTemp > 80) data.soilTemp = 25;

  return data;
}

// ============================================================================
// ============================== PID CONTROLLERS =============================
// ============================================================================

double airTempInput, airTempOutput, airTempSetpoint = 24.0;

double soilTempInput, soilTempOutput, soilTempSetpoint = 25.0;

PID airPID(&airTempInput, &airTempOutput, &airTempSetpoint, 2.0, 5.0, 1.0, DIRECT);
PID soilPID(&soilTempInput, &soilTempOutput, &soilTempSetpoint, 2.5, 4.0, 1.0, DIRECT);

// ============================================================================
// =========================== CONTROL CONSTANTS ==============================
// ============================================================================

const double DEAD_BAND = 0.5;       
const unsigned long TIME_INTERVAL = 10000; 
unsigned long airCycleStart = 0;
unsigned long soilCycleStart = 0;

// ============================================================================
// ============================== CONTROL LOGIC ===============================
// ============================================================================

void controlLight(int hour) {
  bool on = (hour >= 6 && hour <= 23);
  setRelay(relays[0].pin, on);
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
  bool heaterState = (now - airCycleStart) < onTime;
  setRelay(relays[2].pin, heaterState);

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
  bool heaterState = (now - soilCycleStart) < onTime;
  setRelay(relays[7].pin, heaterState);
}

// ============================================================================
// ============================ PRINTING FUNCTION =============================
// ============================================================================

void printData(Time t, SensorData data) {
  Serial.print("\n[");
  if (t.hour < 10) Serial.print('0');
  Serial.print(t.hour); Serial.print(':');
  if (t.minute < 10) Serial.print('0');
  Serial.print(t.minute); Serial.print(':');
  if (t.second < 10) Serial.print('0');
  Serial.print(t.second); Serial.println("]");
  Serial.print("Air: "); Serial.print(data.airTemp); Serial.print("°C ");
  Serial.print(data.airHum); Serial.println("%");
  Serial.print("Soil: "); Serial.print(data.soilTemp); Serial.print("°C ");
  Serial.print(data.soilHum); Serial.println("%");
}

// ============================================================================
// ============================== SETUP FUNCTION ==============================
// ============================================================================

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

  Serial.println("Optimized Greenhouse Controller Initialized.\n");
}

// ============================================================================
// =============================== MAIN LOOP ==================================
// ============================================================================

void loop() {
  Time t = readTime();    
  SensorData data = readSensors();

  printData(t, data);         

  controlLight(t.hour);        
  controlAirHumidity(data.airHum);
  controlSoilHumidity(data.soilHum);
  controlAirTemperature(data.airTemp);
  controlSoilTemperature(data.soilTemp);
}
