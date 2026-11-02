// Smart Tap Monitor - ESP32
// SIMULATE 1 = Wokwi (potentiometers stand in for sensors)
// SIMULATE 0 = real hardware
#define SIMULATE 0

#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- Settings you can change ----------
#if SIMULATE
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";
#else
const char* WIFI_SSID = "Moto edge 60 pro";
const char* WIFI_PASS = "123321123";
#endif

const char* MQTT_HOST  = "broker.hivemq.com";
const char* TOPIC_DATA = "tapmon/iotttt/data";
const char* TOPIC_CMD  = "tapmon/iotttt/cmd";

const float TDS_LIMIT        = 500.0;   // ppm; above this = poor quality
const float TDS_CAL          = 1.0;     // calibration multiplier for TDS
const float PULSES_PER_LITRE = 450.0;   // YF-S201 typical value
const float FLOW_MIN         = 0.2;     // L/min; below this = no flow
const int   NO_SUPPLY_SECS   = 10;      // demo value (use 600 for real use)
const bool  AUTO_SHUTOFF     = true;    // close valve on poor quality
#define RELAY_ACTIVE_LOW 1              // set 0 if the valve acts inverted

// ---------- Pins ----------
const int PIN_FLOW     = 27;
const int PIN_TDS      = 34;
const int PIN_TEMP     = 4;
const int PIN_RELAY    = 26;
const int PIN_SIM_FLOW = 35;   // simulation only

// ---------- Objects and state ----------
OneWire oneWire(PIN_TEMP);
DallasTemperature tempSensor(&oneWire);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

volatile uint32_t pulseCount = 0;
void IRAM_ATTR onPulse() { pulseCount++; }

float flowLpm = 0, totalLitres = 0, tds = 0, tempC = 25;
bool valveOpen = true;
String status = "WORKING";
int noFlowSecs = 0;
unsigned long lastTick = 0, lastPublish = 0, lastWifiTry = 0, lastMqttTry = 0;

// ---------- Valve ----------
void setValve(bool open) {
  valveOpen = open;
  bool level = RELAY_ACTIVE_LOW ? !open : open;
  digitalWrite(PIN_RELAY, level ? HIGH : LOW);
}

// ---------- MQTT commands: send OPEN or CLOSE ----------
void onMessage(char* topic, byte* payload, unsigned int len) {
  Serial.print("MQTT message received on: ");
  Serial.println(topic);

  String msg;
  for (unsigned int i = 0; i < len; i++) {
    msg += (char)payload[i];
  }

  msg.trim();
  msg.toUpperCase();

  Serial.print("Command: ");
  Serial.println(msg);

  if (msg == "OPEN") {
    setValve(true);
    Serial.println("Valve command: OPEN");
  } else if (msg == "CLOSE") {
    setValve(false);
    Serial.println("Valve command: CLOSE");
  } else {
    Serial.println("Unknown MQTT command");
  }
}

// ---------- TDS ----------
float readTdsVoltage() {
  long sum = 0;
  for (int i = 0; i < 20; i++) { sum += analogRead(PIN_TDS); delay(5); }
  return (sum / 20.0) / 4095.0 * 3.3;
}

float voltageToTds(float v, float t) {
  float k = 1.0 + 0.02 * (t - 25.0);   // temperature compensation
  float vc = v / k;
  float ppm = (133.42 * vc * vc * vc - 255.86 * vc * vc + 857.39 * vc) * 0.5;
  return ppm * TDS_CAL;
}

// ---------- Status logic ----------
void updateStatus() {
  if (valveOpen && flowLpm < FLOW_MIN) noFlowSecs++; else noFlowSecs = 0;

  if (tds > TDS_LIMIT) {
    status = "POOR QUALITY";
    if (AUTO_SHUTOFF && valveOpen) setValve(false);
  } else if (!valveOpen) {
    status = "VALVE CLOSED";
  } else if (noFlowSecs >= NO_SUPPLY_SECS) {
    status = "NO SUPPLY";
  } else {
    status = "WORKING";
  }
}

// ---------- OLED ----------
void drawScreen() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);  oled.println(status);
  oled.setCursor(0, 11); oled.print("Flow: ");  oled.print(flowLpm, 1);    oled.println(" L/min");
  oled.setCursor(0, 22); oled.print("Total: "); oled.print(totalLitres, 1); oled.println(" L");
  oled.setCursor(0, 33); oled.print("TDS: ");   oled.print(tds, 0);         oled.println(" ppm");
  oled.setCursor(0, 44); oled.print("Temp: ");  oled.print(tempC, 1);       oled.println(" C");
  oled.setCursor(0, 55); oled.print("Valve: "); oled.println(valveOpen ? "OPEN" : "CLOSED");
  oled.display();
}

// ---------- Wi-Fi and MQTT (never blocks the sensors) ----------
void keepConnected() {
  // Retry Wi-Fi only when disconnected and not already connecting.
  if (WiFi.status() != WL_CONNECTED) {
    if (WiFi.status() == WL_IDLE_STATUS &&
        millis() - lastWifiTry >= 10000) {
      lastWifiTry = millis();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    return;
  }

  // Connect to MQTT once Wi-Fi is available.
  if (!mqtt.connected() && millis() - lastMqttTry >= 5000) {
    lastMqttTry = millis();
    String id = "tapmon-" + String(random(0xffff), HEX);

    if (mqtt.connect(id.c_str())) {
      Serial.println("MQTT connected");
      mqtt.subscribe(TOPIC_CMD);
      Serial.println("Subscribed to command topic");
    }

  }
}

void setup() {
  Serial.begin(115200);

  digitalWrite(PIN_RELAY, RELAY_ACTIVE_LOW ? HIGH : LOW);  // relay off at boot
  pinMode(PIN_RELAY, OUTPUT);
  setValve(true);

  pinMode(PIN_TDS, INPUT);
  analogReadResolution(12);
  tempSensor.begin();

#if SIMULATE
  pinMode(PIN_SIM_FLOW, INPUT);
#else
  pinMode(PIN_FLOW, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), onPulse, FALLING);
#endif

  Wire.begin(21, 22);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  mqtt.setServer(MQTT_HOST, 1883);
  mqtt.setCallback(onMessage);
  lastTick = millis();
}

void loop() {
  keepConnected();
  mqtt.loop();

  if (millis() - lastTick >= 1000) {
    float dt = (millis() - lastTick) / 1000.0;
    lastTick = millis();

#if SIMULATE
    flowLpm = analogRead(PIN_SIM_FLOW) / 4095.0 * 10.0;   // pot = 0 to 10 L/min
    totalLitres += flowLpm / 60.0 * dt;
#else
    noInterrupts();
    uint32_t p = pulseCount;
    pulseCount = 0;
    interrupts();
    float litres = p / PULSES_PER_LITRE;
    totalLitres += litres;
    flowLpm = litres / dt * 60.0;
#endif

    tempSensor.requestTemperatures();
    float t = tempSensor.getTempCByIndex(0);
    tempC = (t == DEVICE_DISCONNECTED_C) ? 22.0 : t;
    tds = voltageToTds(readTdsVoltage(), tempC);

    updateStatus();
    drawScreen();

    Serial.print(status);
    Serial.print(" | flow ");  Serial.print(flowLpm);
    // Serial.print(" L/min | total "); Serial.print(totalLitres);
    Serial.print(" L | TDS ");  Serial.print(tds);
    Serial.print(" ppm | temp "); Serial.print(tempC);
    Serial.print(" C | valve "); Serial.println(valveOpen ? "OPEN" : "CLOSED");
  }

  if (millis() - lastPublish >= 5000 && mqtt.connected()) {
    lastPublish = millis();
    char buf[200];
    snprintf(buf, sizeof(buf),
      R"({"flow":%.2f,"tds":%.0f,"temp":%.1f,"valve":"%s","status":"%s"})",
      flowLpm, tds, tempC, valveOpen ? "OPEN" : "CLOSED", status.c_str());
    mqtt.publish(TOPIC_DATA, buf);
  }
}