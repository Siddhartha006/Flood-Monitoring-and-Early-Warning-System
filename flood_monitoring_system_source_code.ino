#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// =====================================================
// OLED
// =====================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

// =====================================================
// BME280
// =====================================================


Adafruit_BME280 bme;
bool bmeFound = false;

// =====================================================
// THINGSBOARD CLOUD
// =====================================================

// Enter your Wi-Fi details here
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Enter the access token of your ThingsBoard device.
const char* TB_HOST = "mqtt.thingsboard.cloud";
const int TB_PORT = 8883;
const char* TB_ACCESS_TOKEN = "YOUR_THINGSBOARD_DEVICE_ACCESS_TOKEN";

WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);

const char* TB_TELEMETRY_TOPIC = "v1/devices/me/telemetry";

// =====================================================
// PIN DEFINITIONS
// =====================================================

// JSN-SR04T
#define TRIG_PIN D5
#define ECHO_PIN D6

// LEDs
#define GREEN_LED  D0
#define YELLOW_LED D3
#define RED_LED    D4

// Buzzer
#define BUZZER_PIN D7

// =====================================================
// SENSOR SETTINGS
// =====================================================

// testing showed unreliable readings below ~20 cm.
// We keep a safety margin.
const float MIN_VALID_DISTANCE = 22.0;

const float MAX_VALID_DISTANCE = 600.0;

// =====================================================
// FILTER SETTINGS
// =====================================================

const int MEDIAN_SAMPLES = 7;
const int AVERAGE_SAMPLES = 5;

// =====================================================
// TIMING
// =====================================================

const unsigned long CALIBRATION_TIME = 30000;

const unsigned long MEASUREMENT_INTERVAL = 5000;

// =====================================================
// WATER LEVEL THRESHOLDS
// 8 cm TEST CONTAINER
// =====================================================

const float MODERATE_DEPTH = 2.0;
const float HIGH_DEPTH     = 4.0;
const float CRITICAL_DEPTH = 6.0;

// =====================================================
// STATUS CONSTANTS
// =====================================================

const int STATUS_NORMAL   = 0;
const int STATUS_MODERATE = 1;
const int STATUS_HIGH     = 2;
const int STATUS_CRITICAL = 3;

// Current status
int currentStatus = STATUS_NORMAL;

// =====================================================
// VARIABLES
// =====================================================

float referenceDistance = 0.0;

float currentDistance = 0.0;

float filteredDistance = 0.0;

float waterDepth = 0.0;

float previousDepth = 0.0;

float riseRate = 0.0;

unsigned long previousMeasurementTime = 0;

// =====================================================
// MOVING AVERAGE BUFFER
// =====================================================

float distanceBuffer[AVERAGE_SAMPLES];

int bufferIndex = 0;

int bufferCount = 0;

// =====================================================
// READ ULTRASONIC DISTANCE
// =====================================================

float readDistance()
{
  digitalWrite(TRIG_PIN, LOW);

  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);

  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(
    ECHO_PIN,
    HIGH,
    30000
  );

  if (duration == 0)
  {
    return -1;
  }

  float distance =
    (duration * 0.0343) / 2.0;

  return distance;
}

// =====================================================
// MEDIAN FILTER
// =====================================================

float getMedianDistance()
{
  float readings[MEDIAN_SAMPLES];

  int validCount = 0;

  for (int i = 0; i < MEDIAN_SAMPLES; i++)
  {
    float d = readDistance();

    if (d >= MIN_VALID_DISTANCE &&
        d <= MAX_VALID_DISTANCE)
    {
      readings[validCount] = d;

      validCount++;
    }

    delay(60);
  }

  if (validCount == 0)
  {
    return -1;
  }

  // Sort readings
  for (int i = 0; i < validCount - 1; i++)
  {
    for (int j = i + 1; j < validCount; j++)
    {
      if (readings[j] < readings[i])
      {
        float temp = readings[i];

        readings[i] = readings[j];

        readings[j] = temp;
      }
    }
  }

  // Return median
  if (validCount % 2 == 1)
  {
    return readings[validCount / 2];
  }
  else
  {
    return
      (readings[(validCount / 2) - 1] +
       readings[validCount / 2]) / 2.0;
  }
}

// =====================================================
// MOVING AVERAGE
// =====================================================

float movingAverage(float newValue)
{
  distanceBuffer[bufferIndex] = newValue;

  bufferIndex++;

  if (bufferIndex >= AVERAGE_SAMPLES)
  {
    bufferIndex = 0;
  }

  if (bufferCount < AVERAGE_SAMPLES)
  {
    bufferCount++;
  }

  float sum = 0;

  for (int i = 0; i < bufferCount; i++)
  {
    sum += distanceBuffer[i];
  }

  return sum / bufferCount;
}

// =====================================================
// DETERMINE STATUS
// =====================================================

int determineStatus(float depth)
{
  if (depth < MODERATE_DEPTH)
  {
    return STATUS_NORMAL;
  }

  else if (depth < HIGH_DEPTH)
  {
    return STATUS_MODERATE;
  }

  else if (depth < CRITICAL_DEPTH)
  {
    return STATUS_HIGH;
  }

  else
  {
    return STATUS_CRITICAL;
  }
}

// =====================================================
// STATUS TEXT
// =====================================================

const char* getStatusText(int status)
{
  if (status == STATUS_NORMAL)
  {
    return "NORMAL";
  }

  else if (status == STATUS_MODERATE)
  {
    return "MODERATE";
  }

  else if (status == STATUS_HIGH)
  {
    return "HIGH";
  }

  else if (status == STATUS_CRITICAL)
  {
    return "CRITICAL";
  }

  else
  {
    return "UNKNOWN";
  }
}

// =====================================================
// LED + BUZZER CONTROL
// =====================================================

void updateIndicators(int status)
{
  // Turn everything OFF
  digitalWrite(GREEN_LED, LOW);

  digitalWrite(YELLOW_LED, LOW);

  digitalWrite(RED_LED, LOW);

  digitalWrite(BUZZER_PIN, LOW);

  // NORMAL
  if (status == STATUS_NORMAL)
  {
    digitalWrite(GREEN_LED, HIGH);
  }

  // MODERATE
  else if (status == STATUS_MODERATE)
  {
    digitalWrite(YELLOW_LED, HIGH);
  }

  // HIGH
  else if (status == STATUS_HIGH)
  {
    digitalWrite(RED_LED, HIGH);
  }

  // CRITICAL
  else if (status == STATUS_CRITICAL)
  {
    digitalWrite(RED_LED, HIGH);

    digitalWrite(BUZZER_PIN, HIGH);
  }
}

// =====================================================
// OLED DISPLAY
// =====================================================

void updateOLED()
{
  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);

  // Water depth
  display.setTextSize(1);

  display.setCursor(0, 0);

  display.print("Water Depth:");

  display.setTextSize(2);

  display.setCursor(0, 10);

  display.print(waterDepth, 1);

  display.print("cm");

  // Distance
  display.setTextSize(1);

  display.setCursor(0, 32);

  display.print("Distance:");

  display.print(filteredDistance, 1);

  display.print("cm");

  // Status
  display.setCursor(0, 43);

  display.print("Status:");

  display.print(
    getStatusText(currentStatus)
  );

  // Rise rate
  display.setCursor(0, 55);

  display.print("Rise:");

  display.print(riseRate, 2);

  display.print("cm/m");

  display.display();
}

// =====================================================
// CALIBRATION
// =====================================================

void calibrateSensor()
{
  Serial.println();
  Serial.println("==============================");
  Serial.println("STARTING CALIBRATION");
  Serial.println("==============================");

  Serial.println(
    "Keep container EMPTY."
  );

  Serial.println(
    "Do NOT move the sensor."
  );

  display.clearDisplay();

  display.setTextSize(1);

  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);

  display.println("CALIBRATION");

  display.setCursor(0, 15);

  display.println("Keep container empty");

  display.setCursor(0, 30);

  display.println("Do not move sensor");

  display.display();

  unsigned long startTime = millis();

  float sum = 0;

  int count = 0;

  while (
    millis() - startTime <
    CALIBRATION_TIME
  )
  {
    float d = getMedianDistance();

    if (d >= MIN_VALID_DISTANCE)
    {
      sum += d;

      count++;

      Serial.print(
        "Calibration distance: "
      );

      Serial.print(d);

      Serial.println(" cm");
    }

    delay(300);
  }

  // Calculate reference distance
  if (count > 0)
  {
    referenceDistance =
      sum / count;
  }

  else
  {
    // Fallback
    referenceDistance = 58.0;

    Serial.println(
      "Calibration failed!"
    );

    Serial.println(
      "Using 58 cm fallback."
    );
  }

  Serial.println();

  Serial.print(
    "Reference Distance = "
  );

  Serial.print(
    referenceDistance,
    2
  );

  Serial.println(" cm");

  // Display result
  display.clearDisplay();

  display.setTextSize(1);

  display.setCursor(0, 0);

  display.println(
    "CALIBRATION DONE"
  );

  display.setCursor(0, 20);

  display.print(
    "Reference: "
  );

  display.print(
    referenceDistance,
    1
  );

  display.println(" cm");

  display.display();

  delay(2000);

  // Reset filter
  bufferIndex = 0;

  bufferCount = 0;

  for (int i = 0;
       i < AVERAGE_SAMPLES;
       i++)
  {
    distanceBuffer[i] =
      referenceDistance;
  }

  filteredDistance =
    referenceDistance;

  currentDistance =
    referenceDistance;

  waterDepth = 0;

  previousDepth = 0;

  riseRate = 0;
}


// =====================================================
// WI-FI
// =====================================================

void connectWiFi()
{
  if (WiFi.status() == WL_CONNECTED)
    return;

  Serial.print("Connecting to Wi-Fi");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - start < 20000)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Wi-Fi connected!");
    Serial.print("ESP8266 IP: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("Wi-Fi connection failed.");
  }
}


// =====================================================
// THINGSBOARD CLOUD CONNECTION
// =====================================================

void connectThingsBoard()
{
  if (WiFi.status() != WL_CONNECTED)
    return;

  if (mqttClient.connected())
    return;

  Serial.print("Connecting to ThingsBoard Cloud... ");

  String clientID = "FloodMonitoringNode1";

  if (mqttClient.connect(
        clientID.c_str(),
        TB_ACCESS_TOKEN,
        ""
      ))
  {
    Serial.println("CONNECTED!");
  }
  else
  {
    Serial.print("FAILED, MQTT state = ");
    Serial.println(mqttClient.state());
  }
}


// =====================================================
// SEND TELEMETRY TO THINGSBOARD
// =====================================================

void sendTelemetry()
{
  if (!mqttClient.connected())
  {
    Serial.println("Telemetry not sent: ThingsBoard is offline.");
    return;
  }

  float temperature = 0.0;
  float humidity = 0.0;
  float pressure = 0.0;

  if (bmeFound)
  {
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
  }

  char payload[350];

  snprintf(
    payload,
    sizeof(payload),
    "{\"water_depth_cm\":%.2f,"
    "\"rate_of_rise_cm_min\":%.2f,"
    "\"temperature_C\":%.2f,"
    "\"humidity_percent\":%.2f,"
    "\"pressure_hPa\":%.2f,"
    "\"flood_status\":\"%s\","
    "\"flood_status_code\":%d}",
    waterDepth,
    riseRate,
    temperature,
    humidity,
    pressure,
    getStatusText(currentStatus),
    currentStatus
  );

  if (mqttClient.publish(
        TB_TELEMETRY_TOPIC,
        payload
      ))
  {
    Serial.println("ThingsBoard telemetry sent:");
    Serial.println(payload);
  }
  else
  {
    Serial.println("ThingsBoard telemetry FAILED!");
  }
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();

  Serial.println("==============================");

  Serial.println(
    "IoT FLOOD MONITORING SYSTEM"
  );

  Serial.println(
    "NodeMCU ESP8266"
  );

  Serial.println("==============================");

  // ===================================================
  // THINGSBOARD CLOUD SETUP
  // ===================================================


  secureClient.setInsecure();

  mqttClient.setServer(
    TB_HOST,
    TB_PORT
  );

  connectWiFi();

  // Pin configuration
  pinMode(TRIG_PIN, OUTPUT);

  pinMode(ECHO_PIN, INPUT);

  pinMode(GREEN_LED, OUTPUT);

  pinMode(YELLOW_LED, OUTPUT);

  pinMode(RED_LED, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  // Initial states
  digitalWrite(TRIG_PIN, LOW);

  digitalWrite(GREEN_LED, LOW);

  digitalWrite(YELLOW_LED, LOW);

  digitalWrite(RED_LED, LOW);

  digitalWrite(BUZZER_PIN, LOW);

  // I2C
  Wire.begin(D2, D1);

  // ===================================================
  // OLED
  // ===================================================

  if (
    !display.begin(
      SSD1306_SWITCHCAPVCC,
      OLED_ADDRESS
    )
  )
  {
    Serial.println(
      "OLED NOT FOUND!"
    );

    while (true)
    {
      delay(1000);
    }
  }

  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);

  display.setCursor(0, 0);

  display.println(
    "Flood Monitoring"
  );

  display.setCursor(0, 15);

  display.println(
    "System Starting..."
  );

  display.display();

  delay(2000);

  // ===================================================
  // BME280
  // ===================================================

  if (bme.begin(0x76))
  {
    bmeFound = true;

    Serial.println(
      "BME280 found at 0x76"
    );
  }

  else if (bme.begin(0x77))
  {
    bmeFound = true;

    Serial.println(
      "BME280 found at 0x77"
    );
  }

  else
  {
    bmeFound = false;

    Serial.println(
      "BME280 NOT FOUND"
    );
  }

  // ===================================================
  // CALIBRATION
  // ===================================================

  calibrateSensor();

  previousMeasurementTime =
    millis();

  Serial.println();

  Serial.println(
    "SYSTEM READY"
  );

  Serial.println(
    "Start adding water slowly."
  );

  Serial.println();

  // Initial cloud connection attempt.
  connectThingsBoard();

  if (!mqttClient.connected())
  {
    Serial.println(
      "Cloud unavailable. Local monitoring will continue."
    );
  }
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  // Keep MQTT connection alive.
  if (WiFi.status() != WL_CONNECTED)
  {
    connectWiFi();
  }

  if (!mqttClient.connected())
  {
    connectThingsBoard();
  }

  mqttClient.loop();

  unsigned long currentTime =
    millis();

  if (
    currentTime -
    previousMeasurementTime >=
    MEASUREMENT_INTERVAL
  )
  {
    previousMeasurementTime =
      currentTime;

    // =================================================
    // READ SENSOR
    // =================================================

    float measuredDistance =
      getMedianDistance();

    if (measuredDistance < 0)
    {
      Serial.println(
        "Invalid ultrasonic reading."
      );

      return;
    }

    currentDistance =
      measuredDistance;

    // =================================================
    // FILTER
    // =================================================

    filteredDistance =
      movingAverage(
        currentDistance
      );

    // =================================================
    // WATER DEPTH
    // =================================================

    waterDepth =
      referenceDistance -
      filteredDistance;

    if (waterDepth < 0)
    {
      waterDepth = 0;
    }

    // =================================================
    // RISE RATE
    // =================================================

    float timeMinutes =
      (MEASUREMENT_INTERVAL / 1000.0)
      / 60.0;

    float depthChange =
      waterDepth -
      previousDepth;

    // Ignore tiny changes
    if (abs(depthChange) < 0.15)
    {
      depthChange = 0;
    }

    riseRate =
      depthChange /
      timeMinutes;

    // Limit rate
    if (riseRate > 5.0)
    {
      riseRate = 5.0;
    }

    if (riseRate < -5.0)
    {
      riseRate = -5.0;
    }

    previousDepth =
      waterDepth;

    // =================================================
    // STATUS
    // =================================================

    currentStatus =
      determineStatus(
        waterDepth
      );

    // =================================================
    // INDICATORS
    // =================================================

    updateIndicators(
      currentStatus
    );

    // =================================================
    // OLED
    // =================================================

    updateOLED();

    // =================================================
    // SERIAL MONITOR
    // =================================================

    Serial.println();

    Serial.println(
      "------------------------------"
    );

    Serial.print(
      "Reference Distance : "
    );

    Serial.print(
      referenceDistance,
      2
    );

    Serial.println(" cm");

    Serial.print(
      "Measured Distance  : "
    );

    Serial.print(
      currentDistance,
      2
    );

    Serial.println(" cm");

    Serial.print(
      "Filtered Distance  : "
    );

    Serial.print(
      filteredDistance,
      2
    );

    Serial.println(" cm");

    Serial.print(
      "Water Depth        : "
    );

    Serial.print(
      waterDepth,
      2
    );

    Serial.println(" cm");

    Serial.print(
      "Rise Rate          : "
    );

    Serial.print(
      riseRate,
      2
    );

    Serial.println(
      " cm/min"
    );

    Serial.print(
      "Status             : "
    );

    Serial.println(
      getStatusText(
        currentStatus
      )
    );

    // =================================================
    // BME280 DATA
    // =================================================

    if (bmeFound)
    {
      Serial.print(
        "Temperature        : "
      );

      Serial.print(
        bme.readTemperature(),
        2
      );

      Serial.println(" C");

      Serial.print(
        "Humidity           : "
      );

      Serial.print(
        bme.readHumidity(),
        2
      );

      Serial.println(" %");

      Serial.print(
        "Pressure           : "
      );

      Serial.print(
        bme.readPressure() / 100.0F,
        2
      );

      Serial.println(" hPa");
    }

    // =================================================
    // THINGSBOARD TELEMETRY
    // =================================================

    sendTelemetry();

    Serial.println(
      "------------------------------"
    );
  }
}