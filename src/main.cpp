#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <FreeRTOS.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <SparkFun_AS3935.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <driver/rtc_io.h>

// 0x03 is default, but the address can also be 0x02, 0x01.
// Adjust the address jumpers on the underside of the product.
#define AS3935_ADDR 0x03
#define INDOOR 0x12
#define OUTDOOR 0xE
#define LIGHTNING_INT 0x08
#define DISTURBER_INT 0x04
#define NOISE_INT 0x01

#define BUTTON_PIN_BITMASK 0x1000000010

#warning "Replace with your own WiFI credentials!"

static const char *WIFI_SSID_1 = "SSID_1";
static const char *WIFI_PASSWORD_1 = "PASSWORD_1";
static const char *WIFI_SSID_2 = "SSID_2";
static const char *WIFI_PASSWORD_2 = "PASSWORD_2";
static const char *WIFI_SSID_3 = "SSID_3";
static const char *WIFI_PASSWORD_3 = "PASSWORD_3";
static const char *WIFI_SSID_4 = "SSID_4";
static const char *WIFI_PASSWORD_4 = "PASSWORD_4";

#warning "Replace with your own Adafruit IO credentials!"

static const char *MQTT_ADAFRUIT_IO_USERNAME = "USERNAME";
static const char *MQTT_ADAFRUIT_IO_KEY = "KEY";
static const char *MQTT_ADAFRUIT_IO_HOST = "io.adafruit.com";
static const char *MQTT_ADAFRUIT_IO_TOPIC = "TOPIC";
static const int MQTT_ADAFRUIT_IO_PORT = 1883;

static const uint32_t WIFI_CONNECTION_TIMEOUT = 10000;

static const int WIFI_MAX_RETRIES = 3;
static const int MQTT_MAX_RETRIES = 3;
static const int SEND_MAX_RETRIES = 3;

// Interrupt pin for lightning detection
static const int PIN_AS3935_IRQ = GPIO_NUM_15;
static const int PIN_OTA_UPDATE = GPIO_NUM_36;

static const int PIN_LIGHTNING = GPIO_NUM_23;
static const int PIN_EVENT = LED_BUILTIN;

static const uint32_t OTA_MAX_DURATION = 180000;

char chipID[22] = "";
char printBuffer[100] = "";

typedef enum { WIFI_MQTT_PUBLISH, WIFI_OTA_UPDATE } WiFiConnectionReason_enum;

typedef struct {
  uint8_t event;
  uint8_t distance;
} AS3935Event_struct;

TwoWire i2c(0);
SparkFun_AS3935 lightning(AS3935_ADDR);
WiFiClient wifiClient;
WiFiMulti wifiMulti;
PubSubClient mqttClient;

xTaskHandle readAS3935Event;
xTaskHandle connectWiFi;
xTaskHandle connectMQTT;
xTaskHandle sendData;
xTaskHandle otaUpdate;

void deviceSleep();

void xTaskReadAS3935Event(void *pvParameters);
void xTaskConnectWiFi(void *pvParameters);
void xTaskConnectMQTTT(void *pvParameters);
void xTaskSendData(void *pvParameters);
void xTaskOTAUpdate(void *pvParameters);

// This variable holds the number representing the lightning or non-lightning
// event issued by the lightning detector.
int intVal = 0;
int noise = 2;     // Value between 1-7
int disturber = 2; // Value between 1-10

void setup() {
  // When lightning is detected the interrupt pin goes HIGH.
  pinMode(PIN_LIGHTNING, OUTPUT);
  pinMode(PIN_EVENT, OUTPUT);

  Serial.begin(115200);

  uint32_t eFuseMac = ESP.getEfuseMac();
  sprintf(chipID, "ESP32-%016X", eFuseMac);
  Serial.println(chipID);

  Serial.println("AS3935 Franklin Lightning Detector");

  i2c.begin(GPIO_NUM_21, GPIO_NUM_22); // Begin Wire before lightning sensor.

  uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status();
  Serial.print("GPIO that triggered the wake up: GPIO ");
  int GPIO_triggered = (log(GPIO_reason)) / log(2);
  Serial.println(GPIO_triggered);

  if (!lightning.begin(i2c)) { // Initialize the sensor.
    Serial.println("Lightning Detector did not start up, freezing!");
    /* Sleep */
  } else
    Serial.println("Schmow-ZoW, Lightning Detector Ready!");

  // The lightning detector defaults to an indoor setting at
  // the cost of less sensitivity, if you plan on using this outdoors
  // uncomment the following line:
  lightning.setIndoorOutdoor(OUTDOOR);

  if (GPIO_triggered == PIN_OTA_UPDATE) {
    xTaskCreate(xTaskOTAUpdate, "otaUpdate", 4096, NULL, 1, &otaUpdate);
  } else if (GPIO_triggered == PIN_AS3935_IRQ) {
    xTaskCreate(xTaskReadAS3935Event, "readAS3935Event", 4096, NULL, 0,
                &readAS3935Event);
  } else {
    deviceSleep();
  }
}

void loop() {}

void deviceSleep() {
  uint64_t bitmask = (1ULL << PIN_AS3935_IRQ) | (1ULL << PIN_OTA_UPDATE);
  esp_sleep_enable_ext1_wakeup(bitmask, ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  rtc_gpio_pullup_dis((gpio_num_t)PIN_AS3935_IRQ);
  rtc_gpio_pullup_dis((gpio_num_t)PIN_OTA_UPDATE);
  rtc_gpio_pulldown_en((gpio_num_t)PIN_AS3935_IRQ);
  rtc_gpio_pulldown_en((gpio_num_t)PIN_OTA_UPDATE);
  snprintf(printBuffer, sizeof(printBuffer), "Device sleep\n");
  Serial.print(printBuffer);
  esp_deep_sleep_start();
}

void xTaskConnectWiFi(void *pvParameters) {
  WiFiConnectionReason_enum reason = *(WiFiConnectionReason_enum *)pvParameters;
  int retry = 0;
  for (;;) {
    if (retry >= WIFI_MAX_RETRIES) {
      snprintf(printBuffer, sizeof(printBuffer), "WiFi connection failed!\n");
      Serial.print(printBuffer);
      deviceSleep();
      vTaskDelete(NULL);
    }
    WiFi.mode(WIFI_STA);
    wifiMulti.addAP(WIFI_SSID_1, WIFI_PASSWORD_1);
    wifiMulti.addAP(WIFI_SSID_2, WIFI_PASSWORD_2);
    wifiMulti.addAP(WIFI_SSID_3, WIFI_PASSWORD_3);
    wifiMulti.addAP(WIFI_SSID_4, WIFI_PASSWORD_4);
    if (wifiMulti.run(WIFI_CONNECTION_TIMEOUT) == WL_CONNECTED) {
      snprintf(printBuffer, sizeof(printBuffer), "Connected to WiFI AP:\t%s\n",
               WiFi.SSID().c_str());
      Serial.print(printBuffer);
      snprintf(printBuffer, sizeof(printBuffer), "IP address: %s\n",
               WiFi.localIP().toString().c_str());
      Serial.print(printBuffer);
      if (reason == WIFI_MQTT_PUBLISH) {
        vTaskResume(connectMQTT);
      } else if (reason == WIFI_OTA_UPDATE) {
        vTaskResume(otaUpdate);
      }
      vTaskDelete(NULL);
    } else {
      snprintf(printBuffer, sizeof(printBuffer), "WiFi connection  timeout\n");
      Serial.print(printBuffer);
      retry++;
      snprintf(printBuffer, sizeof(printBuffer),
               "WiFi connection retries left:\t%d\n", WIFI_MAX_RETRIES - retry);
      Serial.print(printBuffer);
    }
  }
  vTaskDelete(NULL);
}

void xTaskConnectMQTTT(void *pvParameters) {
  (void)pvParameters;
  vTaskSuspend(NULL);
  mqttClient.setClient(wifiClient);
  mqttClient.setServer(MQTT_ADAFRUIT_IO_HOST, MQTT_ADAFRUIT_IO_PORT);
  int retry = 0;
  for (;;) {
    if (retry >= MQTT_MAX_RETRIES) {
      deviceSleep();
      vTaskDelete(NULL);
    }
    if (WiFi.status() == WL_CONNECTED) {
      while (!mqttClient.connected()) {
        snprintf(printBuffer, sizeof(printBuffer), "Connecting to MQTT\n");
        Serial.print(printBuffer);
        if (mqttClient.connect(chipID, MQTT_ADAFRUIT_IO_USERNAME,
                               MQTT_ADAFRUIT_IO_KEY)) {
          snprintf(printBuffer, sizeof(printBuffer),
                   "MQTT connection success!\n");
          Serial.print(printBuffer);
          vTaskResume(sendData);
          vTaskDelete(NULL);
        } else {
          snprintf(printBuffer, sizeof(printBuffer),
                   "MQTT connection failed!\n");
          Serial.print(printBuffer);
          snprintf(printBuffer, sizeof(printBuffer), "Reason\t%d\n",
                   mqttClient.state());
          Serial.print(printBuffer);
          retry++;
          snprintf(printBuffer, sizeof(printBuffer),
                   "MQTT connection retries left:\t%d\n",
                   MQTT_MAX_RETRIES - retry);
          Serial.print(printBuffer);
          vTaskDelay(pdMS_TO_TICKS(5000));
        }
      }
    } else {
      retry++;
      snprintf(printBuffer, sizeof(printBuffer),
               "MQTT connection retries left:\t%d\n", MQTT_MAX_RETRIES - retry);
      Serial.print(printBuffer);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  vTaskDelete(NULL);
}

void xTaskReadAS3935Event(void *pvParameters) {
  (void)pvParameters;
  AS3935Event_struct sensorData;
  digitalWrite(PIN_EVENT, HIGH);
  // Hardware has alerted us to an event, now we read the interrupt register
  // to see exactly what it is.
  intVal = lightning.readInterruptReg();
  if (intVal == NOISE_INT) {

    Serial.println("Noise.");
    // Too much noise? Uncomment the code below, a higher number means better
    // noise rejection.
    // lightning.setNoiseLevel(setNoiseLevel);
    sensorData.event = NOISE_INT;
    sensorData.distance = 0;
  } else if (intVal == DISTURBER_INT) {
    Serial.println("Disturber.");
    // Too many disturbers? Uncomment the code below, a higher number means
    // better disturber rejection.
    // lightning.watchdogThreshold(threshVal);
    sensorData.event = DISTURBER_INT;
    sensorData.distance = 0;
  } else if (intVal == LIGHTNING_INT) {
    digitalWrite(PIN_LIGHTNING, HIGH);
    Serial.println("Lightning Strike Detected!");
    // Lightning! Now how far away is it? Distance estimation takes into
    // account any previously seen events in the last 15 seconds.
    uint8_t distance = lightning.distanceToStorm();
    Serial.print("Approximately: ");
    Serial.print(distance);
    Serial.println("km away!");
    sensorData.event = LIGHTNING_INT;
    sensorData.distance = distance;
  }
  WiFiConnectionReason_enum reason = WIFI_MQTT_PUBLISH;
  xTaskCreate(xTaskConnectWiFi, "connectWiFi", 4096, &reason, 1, &connectWiFi);
  xTaskCreate(xTaskConnectMQTTT, "connectMQTT", 4096, NULL, 1, &connectMQTT);
  xTaskCreate(xTaskSendData, "sendData", 4096, &sensorData, 1, &sendData);
  vTaskDelay(pdMS_TO_TICKS(500));
  digitalWrite(PIN_LIGHTNING, LOW);
  digitalWrite(PIN_EVENT, LOW);
  vTaskDelete(NULL);
}

void xTaskSendData(void *pvParameters) {
  AS3935Event_struct sensorData = *(AS3935Event_struct *)pvParameters;
  vTaskSuspend(NULL);
  int retry = 0;
  for (;;) {
    if (retry >= SEND_MAX_RETRIES) {
      deviceSleep();
      vTaskDelete(NULL);
    }
    if ((WiFi.status() == WL_CONNECTED) && (mqttClient.connected())) {
      size_t capacity = JSON_OBJECT_SIZE(2);
      DynamicJsonDocument doc(capacity);
      switch (sensorData.event) {
      case NOISE_INT:
        doc["event"] = "NOISE";
        break;
      case DISTURBER_INT:
        doc["event"] = "DISTURBER";
        break;
      case LIGHTNING_INT:
        doc["event"] = "LIGHTNING";
        break;
      default:
        doc["event"] = "ERROR";
        break;
      }
      doc["distance"] = sensorData.distance;
      char msg[256];
      size_t n = serializeJson(doc, msg);
      snprintf(printBuffer, sizeof(printBuffer), "Sending data via MQTT\n");
      Serial.print(printBuffer);
      if (mqttClient.publish(MQTT_ADAFRUIT_IO_TOPIC, msg, n)) {
        snprintf(printBuffer, sizeof(printBuffer), "Data %s sent to %s\n", msg,
                 MQTT_ADAFRUIT_IO_TOPIC);
        Serial.print(printBuffer);
        deviceSleep();
        vTaskDelete(NULL);
      }
      retry++;
      snprintf(printBuffer, sizeof(printBuffer),
               "Send data retries left:\t%d\n", MQTT_MAX_RETRIES - retry);
      Serial.print(printBuffer);
    } else {
      retry++;
      snprintf(printBuffer, sizeof(printBuffer),
               "Send data retries left:\t%d\n", MQTT_MAX_RETRIES - retry);
      Serial.print(printBuffer);
    }
  }
  vTaskDelete(NULL);
}

void xTaskOTAUpdate(void *pvParameters) {
  (void)pvParameters;
  WiFiConnectionReason_enum reason = WIFI_OTA_UPDATE;
  xTaskCreate(xTaskConnectWiFi, "connectWiFi", 4096, &reason, 1, &connectWiFi);
  vTaskSuspend(NULL);
  ArduinoOTA.setHostname(chipID);
  // String hostname = ArduinoOTA.getHostname();
  // int strLen = hostname.length() + 1;
  // char charArray[strLen];
  // hostname.toCharArray(charArray, strLen);
  snprintf(printBuffer, sizeof(printBuffer), "Hostname:\t%s\n",
           ArduinoOTA.getHostname().c_str());
  Serial.print(printBuffer);
  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount
        // SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });
  ArduinoOTA.begin();
  uint32_t otaTimer = xTaskGetTickCount();
  digitalWrite(LED_BUILTIN, HIGH);
  for (;;) {
    if ((xTaskGetTickCount() - otaTimer) > pdMS_TO_TICKS(OTA_MAX_DURATION)) {
      ArduinoOTA.end();
      digitalWrite(LED_BUILTIN, LOW);
      deviceSleep();
      vTaskDelete(NULL);
    }
    ArduinoOTA.handle();
  }
  vTaskDelete(NULL);
}