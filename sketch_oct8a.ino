/***********************************************************************
  Adafruit MQTT Library ESP32 Adafruit IO SSL/TLS example
 **********************************************************************/
#include <WiFi.h>
#include "WiFiClientSecure.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <DHT11.h>

#define pinDATA 14   // Pin du lieu cho DHT11
#define LED_PIN1 23  // Pin dieu khien LED1
#define LED_PIN2 21  // Pin dieu khien LED2 
#define MOTOR_PIN 25 // Pin dieu khien Motor

/************************* WiFi Access Point *********************************/
#define WLAN_SSID "Le Hieu_2.4Hz"
#define WLAN_PASS "12345679"

/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  8883  // Using port 8883 for MQTTS

// Adafruit IO Account Configuration
#define AIO_USERNAME "___"
#define AIO_KEY      "___"

/************ Global State (you don't need to change this!) ******************/
WiFiClientSecure client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// io.adafruit.com root CA
const char* adafruitio_root_ca = \
      "-----BEGIN CERTIFICATE-----\n"
      "MIIEjTCCA3WgAwIBAgIQDQd4KhM/xvmlcpbhMf/ReTANBgkqhkiG9w0BAQsFADBh\n"
      "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
      "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"
      "MjAeFw0xNzExMDIxMjIzMzdaFw0yNzExMDIxMjIzMzdaMGAxCzAJBgNVBAYTAlVT\n"
      "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
      "b20xHzAdBgNVBAMTFkdlb1RydXN0IFRMUyBSU0EgQ0EgRzEwggEiMA0GCSqGSIb3\n"
      "DQEBAQUAA4IBDwAwggEKAoIBAQC+F+jsvikKy/65LWEx/TMkCDIuWegh1Ngwvm4Q\n"
      "yISgP7oU5d79eoySG3vOhC3w/3jEMuipoH1fBtp7m0tTpsYbAhch4XA7rfuD6whU\n"
      "gajeErLVxoiWMPkC/DnUvbgi74BJmdBiuGHQSd7LwsuXpTEGG9fYXcbTVN5SATYq\n"
      "DfbexbYxTMwVJWoVb6lrBEgM3gBBqiiAiy800xu1Nq07JdCIQkBsNpFtZbIZhsDS\n"
      "fzlGWP4wEmBQ3O67c+ZXkFr2DcrXBEtHam80Gp2SNhou2U5U7UesDL/xgLK6/0d7\n"
      "6TnEVMSUVJkZ8VeZr+IUIlvoLrtjLbqugb0T3OYXW+CQU0kBAgMBAAGjggFAMIIB\n"
      "PDAdBgNVHQ4EFgQUlE/UXYvkpOKmgP792PkA76O+AlcwHwYDVR0jBBgwFoAUTiJU\n"
      "IBiV5uNu5g/6+rkS7QYXjzkwDgYDVR0PAQH/BAQDAgGGMB0GA1UdJQQWMBQGCCsG\n"
      "AQUFBwMBBggrBgEFBQcDAjASBgNVHRMBAf8ECDAGAQH/AgEAMDQGCCsGAQUFBwEB\n"
      "BCgwJjAkBggrBgEFBQcwAYYYaHR0cDovL29jc3AuZGlnaWNlcnQuY29tMEIGA1Ud\n"
      "HwQ7MDkwN6A1oDOGMWh0dHA6Ly9jcmwzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydEds\n"
      "b2JhbFJvb3RHMi5jcmwwPQYDVR0gBDYwNDAyBgRVHSAAMCowKAYIKwYBBQUHAgEW\n"
      "HGh0dHBzOi8vd3d3LmRpZ2ljZXJ0LmNvbS9DUFMwDQYJKoZIhvcNAQELBQADggEB\n"
      "AIIcBDqC6cWpyGUSXAjjAcYwsK4iiGF7KweG97i1RJz1kwZhRoo6orU1JtBYnjzB\n"
      "c4+/sXmnHJk3mlPyL1xuIAt9sMeC7+vreRIF5wFBC0MCN5sbHwhNN1JzKbifNeP5\n"
      "ozpZdQFmkCo+neBiKR6HqIA+LMTMCMMuv2khGGuPHmtDze4GmEGZtYLyF8EQpa5Y\n"
      "jPuV6k2Cr/N3XxFpT3hRpt/3usU/Zb9wfKPtWpoznZ4/44c1p9rzFcZYrWkj3A+7\n"
      "TNBJE0GmP2fhXhP1D/XVfIW/h0yCJGEiV9Glm/uGOa3DXHlmbAcxSyCRraG+ZBkA\n"
      "7h4SeM6Y8l/7MBRpPCz6l8Y=\n"
      "-----END CERTIFICATE-----\n";

/****************************** Feeds ***************************************/
// Setup feeds for temperature, humidity, and LED/Motor control
Adafruit_MQTT_Publish Temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish Humi = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");

Adafruit_MQTT_Publish Status_LED1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/status_LED1");
Adafruit_MQTT_Subscribe onoff_LED1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Led1");

Adafruit_MQTT_Publish Status_LED2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/status_LED2");
Adafruit_MQTT_Subscribe onoff_LED2 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Led2");

Adafruit_MQTT_Publish Status_Motor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/status_Motor");
Adafruit_MQTT_Subscribe onoff_Motor = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Motor");

/*************************** Sketch Code ************************************/
DHT11 dht11(pinDATA);

// Biến để theo dõi thời gian gửi dữ liệu nhiệt độ và độ ẩm mỗi 10 giây
uint32_t lastPublishTime = 0;
const uint32_t publishInterval = 10000;  // 10 giây

// Biến trạng thái của LED và Motor
uint32_t status_LED1 = 0;
uint32_t status_LED2 = 0;
uint32_t status_Motor = 0;

void setup() {
  Serial.begin(115200);
  delay(10);
  
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  Serial.println(F("Adafruit IO MQTTS (SSL/TLS) Example"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  delay(1000);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  delay(2000);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Set Adafruit IO's root CA
  client.setCACert(adafruitio_root_ca);

  // Subscribe to the feeds for the LEDs and Motor
  mqtt.subscribe(&onoff_LED1);
  mqtt.subscribe(&onoff_LED2);
  mqtt.subscribe(&onoff_Motor);

  // init value
  digitalWrite(LED_PIN1, HIGH);
  digitalWrite(LED_PIN2, HIGH);
}

void loop() {
  // Ensure the connection to the MQTT server is alive
  MQTT_connect();

  // Kiểm tra thời gian trôi qua để gửi dữ liệu mỗi 10 giây
  uint32_t currentMillis = millis();
  if (currentMillis - lastPublishTime >= publishInterval) {
    lastPublishTime = currentMillis;

    // Đọc dữ liệu từ cảm biến DHT11
    int temperature = 0;
    int humidity = 0;
    int result = dht11.readTemperatureHumidity(temperature, humidity);

    if (result == 0) {

      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print(" °C\tHumidity: ");
      Serial.print(humidity);
      Serial.println(" %");

      uint32_t temp_v = temperature;
      uint32_t humi_v = humidity;
      // Gửi dữ liệu nhiệt độ và độ ẩm lên Adafruit IO
      if (!Temp.publish(temp_v)) {
        Serial.println(F("Failed to publish temperature"));
      } else {
        Serial.println(F("Temperature published!"));
      }

      if (!Humi.publish(humi_v)) {
        Serial.println(F("Failed to publish humidity"));
      } else {
        Serial.println(F("Humidity published!"));
      }

    } else {
      Serial.println(DHT11::getErrorString(result));
    }
  }

  // Kiểm tra các gói tin subscribe đến từ Adafruit IO
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    // Xử lý điều khiển LED1
    if (subscription == &onoff_LED1) {
      Serial.print(F("LED1: "));
      Serial.println((char *)onoff_LED1.lastread);

      if (strcmp((char *)onoff_LED1.lastread, "1") == 0) {
        status_LED1 = 1;  // ON status
        digitalWrite(LED_PIN1, LOW); // Bật LED1
        if (!Status_LED1.publish(status_LED1)) {
          Serial.println(F("Failed to publish status LED1"));
        } else {
          Serial.println(F("LED1 turned ON"));
        }
      } else if (strcmp((char *)onoff_LED1.lastread, "0") == 0) {
        status_LED1 = 0;  // OFF status
        digitalWrite(LED_PIN1, HIGH); // Tắt LED1
        if (!Status_LED1.publish(status_LED1)) {
          Serial.println(F("Failed to publish status LED1"));
        } else {
          Serial.println(F("LED1 turned OFF"));
        }
      }
    }

    // Xử lý điều khiển LED2
    if (subscription == &onoff_LED2) {
      Serial.print(F("LED2: "));
      Serial.println((char *)onoff_LED2.lastread);

      if (strcmp((char *)onoff_LED2.lastread, "1") == 0) {
        status_LED2 = 1;  // ON status
        digitalWrite(LED_PIN2, LOW); // Bật LED2
        if (!Status_LED2.publish(status_LED2)) {
          Serial.println(F("Failed to publish status LED2"));
        } else {
          Serial.println(F("LED2 turned ON"));
        }
      } else if (strcmp((char *)onoff_LED2.lastread, "0") == 0) {
        status_LED2 = 0;  // OFF status
        digitalWrite(LED_PIN2, HIGH); // Tắt LED2
        if (!Status_LED2.publish(status_LED2)) {
          Serial.println(F("Failed to publish status LED2"));
        } else {
          Serial.println(F("LED2 turned OFF"));
        }
      }
    }

    // Xử lý điều khiển Motor
    if (subscription == &onoff_Motor) {
      Serial.print(F("Motor: "));
      Serial.println((char *)onoff_Motor.lastread);

      if (strcmp((char *)onoff_Motor.lastread, "1") == 0) {
        status_Motor = 1;  // ON status
        digitalWrite(MOTOR_PIN, HIGH); // Bật Motor
        if (!Status_Motor.publish(status_Motor)) {
          Serial.println(F("Failed to publish status Motor"));
        } else {
          Serial.println(F("Motor turned ON"));
        }
      } else if (strcmp((char *)onoff_Motor.lastread, "0") == 0) {
        status_Motor = 0;  // OFF status
        digitalWrite(MOTOR_PIN, LOW); // Tắt Motor
        if (!Status_Motor.publish(status_Motor)) {
          Serial.println(F("Failed to publish status Motor"));
        } else {
          Serial.println(F("Motor turned OFF"));
        }
      }
    }
  }

  // Ping server để giữ kết nối MQTT
  if (!mqtt.ping()) {
    mqtt.disconnect();
  }
}

// Function to connect to the MQTT server
void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { 
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  
    retries--;
    if (retries == 0) {
      while (1);
    }
  }

  Serial.println("MQTT Connected!");
}
