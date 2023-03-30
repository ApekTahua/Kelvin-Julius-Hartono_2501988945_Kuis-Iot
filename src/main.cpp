// MQTT 2 sensors and 3 LEDS
// Publishing to broker for temperature every 5 seconds, humidity every 6 seconds, and light every 4 seconds
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <BH1750.h>

#define LED_1 15 // Merah
#define LED_2 2  // Kuning
#define LED_3 14 // Hijau

// define the pins for the DHT11 sensor

#define DHTPIN 16
#define DHTTYPE DHT11

// define the pins for BH1750 sensor

#define SDA_PIN 21
#define SCL_PIN 22

#define WIFI_SSID "ROG"
#define WIFI_PASSWORD "12345678"

#define MQTT_BROKER "broker.emqx.io"
#define MQTT_TOPIC_PUBLISH "esp32_kelvin/data"
#define MQTT_TOPIC_SUBSCRIBE "esp32_kelvin/sensor"

DHT dht(DHTPIN, DHTTYPE);

Ticker timerPublish;
char g_szDeviceId[30];
Ticker timerMqtt;

float temperature = 0;
float humidity = 0;
float lux = 0;

WiFiClient espClient;
PubSubClient mqtt(espClient);

BH1750 lightMeter(0x23);

long lastMsg = 0;
char msg[50];
int value = 0;

unsigned long previousTemperatureMillis = 0;
unsigned long previousHumidityMillis = 0;
unsigned long previousLuxMillis = 0;

const int TEMPERATURE_INTERVAL = 5000; // interval in milliseconds
const int HUMIDITY_INTERVAL = 6000;    // interval in milliseconds
const int LUX_INTERVAL = 4000;         // interval in milliseconds

// void WifiConnect();
// void onPublish(char *topic, byte *payload, unsigned int length);
// void onSubscribe(int packetId, int qos);
// void Cek_kondisi();
// void callback(char *topic, byte *payload, unsigned int length);
// void reconnect();

void WifiConnect()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
}

void onPublish(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }

  Serial.println();
}

void Cek_kondisi()
{
  // Read data from DHT11 sensor
  dht.begin();

  // Read data from BH1750 sensor
  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE, 0x23, &Wire);

  // Read data from DHT11 sensor
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  // Read data from BH1750 sensor
  lux = lightMeter.readLightLevel();

  // Temperature, Humidity, and Light Sensor
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  lux = lightMeter.readLightLevel();

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("C, Humidity: ");
  Serial.print(humidity);
  Serial.print("%, Light: ");
  Serial.print(lux);
  Serial.println("lx");

  // If lux is above 400, gives warnning to serial monitor
  if (lux > 400)
  {
    Serial.println("lux: " + String(lux) + "lx");
    Serial.println("WARNING: Light is too bright!");
  }
  else
  {
    Serial.println("lux: " + String(lux) + "lx");
    Serial.println("Light is normal");
  }

  // If temperature is above 28C, gives warnning to serial monitor
  if (isnan(humidity) || isnan(temperature))
  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  if (temperature > 28 && humidity > 80)
  {
    digitalWrite(LED_1, HIGH);
    Serial.println("LED_1 ON");
  }
  else
  {
    digitalWrite(LED_1, LOW);
    Serial.println("LED_1 OFF");
  }
  if (temperature > 28 && humidity > 60 && humidity < 80)
  {
    digitalWrite(LED_2, HIGH);
    Serial.println("LED_2 ON");
  }
  else
  {
    digitalWrite(LED_2, LOW);
    Serial.println("LED_2 OFF");
  }
  if (temperature < 28 && humidity < 60)
  {
    digitalWrite(LED_3, HIGH);
    Serial.println("LED_3 ON");
  }
  else
  {
    digitalWrite(LED_3, LOW);
    Serial.println("LED_3 OFF");
  }
}

void callback(char *topic, byte *payload, unsigned int length)

{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }

  Serial.println();
}

void reconnect()

{
  while (!mqtt.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (mqtt.connect(g_szDeviceId))
    {
      Serial.println("Connected to Apek");
      mqtt.subscribe(MQTT_TOPIC_SUBSCRIBE);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

void setup()

{
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE, 0x23, &Wire))
  {
    Serial.println("Error initialising BH1750");
  }

  WifiConnect();

  mqtt.setServer(MQTT_BROKER, 1883);
  mqtt.setCallback(callback);

  while (!mqtt.connected())
  {
    String clientId = "ESP32_Kelvin";
    if (mqtt.connect(clientId.c_str()))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed with state ");
      Serial.print(mqtt.state());
      delay(2000);
    }
  }

  // Publish and subscribe
  mqtt.publish(MQTT_TOPIC_PUBLISH, "Hello from ESP32_Kelvin");
  mqtt.subscribe(MQTT_TOPIC_SUBSCRIBE);

  dht.begin();

  timerPublish.attach_ms(3000, Cek_kondisi);
  timerMqtt.attach_ms(1000, reconnect);

  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);

  sprintf(g_szDeviceId, "ESP32_%s", WiFi.macAddress().c_str());
  Serial.println(g_szDeviceId);
  Serial.println("Setup done");
}

void loop()
{
  Cek_kondisi();

  if (!mqtt.connected())
  {
    reconnect();
  }

  // // Publishing data to MQTT broker
  // mqtt.publish(MQTT_TOPIC_PUBLISH, String(temperature).c_str());
  // mqtt.publish(MQTT_TOPIC_PUBLISH, String(humidity).c_str());
  // mqtt.publish(MQTT_TOPIC_PUBLISH, String(lux).c_str());

  unsigned long currentMillis = millis();

  if (currentMillis - previousTemperatureMillis >= TEMPERATURE_INTERVAL)
  {
    mqtt.publish(MQTT_TOPIC_PUBLISH, String(temperature).c_str());
    previousTemperatureMillis = currentMillis;
  }

  if (currentMillis - previousHumidityMillis >= HUMIDITY_INTERVAL)
  {
    mqtt.publish(MQTT_TOPIC_PUBLISH, String(humidity).c_str());
    previousHumidityMillis = currentMillis;
  }

  if (currentMillis - previousLuxMillis >= LUX_INTERVAL)
  {
    mqtt.publish(MQTT_TOPIC_PUBLISH, String(lux).c_str());
    previousLuxMillis = currentMillis;
  }

  mqtt.loop();

  delay(1000);
}
