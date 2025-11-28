#include <SoftwareSerial.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspUdp.h>
#include <PubSubClient.h>
#include <stdlib.h>
#include <Wire.h>
#include <DHT.h>

#define DHTPIN 12
#define DHTTYPE DHT11
#define DUST_SENSOR_PIN 13 // PPD42NS 디지털 핀

IPAddress server(192,168,0,11);
char ssid[] = "iptime2.4G";
char pass[] = "";
int status = WL_IDLE_STATUS;

WiFiEspClient esp8266Client;
PubSubClient client(esp8266Client);
SoftwareSerial esp8266(2, 3);
DHT dht(DHTPIN, DHTTYPE);

unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 5000; // 5초 간격
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;
float ugm3 = 0;

void setup() {
  Serial.begin(9600);
  esp8266.begin(9600);
  WiFi.init(&esp8266);
  dht.begin();
  pinMode(DUST_SENSOR_PIN, INPUT);
  starttime = millis();

  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true);
  }

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(5000);
  }

  Serial.println("You're connected to the network");

  client.setServer(server, 1883);
}

void loop() {
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("arduinoClient1")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }

  duration = pulseIn(DUST_SENSOR_PIN, LOW);
  lowpulseoccupancy += duration;

  unsigned long currentMillis = millis();

  if ((currentMillis - starttime) > sampletime_ms) {
    ratio = lowpulseoccupancy / (sampletime_ms * 10.0);
    concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62;
    ugm3 = concentration * 100 / 13000;

    int temperature = dht.readTemperature();
    int humidity = dht.readHumidity();

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Error reading from DHT sensor");
    } else {
      String tempPayload = String(temperature);
      client.publish("sensor/temp", tempPayload.c_str());

      String humidPayload = String(humidity);
      client.publish("sensor/humid", humidPayload.c_str());

      String dustPayload = String(ugm3);
      client.publish("sensor/dust", dustPayload.c_str());

      Serial.println("Published data to MQTT:");
      Serial.println("Temperature: " + tempPayload);
      Serial.println("Humidity: " + humidPayload);
      Serial.println("Dust: " + dustPayload);
    }

    lowpulseoccupancy = 0;
    starttime = millis();
  }

  client.loop();
  delay(1000);
}
