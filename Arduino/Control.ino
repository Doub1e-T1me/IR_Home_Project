#include <SoftwareSerial.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspUdp.h>
#include <PubSubClient.h>
#include <IRremote.h>

#define IR_SEND_PIN 9  // IR 센서 핀 번호 정의

IPAddress server(192, 168, 0, 11);
char ssid[] = "iptime2.4G";
char pass[] = "";
int status = WL_IDLE_STATUS;

WiFiEspClient esp8266Client;
PubSubClient client(esp8266Client);
SoftwareSerial esp8266(2, 3);

IRsend irsend;

// 에어컨 On/Off에 해당하는 IR 코드 정의
unsigned long AC_ON_CODE = 0x1234;   // 에어컨 On 코드
unsigned long AC_OFF_CODE = 0x5678;  // 에어컨 Off 코드

bool controlEnabled = false;

void callback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, "control") == 0) {
    controlEnabled = (payload[0] == '1');
  } else if (strcmp(topic, "control/ac") == 0 && controlEnabled) {
    if (payload[0] == '1') {
      irsend.sendNEC(AC_ON_CODE, 32);  // 에어컨 On IR 신호 전송
      Serial.println("AC turned on");
    } else if (payload[0] == '0') {
      irsend.sendNEC(AC_OFF_CODE, 32);  // 에어컨 Off IR 신호 전송
      Serial.println("AC turned off");
    }
  }
}

void setup() {
  Serial.begin(9600);
  esp8266.begin(9600);
  WiFi.init(&esp8266);

  irsend.begin();

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
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("arduinoClient2")) {
      Serial.println("connected");
      client.subscribe("control");
      client.subscribe("control/ac");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }

  client.loop();
  delay(1000);
}
