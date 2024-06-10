#include <SoftwareSerial.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspUdp.h>
#include <PubSubClient.h>
#include <IRremote.h>

#define IR_SEND_PIN 9  // IR 센서 핀 번호 정의

IPAddress server(192, 168, 0, 11);  // MQTT 서버 주소
char ssid[] = "iptime2.4G";         // 와이파이 SSID
char pass[] = "A1B2C312";           // 와이파이 비밀번호
int status = WL_IDLE_STATUS;        // 와이파이 상태

WiFiEspClient esp8266Client;
PubSubClient client(esp8266Client);
SoftwareSerial esp8266(2, 3);  // RX, TX to ESP-01

IRsend irsend;  // IR 센서 객체 생성

// 에어컨 On/Off에 해당하는 IR 코드 정의
unsigned long AC_ON_CODE = 0x1234;   // 에어컨 On 코드 (예시)
unsigned long AC_OFF_CODE = 0x5678;  // 에어컨 Off 코드 (예시)

bool controlEnabled = false;  // control 토픽 값

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

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
    delay(5000);
  }

  // you're connected now, so print out the data
  Serial.println("You're connected to the network");

  // connect to MQTT server
  client.setServer(server, 1883);
  client.setCallback(callback);
}

void loop() {
  // reconnect to MQTT server if needed
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