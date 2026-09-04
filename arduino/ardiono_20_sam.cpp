#include <WiFiS3.h>
#include <WiFiUdp.h>

const char* ssid     = "RPi5-Hotspot";
const char* password = "12345678";

IPAddress receiverIP(192,168,50,1);   // IP مال الرازبري/اللابتوب
const uint16_t receiverPort = 5005;

WiFiUDP udp;

const int CH1_PIN = A0;
const int CH2_PIN = A1;

const uint16_t BATCH = 20;            // 20 samples per packet
const uint32_t SAMPLE_PERIOD_US = 1000; // ~1kHz

uint32_t lastSample = 0;
uint16_t idx = 0;
uint32_t packet_id = 0;

int s1[BATCH];
int s2[BATCH];

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println("\n[OK] WiFi connected!");
  Serial.print("UNO R4 IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(0); // local ephemeral port
}

void sendPacket() {
  udp.beginPacket(receiverIP, receiverPort);

  // packet format: id;v1,v2;v1,v2;...(20)
  udp.print(packet_id);
  for (uint16_t i = 0; i < BATCH; i++) {
    udp.print(';');
    udp.print(s1[i]);
    udp.print(',');
    udp.print(s2[i]);
  }
  udp.print('\n');

  udp.endPacket();

  packet_id++;
}

void loop() {
  uint32_t now = micros();

  if (now - lastSample >= SAMPLE_PERIOD_US) {
    lastSample = now;

    s1[idx] = analogRead(CH1_PIN);
    s2[idx] = analogRead(CH2_PIN);
    idx++;

    if (idx >= BATCH) {
      sendPacket();
      idx = 0;
    }
  }
}
