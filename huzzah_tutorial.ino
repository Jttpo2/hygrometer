#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#define LED_PIN 0

const char* ssid      = "SissyStrut Guest"; 
const char* password  = "Internet!";

const char* host = "wifitest.adafruit.com";

//ESP8266WiFiMulti WiFiMulti;

void setup() {
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);
  delay(100);

  //  WiFiMulti.addAP("SissyStrut Guest", "Internet!");

  Serial.println();
  Serial.print("Connecting to: "); 
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("."); 
  }

  Serial.println();
  Serial.println("Wifi connected");
  Serial.println("IP: ");
  Serial.println(WiFi.localIP());
  
}

int value = 0;

void loop() {
  delay(5000);
  value++;

  Serial.print("Connecting to : ");
  Serial.println(host);

  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("Connection failed");
    return;
  }

  String url = "/testwifi/index.html";
  Serial.print("Requesting URL: ");
  Serial.println(url);
  
  // Send request to server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" + 
  "Host: " + host + "\r\n" + 
  "Connection: close\r\n\r\n");
  
  delay(500);
  
  // Read all of reply from server and print
  while (client.available()) {
    String line = client.readStringUntil('\r');
    Serial.print(line);
    }
    
    Serial.println();
    Serial.println("Closing connection");

    delay(5000);
}

void blinkLED() {
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);  
}

