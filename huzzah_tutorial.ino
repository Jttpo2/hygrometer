#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#define ONBOARD_LED_PIN 0
#define LED_PIN 13
#define BUTTON_PIN 4

// Button state
int current = LOW;
int last = HIGH;

const char* ssid      = "SissyStrut Guest"; 
const char* password  = "Internet!";

const char* host = "wifitest.adafruit.com";

//ESP8266WiFiMulti WiFiMulti;

void setup() {
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

//  Serial.begin(115200);
//  delay(100);
//
//  //  WiFiMulti.addAP("SissyStrut Guest", "Internet!");
//
//  Serial.println();
//  Serial.print("Connecting to: "); 
//  Serial.println(ssid);
//  
//  WiFi.begin(ssid, password);
//
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print("."); 
//  }
//
//  Serial.println();
//  Serial.println("Wifi connected");
//  Serial.println("IP: ");
//  Serial.println(WiFi.localIP());
  
}

int value = 0;

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    current = HIGH; 
  } else {
    current = LOW;
  }

  if (current == last) {
    return;
  }

  digitalWrite(LED_PIN, current);

  last = current;
  
//  delay(5000);
//  value++;
//
//  Serial.print("Connecting to : ");
//  Serial.println(host);
//
//  WiFiClient client;
//  const int httpPort = 80;
//  if (!client.connect(host, httpPort)) {
//    Serial.println("Connection failed");
//    return;
//  }
//
//  String url = "/testwifi/index.html";
//  Serial.print("Requesting URL: ");
//  Serial.println(url);
//  
//  // Send request to server
//  client.print(String("GET ") + url + " HTTP/1.1\r\n" + 
//  "Host: " + host + "\r\n" + 
//  "Connection: close\r\n\r\n");
//  
//  delay(500);
//  
//  // Read all of reply from server and print
//  while (client.available()) {
//    String line = client.readStringUntil('\r');
//    Serial.print(line);
//    }
//    
//    Serial.println();
//    Serial.println("Closing connection");
//
//    delay(5000);
}

void blinkLED() {
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);  
}

