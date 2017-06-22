#define IO_USERNAME   "Jttpo2"
#define IO_KEY        "269663b829ae4fa8b3c1f8354cff7cd5"

#define WIFI_SSID     "SissyStrut Guest"
#define WIFI_PASS     "Internet!"

#include "AdafruitIO_WiFi.h"
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

#include <ESP8266WiFi.h>
#include <AdafruitIO.h>
#include <Adafruit_MQTT.h>
#include <ArduinoHttpClient.h>

#define ONBOARD_LED_PIN 0
#define LED_PIN 13
#define BUTTON_PIN 4

// Button state
int current = LOW;
int last = HIGH;

const char* host = "wifitest.adafruit.com";

// Adafruit command feed
AdafruitIO_Feed *command = io.feed("command");

void setup() {
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(50);

  Serial.print("Connecting to Adafruit IO");
  io.connect();

  // Message handler receives messages from adafruit.io
  command->onMessage(handleMessage);

  // Wait for connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // Connected
  
  Serial.println();
  Serial.println(io.statusText());
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Required at the top to keep talking to io.adafruit.com
  io.run();
  
  // Read button
  if (digitalRead(BUTTON_PIN) == LOW) {
    current = HIGH; 
  } else {
    current = LOW;
  }

  if (current == last) {
    return;
  }

  // Save button state to command feed on adafruit io
  Serial.print("Sending button -> ");
  Serial.println(current);
  command->save(current);

  // Show state with LED
  digitalWrite(LED_PIN, current);

  last = current;
}

void handleMessage(AdafruitIO_Data *data) {
  int command = data->toInt();

  if (command == 1) {
    Serial.print("received <- ");
    Serial.println(command);
    blinkLED();
  } else {
    Serial.print("received <- ");
    Serial.println(command);
  }
}

void blinkPin(int pin) {
  digitalWrite(pin, HIGH);
  delay(500);
  digitalWrite(pin, LOW);  
}

void blinkOnboardLED() {
  blinkPin(ONBOARD_LED_PIN);
}

void blinkLED() {
  blinkPin(LED_PIN);
}

