#include <DHT.h>

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

#include <DHT.h>

#define ONBOARD_LED_PIN 0
#define LED_PIN 13
#define BUTTON_PIN 4

#define HUMIDITY_SENSOR_PIN 12
#define DHTTYPE DHT22
DHT dht(HUMIDITY_SENSOR_PIN, DHTTYPE);

// Humidity variables
float humidity;
float temperature; 

// Button state
int current = LOW;
int last = HIGH;

const char* host = "wifitest.adafruit.com";

// Adafruit feeds
AdafruitIO_Feed *humidityCommand = io.feed("humidity");
AdafruitIO_Feed *temperatureCommand = io.feed("temperature");
AdafruitIO_Feed *buttonCommand = io.feed("button");

void setup() {
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(HUMIDITY_SENSOR_PIN, INPUT);

  Serial.begin(115200);
  delay(50);

  dht.begin();

  Serial.print("Connecting to Adafruit IO");
  io.connect();

  // Message handler receives messages from adafruit.io
  humidityCommand->onMessage(handleMessage);
  temperatureCommand->onMessage(handleMessage);
  buttonCommand->onMessage(handleMessage);

  // Wait for connection
  while (io.status() < AIO_CONNECTED) {
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

  runHumiditySensor();
  printHumidityReadings();
  sendReadingsToCloud();

  handleButtonSensing();
  sendButtonStatusToCloud();

  delay(5000);
}

void handleMessage(AdafruitIO_Data *data) {
  int command = data->toInt();
//  if (command == 1) {
//    Serial.print("received <- ");
//    Serial.println(command);
//    blinkLED();
//  } else {
    Serial.print("received <- ");
    Serial.println(command);

//    Serial.println(data);
//  }
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

void runHumiditySensor() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
}

void printHumidityReadings() {
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%, Temp: ");
  Serial.print(temperature);
  Serial.print(humidity);
  Serial.println(" C");
}

void sendReadingsToCloud() {
  Serial.print("Sending humidity -> ");
  Serial.println(humidity);
  humidityCommand->save(humidity);

  Serial.print("Sending temperature -> ");
  Serial.println(temperature);
  temperatureCommand->save(temperature); 
}

void sendButtonStatusToCloud() {
   // Save button state to command feed on adafruit io
  Serial.print("Sending button -> ");
  Serial.println(current);
  buttonCommand->save(current);
}

void handleButtonSensing() {
  // Read button
  if (digitalRead(BUTTON_PIN) == LOW) {
    current = HIGH;
  } else {
    current = LOW;
  }

  if (current == last) {
    return;
  }

  // Show state with LED
  digitalWrite(LED_PIN, current);

  last = current;

}

