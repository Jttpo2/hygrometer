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

#include <EEPROM.h>

#define BATTERY_INTERVAL 10e6 //
#define SLEEP_LENGTH 10e6

#include <DHT.h>

#define HUMIDITY_SENSOR_PIN 12
#define DHTTYPE DHT22
DHT dht(HUMIDITY_SENSOR_PIN, DHTTYPE);

// Humidity variables
float humidity;
float temperature; 

#define ONBOARD_LED_PIN 0
#define LED_PIN 13
#define BUTTON_PIN 4

// Button state
int current = LOW;
int last = HIGH;

// Adafruit feeds
AdafruitIO_Feed *humidityCommand = io.feed("humidity");
AdafruitIO_Feed *temperatureCommand = io.feed("temperature");
AdafruitIO_Feed *buttonCommand = io.feed("button");

// Sketch restarts after sleep, so loop() never runs
void setup() {
  setupSerial();
  setupPins();
  setupHumiditySensor();
  connectToAdafruitIO();

  runHumiditySensor();
  printHumidityReadings();
//  sendReadingsToCloud();

//  handleBatterySensing();

//  handleButtonSensing();
//  sendButtonStatusToCloud();


//  delay(5*1000);
//  hibernate();
}

// Sketch restarts after sleep, so loop() never runs
void loop() {
  // Required at the top to keep talking to io.adafruit.com
//  io.run();
//
//  runHumiditySensor();
//  printHumidityReadings();
//  sendReadingsToCloud();
//
//  handleButtonSensing();
//  sendButtonStatusToCloud();

//  delay(5000);
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

void setupPins() {
  Serial.print("Setting up pins... ");
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(HUMIDITY_SENSOR_PIN, INPUT);
  Serial.println("Done.");
}

void setupSerial() {
  Serial.begin(115200);
  while (!Serial) { }
  delay(100);
  Serial.println("Serial connection initiated");
}

void setupHumiditySensor() {
  Serial.print("Initializing DHT sensor... ");
  dht.begin();
  Serial.println("Done.");
}

void connectToAdafruitIO() {
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
   Serial.println(connectionStatus(io.status()));
  }

  // Connected
  Serial.println();
  Serial.println(io.statusText());
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());

  io.run();
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
  
//  humidityCommand->save(80);
//  temperatureCommand->save(100); 
  
  Serial.print("Sending humidity -> ");
  Serial.println(humidity);
  humidityCommand->save(humidity);

  Serial.print("Sending temperature -> ");
  Serial.println(temperature);
  temperatureCommand->save(temperature); 
}

void handleBatterySensing() {
   EEPROM.begin(512);

  // Get current count position from eeprom
  byte battery_count = EEPROM.read(0);

  // Use eeprom to track battery count between resets
  if (battery_count >= (BATTERY_INTERVAL / SLEEP_LENGTH)) {
    // Reset counter
    battery_count = 0;
    runBatterySensor();
    sendBatteryReadingToCloud();
  } else {
    battery_count++;
  }

  // Save current count
  EEPROM.write(0, battery_count);
  EEPROM.commit();
}

void runBatterySensor() {
  Serial.println("Reading battery level");
}

void sendBatteryReadingToCloud() {
  Serial.println("Sending battery level to cloud");
}

void sendButtonStatusToCloud() {
   // Save button state to command feed on adafruit io
  Serial.print("Sending button -> ");
  Serial.println(current);
  buttonCommand->save(current);
}

void hibernate() {
  Serial.println("Going to sleep");
  ESP.deepSleep(SLEEP_LENGTH, WAKE_RF_DISABLED); 
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

String connectionStatus ( int which )
{
    switch ( which )
    {
        case WL_CONNECTED:
            return "Connected";
            break;

        case WL_NO_SSID_AVAIL:
            return "Network not available";
            break;

        case WL_CONNECT_FAILED:
            return "Wrong password";
            break;

        case WL_IDLE_STATUS:
            return "Idle status";
            break;

        case WL_DISCONNECTED:
            return "Disconnected";
            break;

        default:
            return "Unknown";
            break;
    }
}


