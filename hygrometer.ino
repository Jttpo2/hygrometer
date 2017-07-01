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

//#define BATTERY_INTERVAL  60 * 1e6 //
#define SLEEP_LENGTH        60 * 1e6

#include <DHT.h>

#define HUMIDITY_SENSOR_PIN 12
#define DHTTYPE DHT22
DHT dht(HUMIDITY_SENSOR_PIN, DHTTYPE);

// Humidity variables
float humidity;
float temperature; 

float batteryLevel;

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
AdafruitIO_Feed *batteryCommand = io.feed("battery");

#define MAX_CONNECTION_TIME 7 * 1e3 // WiFi connection timeout in millis
boolean keepTryingToConnect = true; // For WiFi connection timeout
long connectionStartTime = 0;

// Sketch restarts after sleep, so loop() never runs
void setup() {
  setupSerial();
  setupPins();
  setupHumiditySensor();
  if (!connectToAdafruitIO()) {
    // Connection failed, sleep and try again
    hibernate();
  }

  runHumiditySensor();
  printHumidityReadings();

  handleBatterySensing();
  printBatteryLevelReading();
  
  sendReadingsToCloud();

//  handleButtonSensing();
//  sendButtonStatusToCloud();

//  delay(5*1000);
  hibernate();
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
  Serial.begin(9600);
  while (!Serial) { }
  delay(100);
  clearSerialBuffer();
  Serial.println("Serial connection initiated");
}

void clearSerialBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
  Serial.println();
}

void setupHumiditySensor() {
  Serial.print("Initializing DHT sensor... ");
  dht.begin();
  Serial.println("Done.");
}

boolean connectToAdafruitIO() {
//  Serial.setDebugOutput(true);
  
  Serial.print("Connecting to Adafruit IO");  
  io.connect();

  // Message handler receives messages from adafruit.io
  humidityCommand->onMessage(handleMessage);
  temperatureCommand->onMessage(handleMessage);
  buttonCommand->onMessage(handleMessage);
  batteryCommand->onMessage(handleMessage);

  // Wait for connection
  startConnectionTimer();
  while (io.status() < AIO_CONNECTED && keepTryingToConnect) {
    Serial.print(".");
    

//    if (io.status() == AIO_NET_CONNECT_FAILED) {
//      Serial.println("Failed to connect to WiFi. Please verify credentials: ");
//      delay(10000);
//    }

    // Debugging connection
//    WiFi.printDiag(Serial);
//    Serial.println(printConnectionStatusAdafruitIO(io.status()));

    checkConnectionTimer();
    if(keepTryingToConnect) {
      delay(500); // Don't wait if we're reached timeout
    }
  }

  if (io.status() < AIO_CONNECTED) {
     Serial.println("Did not manage to connect");
     return false;
  }

  // Connected
  Serial.println();
  Serial.println(io.statusText());
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());

  io.run();

  return true;
}

void disconnectWiFi() {
  Serial.println("Disconnecting Wifi");
  WiFi.disconnect();
  delay(100);
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

void printBatteryLevelReading() {
  Serial.print("Battery level: ");
  Serial.print(batteryLevel);
  Serial.println(" %");
}

void sendReadingsToCloud() {
  Serial.print("Sending humidity -> ");
  Serial.println(humidity);
  humidityCommand->save(humidity);

  Serial.print("Sending temperature -> ");
  Serial.println(temperature);
  temperatureCommand->save(temperature); 

  Serial.print("Sending battery level -> ");
  Serial.println(batteryLevel);
  batteryCommand->save(batteryLevel);
}

void handleBatterySensing() {
   EEPROM.begin(512);

  // Get current count position from eeprom
  byte battery_count = EEPROM.read(0);

  // Use eeprom to track battery count between resets
//  if (battery_count >= (BATTERY_INTERVAL / SLEEP_LENGTH)) {
    // Reset counter
//    battery_count = 0;
    runBatterySensor();
//  } else {
//    battery_count++;
//  }

  // Save current count
  EEPROM.write(0, battery_count);
  EEPROM.commit();
}

void runBatterySensor() {
  Serial.println("Reading battery level");
  float level = analogRead(A0);

  // Convert battery level (between 0 and 1023) to percentage of full charge
  // Voltage divider 560 and 3*560=1680 Ohms converts the lipoly battery's min from
  // 3.0V to 0.75V and it's max from 4.2V to 1.05V. Min analog read value is therefore:
  // 1023*0.75=767.25 and max: 1023*1.05=1074.15. Since this is slightly above
  // maximum the battery will read as full for a bit longer.
  Serial.print("A0 reading value: ");
  Serial.println(level);
  batteryLevel = map(level, 766.25, 1023, 0, 100); 
}

void sendButtonStatusToCloud() {
   // Save button state to command feed on adafruit io
  Serial.print("Sending button -> ");
  Serial.println(current);
  buttonCommand->save(current);
}

void hibernate() {
  // Remember to connect pins 16 and RST to let huzzah wake from deep sleep
//  disconnectWiFi();
  Serial.println("Going to sleep");
  ESP.deepSleep(SLEEP_LENGTH, WAKE_RF_DEFAULT);
  
  // Having the WAKE_RF_DISABLED leaves WiFi off when awakening
//  ESP.deepSleep(SLEEP_LENGTH, WAKE_RF_DISABLED); 
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

String printConnectionStatusAdafruitIO ( int which ) {
    switch ( which ) {
        case AIO_NET_CONNECTED:
            return "Connected";
            break;
        case AIO_NET_CONNECT_FAILED:
            return "Wrong password";
            break;
        case AIO_IDLE:
            return "Idle status";
            break;
        case AIO_NET_DISCONNECTED:
            return "Disconnected";
            break;
        default:
            return "Unknown";
            break;
    }
}

void startConnectionTimer() {
  connectionStartTime = millis();
  keepTryingToConnect = true;
}

void checkConnectionTimer() {
  if (millis() > connectionStartTime + MAX_CONNECTION_TIME) {
    keepTryingToConnect = false;
    Serial.println("Connection timeout");
  }
}
