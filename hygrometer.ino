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

#define SLEEP_LENGTH          60 * 1e6 // Hibernation length in microseconds
#define MAX_CONNECTION_TIME   8 * 1e3 // WiFi connection timeout in millis
boolean keepTryingToConnect = true; // For WiFi connection timeout
long connectionStartTime = 0;


#include <DHT.h>

#define HUMIDITY_SENSOR_PIN 12
#define DHTTYPE DHT22
DHT dht(HUMIDITY_SENSOR_PIN, DHTTYPE);

// Sensor variables
float humidity;
float temperature; 
float batteryLevel;

//#define LED_PIN 13

// Adafruit feeds
AdafruitIO_Feed *humidityCommand = io.feed("humidity");
AdafruitIO_Feed *temperatureCommand = io.feed("temperature");
AdafruitIO_Feed *batteryCommand = io.feed("battery");

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

  runBatterySensor();
  printBatteryLevelReading();
  
  sendReadingsToCloud();

  hibernate();
}

// Sketch restarts after sleep, so loop() never runs
void loop() {}

void handleMessage(AdafruitIO_Data *data) {
  int command = data->toInt();
    Serial.print("received <- ");
    Serial.println(command);
}

void setupPins() {
  Serial.print("Setting up pins... ");
//  pinMode(LED_PIN, OUTPUT);
  pinMode(HUMIDITY_SENSOR_PIN, INPUT);
  Serial.println("Done.");
}

void setupSerial() {
  Serial.begin(9600);
  while (!Serial) { } // Wait while initializing
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
  batteryCommand->onMessage(handleMessage);

  // Wait for connection
  startConnectionTimer();
  while (io.status() < AIO_CONNECTED && keepTryingToConnect) {
    Serial.print(".");
    
    // Debugging connection
    // WiFi.printDiag(Serial);
    // Serial.println(printConnectionStatusAdafruitIO(io.status()));

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
//  Serial.println();
  Serial.println(io.statusText());
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());

  io.run();

  return true;
}

void blinkPin(int pin) {
  digitalWrite(pin, HIGH);
  delay(500);
  digitalWrite(pin, LOW);
}

//void blinkLED() {
//  blinkPin(LED_PIN);
//}

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
  Serial.println("C");
}

void printBatteryLevelReading() {
  Serial.print("Battery level: ");
  Serial.print(batteryLevel);
  Serial.println("%");
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

void hibernate() {
  // Remember to connect pins 16 and RST to let huzzah wake from deep sleep
  Serial.println("Going to sleep");
  ESP.deepSleep(SLEEP_LENGTH, WAKE_RF_DEFAULT);
  
  // Having the WAKE_RF_DISABLED leaves WiFi off when awakening
//  ESP.deepSleep(SLEEP_LENGTH, WAKE_RF_DISABLED); 
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

