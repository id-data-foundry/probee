// Released to the public domain
//
// This sketch will read an uploaded file and increment a counter file
// each time the sketch is booted.

// Be sure to install the Pico LittleFS Data Upload extension for the
// Arduino IDE from:
//    https://github.com/earlephilhower/arduino-pico-littlefs-plugin/
// The latest release is available from:
//    https://github.com/earlephilhower/arduino-pico-littlefs-plugin/releases

// Before running:
// 1) Select Tools->Flash Size->(some size with a FS/filesystem)
// 2) Use the Tools->Pico Sketch Data Upload tool to transfer the contents of
//    the sketch data directory to the Pico

#include <LittleFS.h>
//#include <LittleFS_Mbed_RP2040.h>
#include "DFDataset.h"
#include <Arduino_LSM6DSOX.h>
#include <PDM.h>
#include <WiFi.h>

// SSID of your Wifi network, the library currently does not support WPA2 Enterprise networks
const char* ssid = "<ssid>";
// Password of your Wifi network.
const char* password = "<wifi_pass>";
// Data Foundry address
const char* datafoundry = "<data_foundry_instance>";
// Device name, needs to be unique in batch
const char* device_name = "<probe_name>";

// For the file upload we define these variables
String serverName = "<data_foundry_instance>";

// use ID of the IoT dataset
String serverPath = "/api/v1/datasets/ts/{}";

const int serverPort = 80;

// Create connection to dataset with server address, dataset id, and the access token
const char* api_token = "<dataset_api_token>";


// hardware
const int btn1Pin = D10;
const int btn2Pin = D4;
const int sldrPin = A3;
// default number of output channels
static const char channels = 1;
// default PCM output frequency
static const int frequency = 16000;

// control flow
long timestamp = 0;
long nextLogInstance = 0;
int btn1State = LOW;
int btn2State = LOW;

// data
int btn1Value = 0;
int btn2Value = 0;
int sldrValue = 0;
double acceleration = 0;
double accelFilter = 0;
// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[512];
// Number of audio samples read
volatile int samplesRead;
double sqsum;


void setup() {
  Serial.begin(115200);
  delay(5000);

  // establish Wifi connection
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print('.');
  }

  Serial.println("Connected to the WiFi network");

  // check Wifi periodically
  //  if (WiFi.status() == WL_CONNECTED) {
  // check timestamp
  while (timestamp <= 0) {
    // set time with current run time as offset
    timestamp = WiFi.getTime() - millis() / 1000;
    if (timestamp > 0) {
      Serial.print("Acquired time: "); Serial.println(timestamp);
      Serial.print("Time offset: "); Serial.println(millis());
    }

    delay(500);
  }
  //  }

  // --------------------------------------------------------------

  pinMode(btn1Pin, INPUT_PULLUP);
  pinMode(btn2Pin, INPUT_PULLUP);
  // TODO: pinMode(sldrPin, ...);

  // --------------------------------------------------------------

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  } else {
    Serial.print("IMU sample rate: ");
    Serial.println(IMU.accelerationSampleRate());
  }

  // --------------------------------------------------------------

  PDM.onReceive(onPDMdata);
  // Optionally set the gain
  // Defaults to 20 on the BLE Sense and -10 on the Portenta Vision Shields
  // PDM.setGain(30);
  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate for the Arduino Nano 33 BLE Sense 32
  // - a 32 kHz or 64 kHz sample rate for the Arduino Portenta Vision Shields
  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  // --------------------------------------------------------------

  LittleFS.begin();
  // LittleFS.remove("logdata.txt");
  
}

void loop() {

  // --------------------------------------------------------------

  int controlFlowBtns = 0;

  // collect data

  int temp = digitalRead(btn1Pin);
  if (temp != btn1State) {
    if (btn1State == HIGH) {
      Serial.println("Button 1 pressed");
      btn1Value = 1;
      logData();
      controlFlowBtns++;
    }
    btn1State = temp;
  }

  temp = digitalRead(btn2Pin);
  if (temp != btn2State) {
    if (btn2State == HIGH) {
      Serial.println("Button 2 pressed");
      btn2Value = 1;
      logData();
      controlFlowBtns++;
    }
    btn2State = temp;
  }

  // slide poti read
  int temp3 = analogRead(sldrPin);
  if (temp3 != sldrValue) {
    sldrValue = temp3;
  }

  logAccelerationLevel();

  soundPressure();

  // --------------------------------------------------------------

  // control flow

  // check whether both button have been pressed to connect to backend
  if (controlFlowBtns == 2) {
    Serial.println("Connecting to backend...");
    checkConnection();
  }

  // only log every couple of seconds (10s)
  if (nextLogInstance > millis()) {
    delay(20);
    return;
  }

  // --------------------------------------------------------------

  logData();

  // set next log instance in 10 seconds
  nextLogInstance += 10000;

}

void logData() {

  // print to flash storage format (CSV)
  char buffer[85];
  snprintf(buffer, 84, "%ld,%d,%d,%d,%lf,%lf\r\n", timestamp + millis() / 1000, btn1Value, btn2Value, sldrValue, acceleration, sqrt(sqsum));
  // Serial.print(buffer);

  // append to logdata file
  File f = LittleFS.open("logdata.txt", "a");
  if (f) {
    f.write(buffer, strlen(buffer));
    f.close();
  }

  // reset data
  btn1Value = 0;
  btn2Value = 0;
  sldrValue = 0;
  acceleration = 0;
  sqsum = 0;
}

void logAccelerationLevel() {
  float x, y, z;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
  }

  // filter IMU accel data
  accelFilter += (sqrt((sq(x) + sq(y) + sq(z)) / 3) - accelFilter) / 10;
  acceleration += abs(sqrt((sq(x) + sq(y) + sq(z)) / 3) - accelFilter);
}

void soundPressure() {
  if (samplesRead) {
    double squareMean = 0;
    for (int i = 0; i < samplesRead; i++) {
      squareMean += sq(sampleBuffer[i]);
    }
    sqsum += squareMean / samplesRead;

    // Clear the read count
    samplesRead = 0;
  }
}

bool checkConnection() {

  String getAll;
  String getBody;

  // append to logdata file
  File f = LittleFS.open("hello.txt", "a");
  if (f) {
    f.write("Hello World! Peter is awesome", strlen("Hello World! Peter is awesome"));
    f.close();
  }
  
  WiFiClient client;

  Serial.println("Connecting to server: " + serverName);

  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");

    // ---------------------------------------------------------------------

    // calculate the content length
    String head = "timestamp,button1,button2,slider,acceleration,soundPressure\r\n";
    uint32_t fileLen = f.size();
    uint32_t totalLen = head.length() + fileLen;

    // write HTTP POST request header
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: ", F("text/csv"));
    client.println("api_token: ", api_token);
    client.println("source_id: ", device_name);
    client.println("device_id: ", device_name);
    client.println();
    client.println();

    // write CSV header
    client.print(head);

    // open and print file contents
    File i = LittleFS.open("logdata.txt", "r");
    client.write(i);
    i.close();

    // clean up
    Serial.println("Cleaning local log...");

    LittleFS.remove("logdata.txt");

    Serial.println("Resume logging...");

    // ---------------------------------------------------------------------

    // create buffer
    const int bufSize = 2048;
    byte clientBuf[bufSize];
    int clientCount = 0;

    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + timoutTimer) > millis()) {
      Serial.print(".");
      delay(100);      
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (getAll.length()==0) { state=true; }
          getAll = "";
        }
        else if (c != '\r') { getAll += String(c); }
        if (state==true) { getBody += String(c); }
        startTimer = millis();
      }
      if (getBody.length()>0) { break; }
    }
    Serial.println();
    client.stop();
    Serial.println(getBody);
  }
  
  return true;
}

/**
   Callback function to process the data from the PDM microphone.
   NOTE: This callback is executed as part of an ISR.
   Therefore using `Serial` to print messages inside this function isn't supported.
 * */
void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
