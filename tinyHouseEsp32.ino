/*********
  o3, Tiny house Group_2
*********/

// Import required libraries
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_adc_cal.h"
#define BUTTON_PIN 21
 

//Others
int speed = 255;
int ledLevel = 255;
int temp = 0;
int LM35_Raw_Sensor1 = 0;
float Voltage = 0.0;

// Replace with your network credentials
const char *ssid = "bitia";
const char *password = "40000003";

//pin (fan)
const int fanPin1 = 27; 
const int fanPin2 = 26;
const int fanEnable1Pin = 15;
const int fanAutoPin = 22;
const int tempPin=33; 
//pin (led)
const int led1Pin = 14; 
const int led2Pin = 13; 
const int led3Pin = 12; 
//pin (Auto mode)
//const int BUTTON_PIN = 34;
const int LIGHT_SENSOR_PIN = 36;

//Motion Sensor 
int button_state;       
int last_button_state;
int counttime = 0;


// Setting PWM fan
const int freq = 2000;
const int pwmChannel = 1;
const int resolution = 8;
int dutyCycle = 0;

// Setting PWM led
const int freqLed = 5000;
const int ledChannel = 0;

//true variable(state)
bool fanState = 0;
bool doorState = 0;
bool led1State = 0;
bool led2State = 0;
bool led3State = 0;

bool fanAuto = 0;
bool doorAuto = 0;
bool ledsAuto = 0;

//get variable (values)
float temperatureValue = 0.0;
int fanValue = 52;
int luminosityValue = 0;
int flameValue = 0;

//manuel input

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


void dataSent(String data) {
  //LED
  if (data.startsWith("ledsAuto")) {
    ledsAuto = !ledsAuto;
  }
  if (data.startsWith("ledA")) {
    led1State = !led1State;
  }
  if (data.startsWith("ledB")) {
    led2State = !led2State;
  }
  if (data.startsWith("ledC")) {
    led3State = !led3State;
  }
  if (data.startsWith("ledLevel_")) {
    String value = data;
    value.replace("ledLevel_", " ");
    ledLevel = map(value.toInt(), 0, 100, 0, 255);
    //ledcWrite(pwmChannel,fanValue);
  }
  //FAN 
  if (data.startsWith("fan_toggle")) {
    fanState = !fanState;
  }
  if (data.startsWith("fanAuto")) {
    fanAuto = !fanAuto;
  }
  if (data.startsWith("speed_")) {
    String value = data;
    value.replace("speed_", " ");
    speed = map(value.toInt(), 0, 100, 52, 255);
    fanValue = speed;
    ledcWrite(pwmChannel,fanValue);
  }
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    
    dataSent((char *)data);
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:    
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void  ledAuto(){
  
}

uint32_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

void setup() {

  // sets the pins as outputs:
  pinMode(fanPin1, OUTPUT);
  pinMode(fanPin2, OUTPUT);
  pinMode(fanEnable1Pin, OUTPUT);
  //initial setting for fan
  digitalWrite(fanPin1, LOW);
  // attach the channel to the GPIO to be controlled FAN
  ledcAttachPin(fanEnable1Pin, pwmChannel);
  // configure PWM functionalitites FAN
  ledcSetup(pwmChannel, freq, resolution);
  // configure LED PWM functionalitites
  ledcSetup(2, freqLed, resolution);
  ledcSetup(3, freqLed, resolution);
  ledcSetup(4, freqLed, resolution);
  // attach the channel to the GPIO to be controlled LED
  ledcAttachPin(led1Pin, 2);
  ledcAttachPin(led2Pin, 3);
  ledcAttachPin(led3Pin, 4);

  //Motion sensor
  pinMode(BUTTON_PIN, INPUT_PULLUP); // set ESP32 pin to input pull-up mode
  button_state = digitalRead(BUTTON_PIN);

  
  //button mode pullup
  pinMode(fanAutoPin, INPUT);

  // Serial port for debugging purposes
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP Local IP Address
  Serial.println(WiFi.localIP());

  initWebSocket();

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", "");
  });

  // Start server
  server.begin();
}

void loop() {
  //Fan
  if (fanState) {
    digitalWrite(fanPin2,HIGH);
  } else {
    digitalWrite(fanPin2,LOW);
  }
  // Read LM35_Sensor1 ADC Pin
  LM35_Raw_Sensor1 = analogRead(tempPin);  
  // Calibrate ADC & Get Voltage (in mV)
  Voltage = readADC_Cal(LM35_Raw_Sensor1);
  // TempC = Voltage(mV) / 10
  temperatureValue = Voltage / 10;

  //Main function
    ws.textAll(""+
      String(fanState)+"v"+
      String(fanValue)+"v"+
      String(fanAuto)+"v"+
      String(temperatureValue)+"v"+
      String(ledsAuto)+"v"+
      String(led1State)+"v"+
      String(led2State)+"v"+
      String(led3State)+"v"+
      String(ledLevel)+"v"+
      String(doorAuto)+"v"+
      String(doorAuto)+"v"+
      String(doorAuto)+"v"+
      String(doorAuto)+"v"+
      String(doorAuto)+"v"+
      String(doorAuto)+"v"+
      String(doorState));
  delay(430);
  ws.cleanupClients();  
  //Mode
  int buttonState = digitalRead(fanAutoPin);
  if (buttonState) {
    fanAuto=!fanAuto;
  }

  if (led1State) {
    ledcWrite(2, ledLevel);
  }
  else{
    ledcWrite(2, 0);    
  }
  if (led2State) {
    ledcWrite(3, ledLevel);
  }
  else{
    ledcWrite(3, 0);    
  }
  if (led3State) {
    ledcWrite(4, ledLevel);
  }
  else{
    ledcWrite(4, 0);    
  }

  if (ledsAuto){
    int analogValue = analogRead(LIGHT_SENSOR_PIN);
    int pinAutoPin = digitalRead(BUTTON_PIN);  
      if (analogValue < 40) {
        //Serial.println(" => Dark");
        led1State = true; // turn on LED
        led2State = true; // turn on LED
        //Serial.println(analogValue);
      } else if (analogValue < 800) {
        //Serial.println(" => Dim");
        led1State = true; // turn on LED
        led2State = true; // turn on LED
        //Serial.println(analogValue);
      } else if (analogValue < 1300) {
        //Serial.println(" => Light");
        led1State = true; // turn on LED
        led2State = true; // turn on LED
        //Serial.println(analogValue);
      } else if (analogValue < 1420) {
        //Serial.println(" => Bright");
        led1State = false; // turn off LED
        led2State = false; // turn off LED
        //Serial.println(analogValue);
      } else {
        led1State = false; // turn off LED
        led2State = false; // turn off LED
        //Serial.println(" => Very bright");        
      }
    analogValue = analogRead(LIGHT_SENSOR_PIN);
  }
}