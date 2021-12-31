// Ben Branch UAT Drone project
// Based heavily on AnishDey27's NodeMCU Flight controler which appears to be based on Joop Brokings as well.
// Also used a lot of randomnerdtutorials.com
// Challenges faced so far included adapting code so it compiles on a ESP Wroom32, Adapting the control scheme to a website and getting it to communicate by websockets,
// and hooking it up so the motors recognize what it is attatched to. Further work needs to be done on the MPU code to make it compatable. Rather than HTTP requests 
// which are less reliable or UDP. I had seen it done with other RC projects so figured it would be worth it, but it took a lot of time.
// Including arduino, wifi
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
//#include <Arduino_JSON.h>
#include <Wire.h>

//
//WiFiUDP UDP;
// Command recieving packet and state declaration
char packet[7];
boolean recvState;
int key_0; // 12
int roll_1; // 0
int pitch_2; // 50
int throttle_3; // 50
int yaw_4; // 50
int kp_5; // 175
int ki_6; // 15
int kd_7;

//-----------------------------------------------------------------------//
// states, pitch input, roll input, yaw input, throttle input, output from Electronic Speed Controllers
int ESCout_1 , ESCout_2 , ESCout_3 , ESCout_4;
int input_PITCH = 50;
int input_ROLL = 50;
int input_YAW;
int input_THROTTLE = 1100;
int state1, state2, state3, state4;
//-----------------------------------------------------------------------//
// int 16 of gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temperature, acc_total_vector
int16_t gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temperature, acc_total_vector;
// angle of pitch, roll, and yaw
float angle_pitch, angle_roll, angle_yaw;
// boolean for setting gyro angles
boolean set_gyro_angles;
// angle of roll acceleration, angle of pitch acceleration
float angle_roll_acc, angle_pitch_acc;
// angle of pitch output, angle of roll output
float angle_pitch_output, angle_roll_output;
// float to hold the elapsed time
float elapsedTime;
// long to hold time, previous time, and time2?
long Time, timePrev, time2;
// long for gyro x y and z calibration
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
//-----------------------------------------------------------------------//
// pitch for PID, roll for PID, yaw for PID
float pitch_PID, roll_PID, yaw_PID;
// variables for roll error, previous roll error, pitch errer, previous pitch error, and yaw error
float roll_error, roll_previous_error, pitch_error, pitch_previous_error, yaw_error;
// float for pids for pitch roll yaw
float roll_pid_p, roll_pid_d, roll_pid_i, pitch_pid_p, pitch_pid_i, pitch_pid_d, yaw_pid_p, yaw_pid_i;
// roll, pitch and yaw desired angles
float roll_desired_angle, pitch_desired_angle, yaw_desired_angle;
// PID parameters
double twoX_kp = 5;
double twoX_ki = 0.003;
double twoX_kd = 2;
double yaw_kp = 3;
double yaw_ki = 0.002;
//------------------------------------
// Network Credentials
const char* ssid = "Scytale";
const char* password = "";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
// Create a WebSocket object

AsyncWebSocket ws("/ws");

String dtring = "";
String message = "";
String values = "";


//Json Variable to Hold Slider Values
//JSONVar sliderValues;



// Initialize SPIFFS
void initFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  else {
    Serial.println("SPIFFS mounted successfully");
  }
}

// Initialize WiFi

//void notifyClients(String sliderValues) {
//  ws.textAll(sliderValues);
//}
// websockets
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  //AwsFrameInfo *info = (AwsFrameInfo*)arg;
  char* input = (char*)data;
  size_t inputLength = len;
  //dtring = input;

  StaticJsonDocument<128> doc;

  DeserializationError error = deserializeJson(doc, input, inputLength);

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  key_0 = doc[0]; // 12
  roll_1 = doc[1]; // 0
  pitch_2 = doc[2]; // 50
  throttle_3 = doc[3]; // 50
  yaw_4 = doc[4]; // 50
  kp_5 = doc[5]; // 175
  ki_6 = doc[6]; // 15
  kd_7 = doc[7]; // 121
  Serial.print(key_0);
  Serial.print(" ");
  Serial.print(roll_1);
  Serial.print(" ");
  Serial.print(pitch_2);
  Serial.print(" ");
  Serial.print(throttle_3);
  Serial.print(" ");
  Serial.print(yaw_4);
  Serial.print(" ");
  Serial.print(kp_5);
  Serial.print(" ");
  Serial.print(ki_6);
  Serial.print(" ");
  Serial.println(kd_7);

  //dtring = JSON.stringify(message);
  // values = JSON.parse(dtring);
  //Serial.println(message);
  //if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
  data[len] = 0;
  message = (char*)data;
  if (String(key_0) == "11") {
    input_ROLL = roll_1;
    input_PITCH = pitch_2;
    input_THROTTLE = 1000 + throttle_3 * 10;
    input_YAW = yaw_4;
  }
  if (String(key_0) == "12") {

    input_ROLL = roll_1;
    input_PITCH = pitch_2;
    input_THROTTLE = 1000 + throttle_3*10; //+ int(message[0]);
    input_YAW = yaw_4;
  }
//  if (String(key_0) == 12 {
//
//  twoX_kp = roll_1 / (float)100;
//    twoX_ki = pitch_2 / (float)1000;
//    twoX_kd = yaw_3 / (float)100;
//  }
//  if (String(key_0) == 12) {
//
//  twoX_kp = (float)(float)(int(message[0]) * 100 + int(message[0])) / (float)100;
//    twoX_ki = (float)(float)(int(message[0]) * 100 + int(message[0])) / (float)1000;
//    twoX_kd = (float)(float)(int(message[0]) * 100 + int(message[0])) / (float)100;
//  }
//  if (String(key_0) == 12) {
//
//  twoX_kp = (float)(int(message[0]) * 100 + int(message[0])) / (float)100;
//    twoX_ki = (float)int(message[0]) / (float)1000;
//    twoX_kd = (float)int(message[0]) / (float)100;
//  }
//  if (String(key_0) == 12) {
//
//  twoX_kp = (float)(int(message[0]))) / (float)100;
//    twoX_ki = (float)(int(message[0]) * 100 + int(message[0])) / (float)1000;
//    twoX_kd = (float)(int(message[0])) / (float)100;
//  }
//  if (String(key_0) == 12) {
//
//  twoX_kp = (float)int(message[0])) / (float)100;
//    twoX_ki = (float)(int(message[0])) / (float)1000;
//    twoX_kd = (float)(int(message[0]) * 100 + int(message[0])) / (float)100;
//  }
//  if (String(key_0) == 12) {
//
//  twoX_kp = (float)(int(message[0] * 100 + int(message[0]))) / (float)100;
//    twoX_ki = (float)(int(message[0]) * 100 + int(message[0])) / (float)1000;
//    twoX_kd = (float)(int(message[0])) / (float)100;
//  }
//  if (String(key_0) == 12) {
//
//  twoX_kp = (float)(int(sliderValue9.toInt())) / (float)100;
//    twoX_ki = (float)(int(sliderValue9.toInt()) * 100 + int(sliderValue9.toInt())) / (float)1000;
//    twoX_kd = (float)(int(sliderValue9.toInt()) * 100 + int(sliderValue9.toInt())) / (float)100;
//  }
//  if (String(key_0) == 12) {
//
//  twoX_kp = (float)(int(message[0])) * 100 + int(message[0]) / (float)100;
//    twoX_ki = (float)(int(message[0])) / (float)1000;
//    twoX_kd = (float)(int(message[0])) * 100 + int(message[0])) / (float)100;
//  }
  Serial.print(input_ROLL); Serial.print(" ");
  Serial.print(input_THROTTLE); Serial.print(" ");
  Serial.print(twoX_kp); Serial.print(" ");
  Serial.print(twoX_ki, 3); Serial.print(" ");
  Serial.print(twoX_kd); Serial.println();

  //}
  //data = null;
}
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
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

void setup() {

  pinMode(13, OUTPUT); pinMode(12, OUTPUT); pinMode(14, OUTPUT); pinMode(27, OUTPUT);
  delay(1300);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  //  while (WiFi.status() != WL_CONNECTED) {
  //    Serial.print('.');
  //    delay(1000);
  //  }
  initFS();
  server.serveStatic("/", SPIFFS, "/");

  initWebSocket();
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });
  server.begin();
  //WiFi.mode(WIFI_STA);
  //WiFi.begin("Laptop", "lppasspi");

  //REG_WRITE(GPIO_ENABLE_REG, BIT14); REG_WRITE(GPIO_ENABLE_REG, BIT12); REG_WRITE(GPIO_ENABLE_REG, BIT13); REG_WRITE(GPIO_ENABLE_REG, BIT27);

  while (WiFi.status() != WL_CONNECTED) {
    // have to go back and figure this reg write stuff out
    REG_WRITE(GPIO_OUT_W1TS_REG, BIT14); REG_WRITE(GPIO_OUT_W1TS_REG, BIT12); REG_WRITE(GPIO_OUT_W1TS_REG, BIT13); REG_WRITE(GPIO_OUT_W1TS_REG, BIT27); //GPIO2 HIGH (set)
    delayMicroseconds(1000);
    REG_WRITE(GPIO_OUT_W1TC_REG, BIT14); REG_WRITE(GPIO_OUT_W1TC_REG, BIT12); REG_WRITE(GPIO_OUT_W1TC_REG, BIT13); REG_WRITE(GPIO_OUT_W1TC_REG, BIT27); //GPIO2 LOW (clear)
    delayMicroseconds(1000);



  }
  Serial.println(WiFi.localIP());

  //GPOS = (1 << 14);GPOS = (1 << 12);GPOS = (1 << 13);GPOS = (1 << 15);
  //GPOC = (1 << 14);GPOC = (1 << 12);GPOC = (1 << 13);GPOC = (1 << 15);


  //    initFS();

  // configure LED PWM functionalitites
  //    ledcSetup(ledChannel1, freq, resolution);
  //    ledcSetup(ledChannel2, freq, resolution);
  //    ledcSetup(ledChannel3, freq, resolution);
  //    ledcSetup(ledChannel4, freq, resolution);
  //    ledcSetup(ledChannel5, freq, resolution);
  //    ledcSetup(ledChannel6, freq, resolution);
  //    ledcSetup(ledChannel7, freq, resolution);
  //    ledcSetup(ledChannel8, freq, resolution);

  // attach the channel to the GPIO to be controlled
  //    ledcAttachPin(ledPin1, ledChannel1);
  //    ledcAttachPin(ledPin2, ledChannel2);
  //    ledcAttachPin(ledPin3, ledChannel3);
  //    ledcAttachPin(ledPin4, ledChannel4);
  //    ledcAttachPin(ledPin5, ledChannel5);
  //    ledcAttachPin(ledPin6, ledChannel6);
  //    ledcAttachPin(ledPin7, ledChannel7);
  //    ledcAttachPin(ledPin8, ledChannel8);


  //    initWebSocket();

  // Web Server Root URL

  //void loop() {
  //  ledcWrite(ledChannel1, dutyCycle1);
  //  ledcWrite(ledChannel2, dutyCycle2);
  //  ledcWrite(ledChannel3, dutyCycle3);
  //  ledcWrite(ledChannel4, dutyCycle4);
  //  ledcWrite(ledChannel5, dutyCycle5);
  //  ledcWrite(ledChannel6, dutyCycle6);
  //  ledcWrite(ledChannel7, dutyCycle7);
  //  ledcWrite(ledChannel8, dutyCycle8);
  //
  //  ws.cleanupClients();
  //}


  //-----------------------------------------------------------------------//
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  delay(1000);
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {
    if (cal_int % 125 == 0)Serial.print(".");
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 14);
    while (Wire.available() < 14);
    acc_x = Wire.read() << 8 | Wire.read();
    acc_y = Wire.read() << 8 | Wire.read();
    acc_z = Wire.read() << 8 | Wire.read();
    temperature = Wire.read() << 8 | Wire.read();
    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    REG_WRITE(GPIO_OUT_W1TS_REG, BIT14); REG_WRITE(GPIO_OUT_W1TS_REG, BIT12); REG_WRITE(GPIO_OUT_W1TS_REG, BIT13); REG_WRITE(GPIO_OUT_W1TS_REG, BIT27); //GPIO2 HIGH (set)
    //GPOS = (1 << 14);GPOS = (1 << 12);GPOS = (1 << 13);GPOS = (1 << 15);
    delayMicroseconds(250);
    REG_WRITE(GPIO_OUT_W1TC_REG, BIT14); REG_WRITE(GPIO_OUT_W1TC_REG, BIT12); REG_WRITE(GPIO_OUT_W1TC_REG, BIT13); REG_WRITE(GPIO_OUT_W1TC_REG, BIT27);; //GPIO2 LOW (clear)
    //GPOC = (1 << 14);GPOC = (1 << 12);GPOC = (1 << 13);GPOC = (1 << 15);
    delayMicroseconds(250);
    delay(5000);
  }
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
  Serial.println(gyro_x_cal);
  //-----------------------------------------------------------------------//
  Time = micros();
}

void loop() {

  REG_WRITE(GPIO_OUT_W1TS_REG, BIT14); REG_WRITE(GPIO_OUT_W1TS_REG, BIT12); REG_WRITE(GPIO_OUT_W1TS_REG, BIT13); REG_WRITE(GPIO_OUT_W1TS_REG, BIT27); //GPIO2 HIGH (set)
  //GPOS = (1 << 14);GPOS = (1 << 12);GPOS = (1 << 13);GPOS = (1 << 15);
  timePrev = Time;
  Time = micros();
  elapsedTime = (float)(Time - timePrev) / (float)1000000;
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  while (Wire.available() < 14);
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  angle_pitch += gyro_x * elapsedTime * 0.01526717557;
  angle_roll += gyro_y * elapsedTime * 0.01526717557;
  angle_yaw += gyro_z * elapsedTime * 0.01526717557;
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;
  angle_pitch_acc += 0;
  angle_roll_acc += 0;
  if (set_gyro_angles) {
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
  }
  else {
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_gyro_angles = true;
  }
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
  //-----------------------------------------------------------------------//
  roll_desired_angle = 3 * ((float)input_ROLL / (float)10 - (float)5);
  pitch_desired_angle = 3 * ((float)input_PITCH / (float)10 - (float)5);
  //yaw_desired_angle =0;

  roll_error =  angle_roll_output - roll_desired_angle;
  pitch_error = angle_pitch_output - pitch_desired_angle;
  yaw_error = angle_yaw - yaw_desired_angle;

  roll_pid_p = twoX_kp * roll_error;
  pitch_pid_p = twoX_kp * pitch_error;
  yaw_pid_p = yaw_kp * yaw_error;

  if (-3 < roll_error < 3) {
    roll_pid_i = roll_pid_i + (twoX_ki * roll_error);
  }
  if (-3 < pitch_error < 3) {
    pitch_pid_i = pitch_pid_i + (twoX_ki * pitch_error);
  }
  if (-3 < yaw_error < 3) {
    yaw_pid_i = yaw_pid_i + (yaw_ki * yaw_error);
  }

  roll_pid_d = twoX_kd * ((roll_error - roll_previous_error) / elapsedTime);
  pitch_pid_d = twoX_kd * ((pitch_error - pitch_previous_error) / elapsedTime);
  roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
  pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
  yaw_PID = yaw_pid_p + yaw_pid_i;

  if (roll_PID < -400) {
    roll_PID = -400;
  }
  else if (roll_PID > 400) {
    roll_PID = 400;
  }
  if (pitch_PID < -400) {
    pitch_PID = -400;
  }
  else if (pitch_PID > 400) {
    pitch_PID = 400;
  }
  if (yaw_PID < -400) {
    yaw_PID = -400;
  }
  else if (yaw_PID > 400) {
    yaw_PID = 400;
  }

  ESCout_1 = input_THROTTLE - roll_PID - pitch_PID - yaw_PID;
  ESCout_2 = input_THROTTLE + roll_PID - pitch_PID + yaw_PID;
  ESCout_3 = input_THROTTLE + roll_PID + pitch_PID - yaw_PID;
  ESCout_4 = input_THROTTLE - roll_PID + pitch_PID + yaw_PID;

  if (ESCout_1 > 2000) ESCout_1 = 2000;
  else if (ESCout_1 < 1100) ESCout_1 = 1100;
  if (ESCout_2 > 2000) ESCout_2 = 2000;
  else if (ESCout_2 < 1100) ESCout_2 = 1100;
  if (ESCout_3 > 2000) ESCout_3 = 2000;
  else if (ESCout_3 < 1100) ESCout_3 = 1100;
  if (ESCout_4 > 2000) ESCout_4 = 2000;
  else if (ESCout_4 < 1100) ESCout_4 = 1100;

  roll_previous_error = roll_error;
  pitch_previous_error = pitch_error;
  //-----------------------------------------------------------------------//
  while ((micros() - Time) < 1000);
  state1 = 1; state2 = 1; state3 = 1; state4 = 1;
  while (state1 == 1 || state2 == 1 || state3 == 1 || state4 == 1) {
    time2 = micros();
    //    if((time2 - Time) >= ESCout_1 && state1 == 1){ GPOC = (1 << 14); state1=0;}
    //    if((time2 - Time) >= ESCout_2 && state2 == 1){ GPOC = (1 << 12);state2=0;}
    //    if((time2 - Time) >= ESCout_3 && state3 == 1){ GPOC = (1 << 13);state3=0;}
    //    if((time2 - Time) >= ESCout_4 && state4 == 1){ GPOC = (1 << 15);state4=0;}
    if ((time2 - Time) >= ESCout_1 && state1 == 1) {
      REG_WRITE(GPIO_OUT_W1TC_REG, BIT14);
      //GPIO_OUT_W1TC_REG = (BIT2 << 14);
      state1 = 0;
    }
    if ((time2 - Time) >= ESCout_2 && state2 == 1) {
      //GPIO_OUT_W1TC_REG = (BIT2 << 12);
      REG_WRITE(GPIO_OUT_W1TC_REG, BIT12);
      state2 = 0;
    }
    if ((time2 - Time) >= ESCout_3 && state3 == 1) {
      //GPIO_OUT_W1TC_REG = (BIT2 << 13);
      REG_WRITE(GPIO_OUT_W1TC_REG, BIT13);
      state3 = 0;
    }
    if ((time2 - Time) >= ESCout_4 && state4 == 1) {
      //GPIO_OUT_W1TC_REG = (BIT2 << 15);
      REG_WRITE(GPIO_OUT_W1TS_REG, BIT27);
      state4 = 0;
    }
  }
  //-----------------------------------------------------------------------//
  
  //-----------------------------------------------------------------------//
  Serial.print(angle_roll_output); Serial.print("  ");
  Serial.print(angle_pitch_output); Serial.print(" | ");
  Serial.print(roll_desired_angle); Serial.print("  ");
  Serial.print(pitch_desired_angle);
  Serial.println();
}
