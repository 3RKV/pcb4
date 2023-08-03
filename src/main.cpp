#include <Arduino.h>
#include <ezButton.h>
#include <SPI.h>  //!!not used here, but needed to prevent a RTClib compile error!!
#include <RTClib.h>
#include <ESP32Servo.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h>
#include <GyverPortal.h>
#include "INA226.h"
#include <Preferences.h>
#include "Wire.h"
#include "FS.h"
#include "SD.h"

#define SHOW_CURRENT
#define WRITELOG
// #define DEBUG
#define WIFIDEBUG                  
#define BUTTON_DEBOUNCE_TIME 500             
#define SERVO_PWR 32                  
#define MATERIAL_SHAFT_STOP_SERVO_PIN 33
#define CABLE_SHAFT_STOP_SERVO_PIN 25
#define BKWD_PWM_PIN 26
#define FWD_PWM_PIN 27
#define EN_PIN 13
#define MOTOR_SELECTION_PIN 15

uint8_t posMin = 40; 
uint8_t posMax = 113;
bool direction = false;
bool moving = false;
bool control = 0;
bool wifiConnect = 1;

char logFileName [25] = "/";

String logData;

uint8_t move_f = 0; 

//---------PWM configure-------------
const uint8_t pwmResolution = 10;
float powerPercent = 0.6;
const uint16_t maxPWM = powerPercent*(pow(2,pwmResolution));
uint32_t startPwm = 0.25 * maxPWM;
uint32_t pwm = startPwm;
const uint32_t pwmFrequency = 15000;
uint8_t pwmChannelFwd = 5;
uint8_t pwmChannelBkwd = 4;
uint8_t accelDelay = 10;
//-----------------------------------

  //--create object:ezButton that attach to pin--
ezButton buttonManualOpen(35, INPUT);
ezButton buttonManualClose(34, INPUT);
ezButton buttonManualUnlock(39, INPUT);
ezButton run_button(12);
ezButton rearLimit(16, INPUT);
ezButton frontLimit(17, INPUT);
  //--create object:servo--
Servo MATERIAL_SHAFT_STOP_SERVO;
Servo CABLE_SHAFT_STOP_SERVO;
  //--create object:real time clock--
RTC_DS3231 rtc;
  //--create object:web UI--
GyverPortal ui;
  //--create object:INA 226--
INA226 INA(0x40);
//-----------------------------------

Preferences preferences;

//------------WebUI build------------
void build() {
  GP.BUILD_BEGIN(2000);
  GP.THEME(GP_DARK);
  GP.AREA_LOG(20,2000,"600px");
  GP.TITLE("v0.0.1");
  GP.BUILD_END();
}
//-----------------------------------

//---------Data date build-----------
String showDate(bool type)
{
    DateTime now = rtc.now();
    String date = "";
    String time = "";
    uint8_t day = now.day();
    String d ="";
    (day < 10) ? d = "0" + (String)day : d=(String)day;
    uint8_t month = now.month();
    String m ="";
    (month < 10) ? m = "0" + (String)month : m = (String)month;
    String year = (String)now.year();
    uint8_t hour = now.hour();
    String hh ="";
    (hour < 10) ? hh = "0" + (String)hour : hh = (String)hour;
    uint8_t minute = now.minute();
    String mm ="";
    (minute < 10) ? mm = "0" + (String)minute : mm = (String)minute;
    uint8_t second = now.second();
    String ss ="";
    (second < 10) ? ss = "0" + (String)second : ss = (String)second;
    uint8_t temperature = rtc.getTemperature();
    date = (d + m + year + "_");
    time = (hh + mm + ss + "T" +(String)temperature + "C");
    if (type) return date;
    else return time;
}
//-----------------------------------

//----------SD card event------------
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  ui.log.println("Listing directory: " + (String)dirname);
#ifdef DEBUG
  Serial.printf("Listing directory: %s\n", dirname);
#endif

  File root = fs.open(dirname);
  if(!root){
    ui.log.println("Failed to open directory");
  #ifdef DEBUG
    Serial.println("Failed to open directory");
  #endif
    return;
  }
  if(!root.isDirectory()){
    ui.log.println("Not a directory");
  #ifdef DEBUG
    Serial.println("Not a directory");
  #endif
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      String dirMsg = "  DIR : ";
      dirMsg += file.name();
      ui.log.println(dirMsg);
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      ui.log.print("  FILE: ");
      ui.log.println(file.name());
      ui.log.print("  SIZE: ");
      ui.log.println(file.size());
  #ifdef DEBUG
      Serial.print("  FILE: ");
      Serial.println(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
  #endif
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char * path){
  if(!fs.mkdir(path)){
    ui.log.println("mkdir failed");
#ifdef DEBUG
    Serial.println("mkdir failed");
#endif
  }
}

void removeDir(fs::FS &fs, const char * path){
  if(fs.rmdir(path)){
    ui.log.println("Dir removed");
#ifdef DEBUG
    Serial.println("Dir removed");
#endif
  } else {
    ui.log.println("rmdir failed");
#ifdef DEBUG
    Serial.println("rmdir failed");
#endif
  }
}

void readFile(fs::FS &fs, const char * path){
  File file = fs.open(path);
  if(!file){
    ui.log.println("Failed to open file for reading");
#ifdef DEBUG
    Serial.println("Failed to open file for reading");
#endif
    return;
  }

  Serial.print("Read from file: ");
  while(file.available()){
#ifdef DEBUG
    Serial.write(file.read());
#endif
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  File file = fs.open(path, FILE_WRITE);
  if(!file){
    ui.log.println("Failed to open file for writing");
#ifdef DEBUG
    Serial.println("Failed to open file for writing");
#endif
    return;
  }
  if(!file.print(message)){
    ui.log.println("Write failed");
#ifdef DEBUG
    Serial.println("Write failed");
#endif
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  File file = fs.open(path, FILE_APPEND);
  if(!file){
    ui.log.println("Failed to open file for appending");
#ifdef DEBUG
    Serial.println("Failed to open file for appending");
#endif
    return;
  }
  if(!file.print(message)){
    ui.log.println("Append failed");
#ifdef DEBUG
    Serial.println("Append failed");
#endif
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
  if (fs.rename(path1, path2)) {
    ui.log.println("File renamed");
#ifdef DEBUG
    Serial.println("File renamed");
#endif
  } else {
    ui.log.println("Rename failed");
#ifdef DEBUG
    Serial.println("Rename failed");
#endif
  }
}

void deleteFile(fs::FS &fs, const char * path){
  if(fs.remove(path)){
    ui.log.println("Delete failed");
#ifdef DEBUG
    Serial.println("File deleted");
#endif
  } else {
    ui.log.println("Delete failed");
#ifdef DEBUG
    Serial.println("Delete failed");
#endif
  }
}

void testFileIO(fs::FS &fs, const char * path){
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if(file){
    len = file.size();
    size_t flen = len;
    start = millis();
    while(len){
      size_t toRead = len;
      if(toRead > 512){
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    ui.log.println((String)flen + "u bytes read for" +  (String)end + "ms");
  #ifdef DEBUG
    Serial.printf("%u bytes read for %u ms\n", flen, end);
  #endif
    file.close();
  } else {
  #ifdef DEBUG
    Serial.println("Failed to open file for reading");
  #endif
    ui.log.println("Failed to open file for reading");
  }
  file = fs.open(path, FILE_WRITE);
  if(!file){
    ui.log.println("Failed to open file for writing");
  #ifdef DEBUG
    Serial.println("Failed to open file for writing");
  #endif
    return;
  }
  size_t i;
  start = millis();
  for(i=0; i<2048; i++){
    file.write(buf, 512);
  }
  end = millis() - start;
  ui.log.println((String)1048576 + "bytes written for" + (String)end + "ms");
  Serial.printf("%u bytes written for %u ms\n", 1048576, end);
  file.close();
}
//-----------------------------------

//---------Send messange-------------
void sendMsg (String msg)
{
#ifdef DEBUG
  Serial.println(showDate(0)+msg);
#endif

#ifdef WIFIDEBUG
  if (wifiConnect) ui.log.println(showDate(0) + ":" + msg);
#endif

#ifdef WRITELOG
  logData += msg;
  static uint64_t logTimer = 0;
 if (millis() - logTimer > (maxPWM*accelDelay))
 {
   logTimer = millis();
   appendFile(SD, logFileName, logData.c_str());
   logData = "";
 }
#endif
}
//-----------------------------------

//-----------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);
//-------connet to Wifi-------------
  WiFi.mode(WIFI_STA);
  WiFi.begin("Radient_lab", "TYU_!jqw");
  uint32_t notConnectedCounter = 0;
  while (WiFi.status() != WL_CONNECTED) {
      delay(100);
      Serial.println("Wifi connecting...");
      notConnectedCounter++;
 //--event:wifi not found--
      if(notConnectedCounter > 10) {
        wifiConnect = 0;
      Serial.println("!!WiFi not connecting. Turn on WiFi and reboot your device to reconnect!!");
      goto SKIP_WEB_UI_BUILD;
      }
  }
  Serial.println("Wifi connected, IP address: ");
  Serial.println(WiFi.localIP());

  ui.attachBuild(build);
  ui.start();
  ui.log.start(4096);   //buffer size

 //--Enable OTA update--
  ArduinoOTA.begin();
  delay(300);
  ui.log.clear();
  ui.log.println("|Log entries will be here|");

  SKIP_WEB_UI_BUILD:
  //-----------------------------------

  //--rtc initial--
  rtc.begin();
  rtc.adjust(DateTime(__DATE__, __TIME__));

  //--event: rtc lost power--
  if (rtc.lostPower()) {
  ui.log.println("!!RTC lost power, let's set the time!!");
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  //--event: SD card initialise
   if(!SD.begin(5)){
    ui.log.println("Card Mount Failed");
#ifdef DEBUG
    Serial.println("Card Mount Failed");
#endif
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    ui.log.println("No SD card attached");
#ifdef DEBUG
    Serial.println("No SD card attached");
#endif
    return;
  }
  String msg = "SD Card Type: ";
  if(cardType == CARD_MMC){
  // sendMsg("MMC");
    msg += "MMC";
  } else if(cardType == CARD_SD){
  // sendMsg("SDSC");
    msg += "SDSC";
  } else if(cardType == CARD_SDHC){
  // sendMsg("SDHC");
    msg += "SDHC";
  } else {
  // sendMsg("UNKNOWN");
    msg += "UNKNOWN";
  }
    ui.log.println(msg);
#ifdef DEBUG
    Serial.println(msg);
#endif

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  ui.log.println("SD Card Size: " + (String)cardSize + "MB");
#ifdef DEBUG
  Serial.println("SD Card Size: " + (String)cardSize + "MB");
#endif

#ifdef WRITELOG
  char postfix [5] = ".txt";
  strcat(logFileName,showDate(1).c_str());
  strcat(logFileName,showDate(0).c_str());
  strcat(logFileName,postfix);
  ui.log.println("Create logfile, name:" + (String)logFileName);
#endif
  listDir(SD, "/", 0);
  writeFile(SD, logFileName, "Start\n");
  appendFile(SD, logFileName, "SD Initialise ok!\n");
  sendMsg("Total space: " + (String)(SD.totalBytes() / (1024 * 1024)) + "MB");
  sendMsg("Used space: " + (String)(SD.usedBytes() / (1024 * 1024)) + "MB" );

  //--current sensor connect--
  Wire.begin();
  if (!INA.begin()) 
  {
    sendMsg("!!INA could not connect. Fix and Reboot!!");
    goto SKIP_INA_CONNECT;
  }

  //--set INA params--
  INA.setMaxCurrentShunt(1, 0.002);
  INA.setAverage(7);
  
SKIP_INA_CONNECT:
  //--set debounce time--
  buttonManualOpen.setDebounceTime(BUTTON_DEBOUNCE_TIME); 
  buttonManualClose.setDebounceTime(BUTTON_DEBOUNCE_TIME);
  buttonManualUnlock.setDebounceTime(BUTTON_DEBOUNCE_TIME);
  run_button.setDebounceTime(5);
  rearLimit.setDebounceTime(5);
  frontLimit.setDebounceTime(5);
  run_button.setCountMode(COUNT_RISING);
  
  //--set pinmode--
  pinMode(SERVO_PWR, OUTPUT);
  pinMode(MATERIAL_SHAFT_STOP_SERVO_PIN, OUTPUT);
  pinMode(CABLE_SHAFT_STOP_SERVO_PIN, OUTPUT);
  pinMode(BKWD_PWM_PIN, OUTPUT);
  pinMode(FWD_PWM_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(MOTOR_SELECTION_PIN, OUTPUT);//relay
  
  //--set ADC REsolution--
  analogReadResolution(12);

  //--attaches the servo on pin to the servo object--
	MATERIAL_SHAFT_STOP_SERVO.setPeriodHertz(50);
  CABLE_SHAFT_STOP_SERVO.setPeriodHertz(50);
  MATERIAL_SHAFT_STOP_SERVO.attach(MATERIAL_SHAFT_STOP_SERVO_PIN, 700, 2000);
  CABLE_SHAFT_STOP_SERVO.attach(CABLE_SHAFT_STOP_SERVO_PIN, 700, 2000);
 
  digitalWrite(SERVO_PWR, HIGH);

  ledcSetup(pwmChannelFwd, pwmFrequency, pwmResolution);
  ledcSetup((pwmChannelBkwd), pwmFrequency, pwmResolution);
  ledcAttachPin(FWD_PWM_PIN, pwmChannelFwd);
  ledcAttachPin(BKWD_PWM_PIN, pwmChannelBkwd);
  if (wifiConnect)
  {
    ui.log.println(showDate(1));
  }

  #ifdef DEBUG
    Serial.println(showDate(1));
  #endif

  digitalWrite(LED_BUILTIN, HIGH);
  ledcWrite(pwmChannelFwd,pwm);
  ledcWrite(pwmChannelBkwd,pwm);
  digitalWrite(EN_PIN,LOW);
  MATERIAL_SHAFT_STOP_SERVO.write(posMin);
  CABLE_SHAFT_STOP_SERVO.write(posMin);

  preferences.begin("paramNamespace");
  direction = preferences.getBool("direction",0);
  preferences.end();
  }
//-----------------------------------

//-----------Servo event-------------
void servoEvent (bool Event = 1)
{
  if (Event)
  {
    if (direction == 1) 
    {
      CABLE_SHAFT_STOP_SERVO.write(posMin);
      delay(500);
      MATERIAL_SHAFT_STOP_SERVO.write(posMin);
      return;
    }
    if (direction == 0) 
    {
      MATERIAL_SHAFT_STOP_SERVO.write(posMin);
      delay(500);
      CABLE_SHAFT_STOP_SERVO.write(posMin);
      return;
    }
  }
  if (!Event)
  {
   MATERIAL_SHAFT_STOP_SERVO.write(posMax);
   CABLE_SHAFT_STOP_SERVO.write(posMax);
   return;
  }
}
//-----------------------------------

//--------------Current--------------
void currentControl()
{
float current = abs(INA.getCurrent_mA()/1000.00);
float busVoltage = (INA.getBusVoltage());
#ifdef SHOW_CURRENT
  String text;
  text = "\ncurrent = " + (String)current + "A\n" + "busVoltage = " + (String)busVoltage + "V\n" /* + "power = " + (String)power + "W/n" */;
  sendMsg(text);
#endif
}
//-----------------------------------

//-----------Run event---------------
void run(bool dir, bool automode)
{

 sendMsg("check moving params:\nAutomode:" + (String)automode);
 if (!moving)
 {
    moving = true;
    if (automode)
    {
      if (direction)
      {
        direction = false;
        digitalWrite(MOTOR_SELECTION_PIN, HIGH);
      }
      else if (!direction) 
      {
        direction = true;
        digitalWrite(MOTOR_SELECTION_PIN, LOW);
      }
      preferences.begin("paramNamespace");
      preferences.putBool("direction",direction);
      preferences.end();
   }
   else 
   {
     direction = dir;
     preferences.begin("paramNamespace");
     preferences.putBool("direction",dir);
     preferences.end();
     if (dir) digitalWrite(MOTOR_SELECTION_PIN, LOW);
     else digitalWrite(MOTOR_SELECTION_PIN, HIGH);
   }
   sendMsg("\nDirection" + (String)dir);
   pwm = startPwm;
  } 
 else moving = false;
}
//-----------------------------------

//-----------Stop func---------------
void stop()
{
    sendMsg("stop");
	  moving = false;
    control = false;
    pwm = 0;
    ledcWrite(pwmChannelFwd,pwm);
    ledcWrite(pwmChannelBkwd,pwm);
	  digitalWrite(EN_PIN,LOW);
    servoEvent(1);
	}
//-----------------------------------

//-------------PWM ++----------------
void pwmAccel ()
{
  if (pwm <= 200)
  {
    servoEvent(0);
    digitalWrite(EN_PIN,HIGH);
    control = true;
  }
  static uint64_t tick = 0;
    if ((pwm < maxPWM) && (millis() - tick >= accelDelay) && (moving))
      {
        tick = millis();
        pwm++;
      }
    else if ((pwm > startPwm) && (!moving))
	  //&& (millis() - tick >= accelDelay) 
      {
        // tick = millis();
        pwm--;
      } 
    (direction == 0) ? ledcWrite(pwmChannelFwd,pwm) : ledcWrite(pwmChannelBkwd,pwm);
    if ((!moving)&&(pwm == startPwm)) stop();
  return;
}
//-----------------------------------

//---------Check button func---------
void buttonCheck()
{
  //--button state updaete--
  // buttonManualOpen.loop(); 
  // buttonManualClose.loop(); 
  // buttonManualUnlock.loop();
  run_button.loop();
  rearLimit.loop();
  frontLimit.loop();
  static unsigned long autorunTimer;
  //--event:button change--
  // if ((buttonManualOpen.isReleased())&&(!moving)) run(true,false);
  // else if ((buttonManualOpen.isPressed())&&(run_button.getCount() == 0)) run(true,false);
  // else if ((buttonManualClose.isReleased())&&(!moving)) run(false,false);
  // else if ((buttonManualClose.isPressed())&&(run_button.getCount() == 0)) run(false,false);
  // else if (buttonManualUnlock.isReleased()) servoEvent(0);   
  // else if ((buttonManualUnlock.isPressed())&&(!moving)) servoEvent(1);
  // else 
  if ((run_button.isReleased())&&(millis() - autorunTimer > 3000))
  {
    if (run_button.getCount() >= 2) run_button.resetCount();
    run(false,true);
    autorunTimer = millis();
  }

  if (moving)
  {
   if (rearLimit.isReleased()) if (/* (pwm > startPwm)&& */(!direction)&&(moving)) run(false,false);
   if (frontLimit.isReleased()) if (/* (pwm > startPwm)&& */(direction)&&(moving)) run(false,false);
  }
}
//-----------------------------------

//------------Serial msg-------------
#ifdef DEBUG
void serialEvent()
{
  String inputMessage = "";
  inputMessage = Serial.readStringUntil('\n');
  inputMessage.trim();
  sendMsg(inputMessage);
  if (inputMessage == "show_current") currentControl();
}
#endif
//-----------------------------------

//-----------------------------------
void loop() 
{
  //--Check the update request over the air (if there is, start the update)--
  ArduinoOTA.handle();
  ui.tick();

  buttonCheck();
  if ((moving)||(pwm > startPwm)) pwmAccel();
  if (control) currentControl();
}
//-----------------------------------