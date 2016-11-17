//SD card

#include <avr/sleep.h>
#include <avr/power.h>
#include <SPI.h>
#include <SD.h>
int int_pin = 7;
int reset_pin = 5;
int led_pin = 18;
int SD_Vcc = 19;
int MPU_Vcc = 20;
int RTC_Vcc = 21;
int seconds=0;
int sleepFlag = 1;
File log_file;

//RTC
#include <DS3232RTC.h>    //http://github.com/JChristensen/DS3232RTC
#include <Time.h>         //http://www.arduino.cc/playground/Code/Time  
#include <Wire.h>         //http://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#define SQW_PIN 0 //interrupt Alarm pin
int WFRTC = 0;
//using another library as DS3232 seems to have timing issues in minutes
#include <DS3231.h>
DS3231  rtc(SDA, SCL);

//MPU6050
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 accelgyro(0x69); // <-- use for AD0 high
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO

//VOLTAGE
#include <Average.h>
Average <int>volt_reads(10);
int norm_volt;
int volt_pin=8;

//START/STOP
int saveFlag = 0;
int WFSS = 0;
void setup() {
  Serial.begin(9600);
  //GENERAL
  
  digitalWrite(reset_pin,HIGH);
  pinMode(led_pin,OUTPUT);
  digitalWrite(SD_Vcc,HIGH);
  delay(4000);
  digitalWrite(SD_Vcc,LOW);
  Serial.begin(9600);
  pinMode(SD_Vcc, OUTPUT);
  pinMode(MPU_Vcc, OUTPUT);
  pinMode(RTC_Vcc, OUTPUT);
    
  pinMode(reset_pin, OUTPUT);
  pinMode(int_pin, INPUT);
  //USBDevice.attach();
  delay(1000);
  digitalWrite(SD_Vcc,HIGH);
  delay(2000);
  digitalWrite(SD_Vcc,HIGH);
  digitalWrite(MPU_Vcc,HIGH);
  digitalWrite(RTC_Vcc,HIGH);

  //SD
  SD.begin(10);

  
  //SS
  attachInterrupt(digitalPinToInterrupt(int_pin), pinInterruptRoutine, LOW);

  //RTC
  setSyncProvider(RTC.get) ;   // the function to get the time from the RTC
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");
  Serial.println("RTC setup done. should output time every second");
  RTC.squareWave(SQWAVE_NONE);
  pinMode(SQW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SQW_PIN), alarmIsr, FALLING);
  RTC.setAlarm(ALM1_MATCH_SECONDS, 0, 0, 0);
  RTC.alarmInterrupt(ALARM_1, true);

  //backup library
  rtc.begin();

  //MPU6050
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  accelgyro.initialize();
  Serial.println("Testing MPU6050 connection...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println("accelerometer output should appear every second");

  //VOLTAGE
  pinMode(volt_pin,INPUT);
  analogReference(INTERNAL);

  //VARIABLES
  sleepFlag = 1;
  seconds=0;
  WFRTC = 0;
  WFSS = 0;
  saveFlag = 0;
}

void loop(){
  Serial.println("Hello World!");
  delay(1000);
  if(WFSS==1){
    
    Serial.println("WFSS==1");
    WFSS=0;
    if(saveFlag==0){
      flickr_once();
    }
    else{
      flickr_twice();
    }
    for(int wait_for_serial=0;wait_for_serial<5;wait_for_serial++){
      delay(1000);
      Serial.println("t");
      if(Serial.available()){
        char ch=Serial.read();
        if (ch=='s'){
         Serial.println("going to sleep");
         Serial.flush();
         delay(100);
         sleepFlag = 1;
        }
        else if(ch=='a'){
         Serial.println("keeping awake");
         Serial.flush();
         delay(100);
         sleepFlag = 0;
        }
        else if(ch=='t'){
          send_data();
        }
      }
      //Serial.println("Awake");
      if(wait_for_serial == 4){
        if(sleepFlag==1){
          enterSleep();  
        }
      }
    }
  attachInterrupt(digitalPinToInterrupt(int_pin), pinInterruptRoutine, LOW);
  attachInterrupt(digitalPinToInterrupt(SQW_PIN), alarmIsr, FALLING);  
  }
  //debugging RTC interrupt
  tmElements_t tm;
  Serial.print("WFRTC:");Serial.println(WFRTC);
  //Serial.print("WFSS: ");Serial.println(WFSS);
  //Serial.println(seconds);
  Serial.println(RTC.alarm(ALARM_1));
  RTC.read(tm);
  Serial.println(tm.Second, DEC);
  flickr();

  
  if(WFRTC==1){
    WFRTC = 0;    
    //Serial.println("WFRTC = 1");
    //sleepFlag=0;
    flickr();
    //Serial.println("starting transfer");
    //Serial.print("Initializing SD card...");
    SD.begin(10);
    delay(100);
    log_file = SD.open("/log.txt",FILE_WRITE);
    //Serial.println("saving data");
    if(log_file){
      //Serial.println("log_file opened");
//      log_file.print(saveFlag);log_file.print("\t");
//      log_file.print(hour());log_file.print("\t");
//      log_file.print(minute());log_file.print("\t");
//      log_file.print(day());log_file.print("\t");
//      log_file.print(month());log_file.print("\t");
//      log_file.print(year());log_file.print("\t");
      log_file.print(saveFlag);log_file.print("|");
      log_file.print(rtc.getTimeStr());log_file.print("|");
      log_file.print(rtc.getDateStr());log_file.print("|");
       
       //MPU6050
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      log_file.print("a:\t");
      log_file.print(ax); log_file.print("\t");
      log_file.print(ay); log_file.print("\t");
      log_file.println(az); 
    }
    log_file.close();
    if(sleepFlag==1){
      //enterSleep();  
    }
  attachInterrupt(digitalPinToInterrupt(int_pin), pinInterruptRoutine, LOW);
  attachInterrupt(digitalPinToInterrupt(SQW_PIN), alarmIsr, FALLING);
  }
  
  //voltage indicator
  int sensorValue = analogRead(volt_pin);
  volt_reads.push(sensorValue);
  Serial.print("sensor value: ");
  Serial.println(sensorValue);
  Serial.print("Mode value is: ");
  norm_volt = volt_reads.mode();
  Serial.println(norm_volt);
  if(norm_volt<430){
    rapid_glow();
  }
  else if(norm_volt<440){
    slow_glow();
  } 
  if(sleepFlag==1){
    enterSleep();  
  }  
}

//SD Card
void pinInterruptRoutine(void)
{
  detachInterrupt(digitalPinToInterrupt(int_pin));
  detachInterrupt(digitalPinToInterrupt(SQW_PIN));
  WFSS = 1;
  // start/stop
  if(saveFlag==0){
    saveFlag=1;
  }
  else{
    saveFlag=0;
  }
}
//RTC

void alarmIsr() {
  //Serial.println("int happened");
  //sleepFlag=0;
  //detachInterrupt(digitalPinToInterrupt(SQW_PIN));
  detachInterrupt(digitalPinToInterrupt(int_pin));
  detachInterrupt(digitalPinToInterrupt(SQW_PIN));
  WFRTC = 1;
  
}
void enterSleep(void)
{
  Serial.end();
  delay(100);
  USBCON |= _BV(FRZCLK);  //freeze USB clock
  PLLCSR &= ~_BV(PLLE);   // turn off USB PLL
  USBCON &= ~_BV(USBE);   // disable USB
  digitalWrite(SD_Vcc,LOW);
  //SPI.end();
  /* Setup pin7 as an interrupt and attach handler. */
  attachInterrupt(digitalPinToInterrupt(int_pin), pinInterruptRoutine, LOW);
  attachInterrupt(digitalPinToInterrupt(SQW_PIN), alarmIsr, FALLING);
  delay(100);
  power_adc_disable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  /* The program will continue from here. */
  /* First thing to do is disable sleep. */
  sleep_disable();
  power_adc_enable();
  delay(100); 
  digitalWrite(SD_Vcc,HIGH);
  MPU6050 accelgyro(0x69);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  accelgyro.initialize();
  Serial.begin(9600);
  USBDevice.attach();
  delay(100);
  Serial.begin(9600);
  delay(100);
}




//LED indications

void flickr_once(){
  digitalWrite(led_pin,HIGH);
  delay(50);
  digitalWrite(led_pin,LOW);
  delay(50);
}
void flickr_twice(){
  flickr_once();
  flickr_once();
}
void flickr(void){
  flickr_twice();
  flickr_twice();
}
void rapid_glow(){
  for(int outer=0;outer<2;outer++){
    for(int i=0;i<=200;i++){
      analogWrite(led_pin,i);
      delay(1);
    }
    for(int j=250;j>=0;j--){
      analogWrite(led_pin,j);
      delay(1);
    }
  }
}
void slow_glow(){
  for(int i=0;i<=200;i++){
      analogWrite(led_pin,i);
      delay(10);
  }
  for(int j=250;j>=0;j--){
    analogWrite(led_pin,j);
    delay(1);
  }
}

void send_data(){
  flickr();
  Serial.println("starting transfer");
  SD.begin(10);
  //  Serial.print("checking if SD connection is ok");Serial.println(SD.exists("/log.txt"));
  
  log_file = SD.open("/log.txt",FILE_READ);
  if(log_file){
      Serial.println("logfile opened");
      
      while (log_file.available()){
        Serial.write(log_file.read());
        Serial.flush();
      }
      Serial.println("transfer complete");
      Serial.flush();
  }
  else{
    Serial.println("couldn't open file");  
  }
  log_file.close();
  flickr();
  delay(2000);
  digitalWrite(reset_pin,LOW);
}

//

