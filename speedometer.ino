//**************************************************************************************************
//                                 BIG FONT (2-line) LCD CHARACTERS 
//                                   Adrian Jones, February 2015
//
//Initial idea by http://www.instructables.com/id/Custom-Large-Font-For-16x2-LCDs/step5/Arduino-Sketch/
//**************************************************************************************************
//********************************************************************************************
#include <Wire.h> 
#include <LCD.h>                     // Standard lcd library
#include <LiquidCrystal_I2C.h>       // library for I@C interface
#include <virtuabotixRTC.h>          // library for I@C interface (TIME)
#include <BluetoothSerial.h> //Header File for Serial Bluetooth, will be added by default into Arduino

// include library to read and write from flash memory
#include "EEPROM.h"

#define I2C_ADDR  0x27               // address found from I2C scanner
#define RS_pin    0                  // pin configuration for LCM1602 interface module
#define RW_pin    1
#define EN_pin    2
#define BACK_pin  3
#define D4_pin    4
#define D5_pin    5
#define D6_pin    6
#define D7_pin    7

BluetoothSerial ESP_BT; //Object for Bluetooth

LiquidCrystal_I2C lcd(I2C_ADDR, EN_pin, RW_pin, RS_pin, D4_pin, D5_pin, D6_pin, D7_pin, BACK_pin, POSITIVE);

const char custom[][8] PROGMEM = {                        // Custom character definitions
      { 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00 }, // char 1 
      { 0x18, 0x1C, 0x1E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F }, // char 2 
      { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0F, 0x07, 0x03 }, // char 3 
      { 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F }, // char 4 
      { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1E, 0x1C, 0x18 }, // char 5 
      { 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x1F, 0x1F }, // char 6 
      { 0x1F, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F }, // char 7 
      { 0x03, 0x07, 0x0F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F }  // char 8 
};

byte rpmMiddleSpike[] = {B00000, B00100, B00100, B00100, B00100, B00100, B00100, B11111 }; // char for rpm
byte rpmMiddleUnderscore[] = {B00000, B00000, B00000, B00000, B00000, B00000, B00000, B11111 }; // char for rpm
byte rpmStart[] = {B00000, B00100, B00100, B00100, B00100, B00100, B00100, B00111 }; // char for rpm
byte rpmEnd[] = {B00000, B00100, B00100, B00100, B00100, B00100, B00100, B11100 }; // char for rpm


// BIG FONT Character Set
// - Each character coded as 1-4 byte sets {top_row(0), top_row(1)... bot_row(0), bot_row(0)..."}
// - number of bytes terminated with 0x00; Character is made from [number_of_bytes/2] wide, 2 high
// - codes are: 0x01-0x08 => custom characters, 0x20 => space, 0xFF => black square

const char bigChars[][8] PROGMEM = {
      { 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // Space
      { 0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // !
      { 0x05, 0x05, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00 }, // "
      { 0x04, 0xFF, 0x04, 0xFF, 0x04, 0x01, 0xFF, 0x01 }, // #
      { 0x08, 0xFF, 0x06, 0x07, 0xFF, 0x05, 0x00, 0x00 }, // $
      { 0x01, 0x20, 0x04, 0x01, 0x04, 0x01, 0x20, 0x04 }, // %
      { 0x08, 0x06, 0x02, 0x20, 0x03, 0x07, 0x02, 0x04 }, // &
      { 0x05, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // '
      { 0x08, 0x01, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00 }, // (
      { 0x01, 0x02, 0x04, 0x05, 0x00, 0x00, 0x00, 0x00 }, // )
      { 0x01, 0x04, 0x04, 0x01, 0x04, 0x01, 0x01, 0x04 }, // *
      { 0x04, 0xFF, 0x04, 0x01, 0xFF, 0x01, 0x00, 0x00 }, // +
      { 0x20, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 
      { 0x04, 0x04, 0x04, 0x20, 0x20, 0x20, 0x00, 0x00 }, // -
      { 0x20, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // .
      { 0x20, 0x20, 0x04, 0x01, 0x04, 0x01, 0x20, 0x20 }, // /
      { 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00 }, // 0
      { 0x01, 0x02, 0x20, 0x04, 0xFF, 0x04, 0x00, 0x00 }, // 1
      { 0x06, 0x06, 0x02, 0xFF, 0x07, 0x07, 0x00, 0x00 }, // 2
      { 0x01, 0x06, 0x02, 0x04, 0x07, 0x05, 0x00, 0x00 }, // 3
      { 0x03, 0x04, 0xFF, 0x20, 0x20, 0xFF, 0x00, 0x00 }, // 4
      { 0xFF, 0x06, 0x06, 0x07, 0x07, 0x05, 0x00, 0x00 }, // 5
      { 0x08, 0x06, 0x06, 0x03, 0x07, 0x05, 0x00, 0x00 }, // 6
      { 0x01, 0x01, 0x02, 0x20, 0x08, 0x20, 0x00, 0x00 }, // 7
      { 0x08, 0x06, 0x02, 0x03, 0x07, 0x05, 0x00, 0x00 }, // 8
      { 0x08, 0x06, 0x02, 0x07, 0x07, 0x05, 0x00, 0x00 }, // 9
      { 0xA5, 0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // :
      { 0x04, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // ;
      { 0x20, 0x04, 0x01, 0x01, 0x01, 0x04, 0x00, 0x00 }, // <
      { 0x04, 0x04, 0x04, 0x01, 0x01, 0x01, 0x00, 0x00 }, // =
      { 0x01, 0x04, 0x20, 0x04, 0x01, 0x01, 0x00, 0x00 }, // >
      { 0x01, 0x06, 0x02, 0x20, 0x07, 0x20, 0x00, 0x00 }, // ?
      { 0x08, 0x06, 0x02, 0x03, 0x04, 0x04, 0x00, 0x00 }, // @
      { 0x08, 0x06, 0x02, 0xFF, 0x20, 0xFF, 0x00, 0x00 }, // A
      { 0xFF, 0x06, 0x05, 0xFF, 0x07, 0x02, 0x00, 0x00 }, // B
      { 0x08, 0x01, 0x01, 0x03, 0x04, 0x04, 0x00, 0x00 }, // C
      { 0xFF, 0x01, 0x02, 0xFF, 0x04, 0x05, 0x00, 0x00 }, // D
      { 0xFF, 0x06, 0x06, 0xFF, 0x07, 0x07, 0x00, 0x00 }, // E
      { 0xFF, 0x06, 0x06, 0xFF, 0x20, 0x20, 0x00, 0x00 }, // F
      { 0x08, 0x01, 0x01, 0x03, 0x04, 0x02, 0x00, 0x00 }, // G
      { 0xFF, 0x04, 0xFF, 0xFF, 0x20, 0xFF, 0x00, 0x00 }, // H
      { 0x01, 0xFF, 0x01, 0x04, 0xFF, 0x04, 0x00, 0x00 }, // I
      { 0x20, 0x20, 0xFF, 0x04, 0x04, 0x05, 0x00, 0x00 }, // J
      { 0xFF, 0x04, 0x05, 0xFF, 0x20, 0x02, 0x00, 0x00 }, // K
      { 0xFF, 0x20, 0x20, 0xFF, 0x04, 0x04, 0x00, 0x00 }, // L
      { 0x08, 0x03, 0x05, 0x02, 0xFF, 0x20, 0x20, 0xFF }, // M
      { 0xFF, 0x02, 0x20, 0xFF, 0xFF, 0x20, 0x03, 0xFF }, // N
      { 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00 }, // 0
      { 0x08, 0x06, 0x02, 0xFF, 0x20, 0x20, 0x00, 0x00 }, // P
      { 0x08, 0x01, 0x02, 0x20, 0x03, 0x04, 0xFF, 0x04 }, // Q
      { 0xFF, 0x06, 0x02, 0xFF, 0x20, 0x02, 0x00, 0x00 }, // R
      { 0x08, 0x06, 0x06, 0x07, 0x07, 0x05, 0x00, 0x00 }, // S
      { 0x01, 0xFF, 0x01, 0x20, 0xFF, 0x20, 0x00, 0x00 }, // T
      { 0xFF, 0x20, 0xFF, 0x03, 0x04, 0x05, 0x00, 0x00 }, // U
      { 0x03, 0x20, 0x20, 0x05, 0x20, 0x02, 0x08, 0x20 }, // V
      { 0xFF, 0x20, 0x20, 0xFF, 0x03, 0x08, 0x02, 0x05 }, // W
      { 0x03, 0x04, 0x05, 0x08, 0x20, 0x02, 0x00, 0x00 }, // X
      { 0x03, 0x04, 0x05, 0x20, 0xFF, 0x20, 0x00, 0x00 }, // Y
      { 0x01, 0x06, 0x05, 0x08, 0x07, 0x04, 0x00, 0x00 }, // Z
      { 0xFF, 0x01, 0xFF, 0x04, 0x00, 0x00, 0x00, 0x00 }, // [
      { 0x01, 0x04, 0x20, 0x20, 0x20, 0x20, 0x01, 0x04 }, // Backslash
      { 0x01, 0xFF, 0x04, 0xFF, 0x00, 0x00, 0x00, 0x00 }, // ]
      { 0x08, 0x02, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00 }, // ^
      { 0x20, 0x20, 0x20, 0x04, 0x04, 0x04, 0x00, 0x00 }  // _
};
byte col,row,nb=0,bc=0;                                   // general
byte bb[8];                                               // byte buffer for reading from PROGMEM
String temperatureString;
String voltageString;
String rpmString;
String btTempRead = "";

int speed = 99;
int temp = 50;
int voltage = 130;
int rpm = 4000;

virtuabotixRTC myRTC(3, 2, 4); // d3, d2, d4 - clk, dat, rst

// esp32 start
volatile int interruptCounter;
int totalInterruptCounter;
 
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
 
}
// esp32 end

float n = 1;
// Reading speed
int reedVal;
int maxReedCounter = 20;//min time (in ms) of one rotation (for debouncing)
int impossibleSpeed = 220;
int reedCounter;
float mph = 0.00;
long timer2 = 0;// time between one full rotation (in ms)
float circumference;
float radius = 13.5;// tire radius (in inches)- CHANGE THIS FOR YOUR OWN BIKE

byte savedCoolingType = 0;
float savedWheelSize = 0;

void setup() {
  Serial.begin(19200);
  if (!EEPROM.begin(5)) {
    Serial.println("failed to initialise EEPROM"); 
    delay(100000);
  }
  
  EEPROM.get(0, savedCoolingType);
  EEPROM.get(1, savedWheelSize);
  Serial.println("EEPROM Memory");
  for (int i = 0; i < 5; i++)
  {
    Serial.print(byte(EEPROM.read(i))); Serial.print(" ");
  }
  Serial.println();
  lcd.begin(20, 4);
  for (nb=0; nb<8; nb++ ) {                     // create 8 custom characters
    for (bc=0; bc<8; bc++) {
      bb[bc]= pgm_read_byte( &custom[nb][bc] );
    }
    lcd.createChar(nb + 1, bb);
  }
//  lcd.createChar ( 100, rpmMiddleSpike );
//  lcd.createChar ( 101, rpmMiddleUnderscore );
//  lcd.createChar ( 102, rpmStart );
//  lcd.createChar ( 103, rpmEnd );

  lcd.clear();

// atmega start
  // speedometer
//  pinMode(A1, INPUT);
  // voltage meter
//  pinMode(A2, INPUT);
//  atmega end
  myRTC.setDS1302Time(15, 22, 21, 7, 14, 1, 2018); //Here you write your actual time/date as shown above 

  //speed
    reedCounter = maxReedCounter;
  circumference = 2*3.14*radius;
   

// atmega timer start
 // TIMER SETUP- the timer interrupt allows preceise timed measurements of the reed switch
  //for mor info about configuration of arduino timers see http://arduino.cc/playground/Code/Timer1
  cli();//stop interrupts
  
//  //set timer1 interrupt at 1kHz

//  TCCR1A = 0;// set entire TCCR1A register to 0
//  TCCR1B = 0;// same for TCCR1B
//  TCNT1  = 0;
//  // set timer count for 1khz increments
//  OCR1A = 1999;// = (1/1000) / ((1/(16*10^6))*8) - 1
//  // turn on CTC mode
//  TCCR1B |= (1 << WGM12);
//  // Set CS11 bit for 8 prescaler
//  TCCR1B |= (1 << CS11);   
//  // enable timer compare interrupt
//  TIMSK1 |= (1 << OCIE1A);
//  
//  sei();//allow interrupts
// atmega timer end

  //esp32 1khz timer
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);
 
  ESP_BT.begin("KLR_Speedometer"); //Name of your Bluetooth Signal
  Serial.println("Bluetooth Device is Ready to Pair");
}

//*****************************************************************************************//
//                                      MAIN LOOP
//*****************************************************************************************//


void loop() {
  n++;
  speed = sin(n/50) * 80 + 80;
  temp = sin(n/30)* 52 + 52;
  voltage += random(-10, 10); //getVoltage(A2);
  rpm = sin(n/4)* 4500 + 4500;

//  Serial.println(speed);
//  Serial.println(interruptCounter);
  
  myRTC.updateTime();

  drawRpm(&lcd, rpm);
  drawTemperature(&lcd, temp);
  drawVoltage(&lcd, voltage / 10.0);
  drawTime(&lcd, myRTC.hours, myRTC.minutes);
  drawSpeed(&lcd, mph);

  if (ESP_BT.available()) {
    while (ESP_BT.available()) { //Check if we receive anything from Bluetooth 
      btTempRead = btTempRead + (char) ESP_BT.read(); //Read what we recevive
    }
    Serial.print("Received:"); 
    Serial.println(btTempRead); 

    if (btTempRead.startsWith("scooling")) {
        byte coolingType = (byte) getValue(btTempRead, ' ', 1).toInt();
        EEPROM.put(0, coolingType);
        EEPROM.commit();
        ESP_BT.println("Saved cooling type: " + String(coolingType));
    } else if (btTempRead.startsWith("swheel")) {
        float wheelSize = getValue(btTempRead, ' ', 1).toFloat();
        EEPROM.put(1, wheelSize);
        EEPROM.commit();

        ESP_BT.println("Saved wheel size: " + String(wheelSize));
    } else if (btTempRead.startsWith("gcooling")) {
        ESP_BT.println("Cooling type: " + String(savedCoolingType));
    } else if (btTempRead.startsWith("gwheel")) {
        ESP_BT.println("Wheel size: " + String(savedWheelSize));
    } else {
      ESP_BT.println("Error");
    }
    btTempRead = "";
  }
  delay(100);
}


// atmega timer start
//ISR(TIMER1_COMPA_vect) {//Interrupt at freq of 1kHz to measure reed switch
//  reedVal = analogRead(A1);//get val of A0
//  if (reedVal < 11){//if reed switch is closed
//    if (reedCounter == 0){//min time between pulses has passed
//      mph = (56.8*float(circumference))/float(timer2);//calculate miles per hour
//      timer2 = 0;//reset timer
//      reedCounter = maxReedCounter;//reset reedCounter
//    }
//    else{
//      if (reedCounter > 0){//don't let reedCounter go negative
//        reedCounter -= 1;//decrement reedCounter
//      }
//    }
//  }
//  else{//if reed switch is open
//    if (reedCounter > 0){//don't let reedCounter go negative
//      reedCounter -= 1;//decrement reedCounter
//    }
//  }
//  if (timer2 > 1000 || mph > impossibleSpeed){
//    mph = 0;//if no new pulses from reed switch- tire is still, set mph to 0
//  }
//  else{
//    timer2 += 1;//increment timer
//  } 
//}
// atmega timer end

void drawRpm(LiquidCrystal_I2C* lcd, int rpm) {
  rpmString = String((rpm / 1000.0), 1) + "K";
  lcd->setCursor(16, 0); 
  lcd->print(rpmString); 

  int rpmBars = rpm / 600;
  lcd->setCursor(rpmBars, 0); 
//  char a[15] = {(char) 11, (char) 10, (char) 9, (char) 10, (char) 9, (char) 10, (char) 9, (char) 10, (char) 9, (char) 10, (char) 9, (char) 10, (char) 9, (char) 10, (char) 12};
//  String rpmBarGraphics = String(a);
  String rpmBarGraphics =("|_|_|_|_|_|_!_!");
  rpmBarGraphics.remove(0, rpmBars);
  lcd->print(rpmBarGraphics); 

  lcd->setCursor(0, 0); 
  for (int i = 0; i < rpmBars; i++) {
      if (i == 0) {
        lcd->print((char) 8);
      } else if (i > 11) {
        lcd->print(F("X"));  
      } else {
        lcd->print((char) 255);
      }
  }
}

void drawTemperature(LiquidCrystal_I2C* lcd, int temperature) {
   if (temperature >= 100  ) {
    temperatureString = "";
  } else if (temperature >= 10) {
    temperatureString = " ";
  } else {
    temperatureString = "  ";
  }
  lcd->setCursor(16, 1); 
  temperatureString += String(temperature) + "C"; //(char) 223 + 
  lcd->print(temperatureString.c_str()); 
}

void drawVoltage(LiquidCrystal_I2C* lcd, float voltage) {
  lcd->setCursor(15, 2); 
  if (voltage < 10) {
    voltageString = " ";
  } else {
    voltageString = "";
  }
  
  voltageString += String(voltage, 1) + "V";
  lcd->print(voltageString.c_str());
}

void drawTime(LiquidCrystal_I2C* lcd, int hours, int minutes) {
  String timeString = String(hours) + ":";
  if (minutes < 10) {
    timeString += "0";
  }
  timeString += String(minutes);
  lcd->setCursor(15, 4); 
  lcd->print(timeString);
}

float getVoltage(int input) {
  int value = analogRead(input);
  return value / 4.6024;
}

void drawSpeed(LiquidCrystal_I2C* lcd, int speed) {
  lcd->setCursor(9, 4); 
  lcd->print(F("km/h")); 
  String speedText;
  if (speed < 10) {
    speedText = String("      ");
  } else if (speed < 100) {
    speedText = String("   ");
  }
  speedText += String(speed);
  writeBigString((char*) speedText.c_str(), 0, 2);
}
// ********************************************************************************** //
//                                      SUBROUTINES
// ********************************************************************************** //
// writeBigChar: writes big character 'ch' to column x, row y; returns number of columns used by 'ch'
int writeBigChar(char ch, byte x, byte y) {
  if (ch < ' ' || ch > '_') return 0;               // If outside table range, do nothing
  nb=0;                                             // character byte counter 
  for (bc=0; bc<8; bc++) {                        
    bb[bc] = pgm_read_byte( &bigChars[ch-' '][bc] );  // read 8 bytes from PROGMEM
    if(bb[bc] != 0) nb++;
  }  
 
  bc=0;
  for (row = y; row < y+2; row++) {
    for (col = x; col < x+nb/2; col++ ) {
      lcd.setCursor(col, row);                      // move to position
      lcd.write(bb[bc++]);                          // write byte and increment to next
    }
//    lcd.setCursor(col, row);
//    lcd.write(' ');                                 // Write ' ' between letters
  }
  return nb/2-1;                                      // returns number of columns used by char
}

String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// writeBigString: writes out each letter of string
void writeBigString(char *str, byte x, byte y) {
  char c;
  while ((c = *str++))
  x += writeBigChar(c, x, y) + 1;
}
