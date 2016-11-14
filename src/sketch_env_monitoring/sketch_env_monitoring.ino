// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Arduino Software Version v1.6.12
// Arduino Board UNO  
//
// Sensors:
//  1. LCD (I2C 4x20)
//  2. PMS3003 (PM2.5 / PM1.0 / PM10.0)
//  3. DHT 溫濕度 (DHT22)         [Ref: DHT22: DHT.read22(); DHT11: DHT:read11()]
//  4. DS1302 Clock (Optional)    [#define ENABLE_CLOCK_DS1302 1]   <<Clock & SD Card Module must use together>>
//  5. SD Card Module (Optional)  [#define ENABLE_SD_CARD 1]        <<Clock & SD Card Module must use together>>
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//  Program Specification 
//    1. setup()
//        initialize all sensors（LCD, PMS3003, DHT, Clock, SD Card)        
//    2. loop()
//        a. Read data from DHT     [readDHT()]
//        b. Read data from PMS3003 [readPMS3003V2()]
//        c. Display data on LCD    [displayDataOnLCD()]
//
//    Problem Solving History: 
//      Arduino: If you got "Low Memory Available" error when compiling this program, comment out all Serial.println("xxxx") code.
//      DS1302 : If it always return 2000:01:01 00:00:00, set the time again.
//                rtc.writeProtect(false);
//                Time t(2016, 11, 12, 19, 49, 00, Time::kSaturday); //年 月 日 時 分 秒 星期幾 (日期時間設定成功後即可註解掉)
//                rtc.time(t);//設定日期時間 (日期時間設定成功後即可註解掉)
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------

// ------------------------------------------
//  Software Configuration (BEGIN)
// ------------------------------------------
#define SERIAL_BAUD_RATE 9600
#define WELCOME_MSG_FIRST_LINE "Amber"
#define WELCOME_MSG_SECOND_LINE "v1.0 Loading......"
#define READING_SENSOR_INTERVAL 10000     // Interval to read ALL sensors. 10s interval.
#define ENABLE_CLOCK_DS1302 1             // 1: Enable Clock, 0; Disable Clock.
#define DISPLAY_CLOCK_DS1302_ON_LED 1     // 1: Display time on LED; 0: Dont display.
#define ENABLE_SD_CARD 1                  // 1: Enable SD Card, 0; Disable.
#define SD_CARD_FILENAME "MONITOR.TXT"    // Filename "FAT" 8.3 format (XXXXXXXX.XXX) You must use the 8.3 format, so that file names look like “NAME001.EXT”, where “NAME001” is an 8 character or fewer string, and “EXT” is a 3 character extension.
                                          // SD Card must be formatted as the "FAT" filesystem first. 
// ------------------------------------------
//  Software Configuration (END)
// ------------------------------------------


// ------------------------------------------
//  PIN Configuration (BEGIN)
// ------------------------------------------
// DHT 溫濕度 PIN 2:
#define DHT_PIN 2 

// LCD Display PIN:
//  LCD     Arduino(UNO)
//  VCC     5V
//  GND     GND
//  SDA     SDA(Pin A4)     (Reference:Arduino Mega2560:Pin 21)
//  SCL     SCL(Pin A5)     (Reference:Arduino Mega2560:Pin 20)

// PMS3003 (PM2.5 / PM1.0 / PM10.0) PIN:
#define PMS3003_RX_PIN 4 
#define PMS3003_TX_PIN 5 
//  PMS3003     Arduino(UNO)
//  PIN1        5V
//  PIN2        GND
//  PIN4        TX Pin 5  (Reference:Arduino Mega2560:18 [Serial1])
//  PIN5        RX Pin 4  (Reference:Arduino Mega2560:19 [Serial1])  

// Clock PIN:
#define CLOCK_RST_PIN 8 //RST
#define CLOCK_DAT_PIN 6  //DAT  
#define CLOCK_CLK_PIN 7  //CLK
//  PMS3003     Arduino(UNO)
//  RST         PIN10
//  DAT         PIN8
//  CLK         PIN9  

// SD Card PIN:
#define SD_CARD_CS_PIN 10  // CS
//  SD Card Module     Arduino(UNO)
//  CS                 PIN10
//  MOSI               PIN11   <-- No Configuration for this library
//  MISO               PIN12   <-- No Configuration for this library 
//  CLK                PIN13   <-- No Configuration for this library 
// ------------------------------------------
//  PIN Configuration (END)
// ------------------------------------------


// LCD Display Configuration (BEGIN)
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define LCD_DIGITS_PER_LINE 20  // No. of Characters per LINE
#define LCD_LINES 4  // LCD Lines
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address  (This LCD address: 0x3F)
//LiquidCrystal_I2C lcd(0x27, DIGITS_PER_LINE, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display
// LCD Display Configuration (END)

// PMS3003 Configuration (BEGIN)
// http://www.icshop.com.tw/product_info.php/products_id/20460
//首先，它可以採樣測定的空氣懸浮微粒有三種規格 0.3-1.0um/1.0-2.5um/2.5-10um，
//也就是說我們可以拿到 PM1.0/PM2.5/PM10 的測定資料(ug/m3)。
//而且PMS3003 的 datasheet 寫到他有兩套檢定空氣品質濃度的方法，
//分別是可以獲得「大氣環境下」和「標準顆粒物」兩組資料值，所以程式裡面每一次從 sensor 那邊得到的資料就會有2組，6個測定值。（這裡我會在意的是「大氣環境下」測得的這組）
#include <SoftwareSerial.h>
SoftwareSerial PMS3003Serial(PMS3003_RX_PIN, PMS3003_TX_PIN); // RX, TX
int pmcf10 = 0; // PM1.0  標準顆粒物
int pmcf25 = 0; // PM2.5  標準顆粒物
int pmcf100 = 0; // PM10.0 標準顆粒物
int pmat10 = 0; // PM1.0  大氣環境下
int pmat25 = 0; // PM2.5  大氣環境下
int pmat100 = 0; // PM10.0 大氣環境下
#define PMS3003_DATALEN 32
uint8_t PMS3003_buf[PMS3003_DATALEN];
//#define PMS3003_DISPLAY_DURATION 10000  // The duration to display PM2.5 message on LCD
// PMS3003 Configuration (BEGIN)

// DHT 溫度和濕度 Configuration (BEGIN)
#include <dht.h>   
#define DHT_INIT_WAIT 1000 
dht DHT;   
float temperature=0;
float humidity=0;
// DHT Configuration (BEGIN)

// Clock Configuration (BEGIN)
//#include <stdio.h>
#include <DS1302.h> // https://github.com/msparks/arduino-ds1302
DS1302 clockDS1302(CLOCK_RST_PIN, CLOCK_DAT_PIN, CLOCK_CLK_PIN);
boolean isClockOK = false;
// Clock Configuration (END)


// SD Card Module Configuration (BEGIN)
//#include <SPI.h>
//#include <SD.h>

//#include <SPI.h>    
//#include "SdFat.h"
//SdFat sd;
//SdFile sdCardFile;
#include <SD.h>
File sdCardFile;
boolean isSuccesToInitSD = false;  // true: Succes to initialize SD Card.
#define SD_FILE_DATA_HEADER "YYYY/MM/DD HH:MN:SS\tTEMP\tHUM\tPM2.5\tPM10\tPM1.0"
// SD Card Module Configuration (END)

// Software variables(BEGIN)
#define FIRST_LOOP_DELAY 2000  // delay seconds to wait for PMS3003 sensor on first-loop.
boolean firstLoop = true; 
boolean writeSDHeader = false;
#define TIME_STR_SIZE 20
#define TEMPERATURE_STR_SIZE 5
#define HUMIDITY_STR_SIZE 5
char timeStr[TIME_STR_SIZE];
char temperatureStr[TEMPERATURE_STR_SIZE];
char humidityStr[HUMIDITY_STR_SIZE];
// Software variables (BEGIN)

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                    FUNCTIONS
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  
  initClock(); // initialize Clock
  initSDCard(); // initialzie SD Card Module
  
  initLCD(); // initialize LCD
  displayLCD(WELCOME_MSG_FIRST_LINE, WELCOME_MSG_SECOND_LINE, "","");
 
  initDHT(); // initialize DHT

  initPMS3003(); // initialize PMS3003
  firstLoop = true;
}


void loop() { // put your main code here, to run repeatedly:

  readDHT(); // read DHT sensor, put it into variables ( temperature, humidity ).
 
  readPMS3003V2(); // read PM2.5 sensor. put it into variables (pmcf10, .., pmat100).
  
  displayDataOnLCD();  // Display message on LCD.

  writeToSDCard();
  
  delay(READING_SENSOR_INTERVAL);
  firstLoop = false;
}


void initLCD(){
  lcd.begin(LCD_DIGITS_PER_LINE, LCD_LINES); // initialize the LCD
  lcd.backlight(); // Turn on the blacklight and print a message.
  //Serial.println("LCD OK");
}

void initDHT(){
  delay(DHT_INIT_WAIT);  //Wait rest of 1000ms recommended delay before accessing DHT sensor   
  //Serial.println("DHT OK");
}

void initPMS3003(){
  PMS3003Serial.begin(SERIAL_BAUD_RATE);
  //Serial.println("PMS3003 OK");
}

void initClock(){
  if(ENABLE_CLOCK_DS1302){
    clockDS1302.writeProtect(true); // 是否防止寫入 (日期時間設定成功後即可改成true)
    clockDS1302.halt(false);        // 是否停止計時
   // calibrate time function.
   // Time t(2016, 7, 7, 21, 31, 00, Time::kThursday); //年 月 日 時 分 秒 星期幾 (日期時間設定成功後即可註解掉)
   // clockDS1302.time(t);//設定日期時間 (日期時間設定成功後即可註解掉)
    //Serial.println("Clock OK");
    
    Time t = clockDS1302.time();
    if (t.yr==2000){
      isClockOK = false; //If it always return 2000:01:01 00:00:00, set the time again.
    } else {
      isClockOK = true;
    }
  }
}

void initSDCard(){
  if ( ENABLE_SD_CARD ){
    //if (!sd.begin(SD_CARD_CS_PIN, SPI_HALF_SPEED)) {
    if (!SD.begin(SD_CARD_CS_PIN)) {
      //sd.initErrorHalt(); //SdFat specified
      isSuccesToInitSD = false;
      //Serial.println("SD Card initialization failed!");
      return;
    } else {
      isSuccesToInitSD = true;
      //Serial.println("SD OK");
    }
  }
}


void writeToSDCard(){
  if ( ENABLE_SD_CARD && isSuccesToInitSD && !firstLoop){  // 2016.11.11: Rule of Thumb: dont log first data, which it is often inaccurate.

    //if ( sdCardFile.open(SD_CARD_FILENAME, O_RDWR | O_CREAT | O_AT_END) ) {
    sdCardFile = SD.open(SD_CARD_FILENAME, FILE_WRITE);
    if ( sdCardFile ) { 
      //Serial.println(String("Writing to ") + SD_CARD_FILENAME);
      Time currentTime = readClock();   
      memset(timeStr, ' ', TIME_STR_SIZE);
      snprintf(timeStr, sizeof(timeStr), "%04d/%02d/%02d %02d:%02d:%02d", currentTime.yr, currentTime.mon, currentTime.date, currentTime.hr, currentTime.min, currentTime.sec); // timeStr[20]
      memset(temperatureStr, ' ', TEMPERATURE_STR_SIZE);
      memset(humidityStr, ' ', HUMIDITY_STR_SIZE);
      dtostrf(DHT.temperature, 2, 1, temperatureStr);  //format the display string (2 digits, 1 dp)  // temperatureStr[4]
      dtostrf(DHT.humidity, 2, 1, humidityStr);  // humidityStr[4]
      if (!writeSDHeader){
        sdCardFile.println(SD_FILE_DATA_HEADER);  // Write Header
        writeSDHeader = true;
      }
      // Modify SD_FILE_DATA_HEADER, if modify this data format
      if (pmat25==0 && pmat100==0 && pmat10==0) { // 2016.11.11 (pmat25==0 && pmat100==0 && pmat10==0) means PMS3003 did not provide good data.
        sdCardFile.println(timeStr + String("\t") +temperatureStr + "\t" +humidityStr);  
      } else {
        sdCardFile.println(timeStr + String("\t") +temperatureStr + "\t" +humidityStr + "\t" + pmat25 + "\t" + pmat100 + "\t" +pmat10);  
      }
      sdCardFile.close();   // flush(), if keep opening file, use flush() to write data immediately.
    } else {
       //Serial.println("ERR: Cannot open ENV_MONITORING.txt...");
    }
  }
}

// ENABLE_CLOCK_DS1302 == 1 in order to use this function.
// Return Time object
Time readClock(){
//  if(!ENABLE_CLOCK_DS1302){
//    Serial.println("ERR: ENABLE_CLOCK_DS1302 must set to 1"); // Use readClock(), ENABLE_CLOCK_DS1302 must set to 1
//  }
  Time t = clockDS1302.time();
//  char buf[50];  
//  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
//           t.yr, t.mon, t.date,
//           t.hr, t.min, t.sec);
//  Serial.println(buf);
  return t;
}

void readDHT(){
  // reset the variables value
  temperature = 0; // DHT Temperature
  humidity = 0; // DHT Humidity
  
  DHT.read22(DHT_PIN);   //去library裡面找DHT.read22
  temperature = DHT.temperature;
  humidity = DHT.humidity;
  //delay(100);// 2016.11.11
}

void readPMS3003V2(){
  // Reset the values.
  pmcf10 = 0; // PMS3003 PM1.0  標準顆粒物.
  pmcf25 = 0; // PMS3003 PM2.5  標準顆粒物.
  pmcf100 = 0; // PMS3003 PM10.0 標準顆粒物.
  pmat10 = 0; // PMS3003 PM1.0  大氣環境下.
  pmat25 = 0; // PMS3003 PM2.5  大氣環境下.
  pmat100 = 0; // PMS3003 PM10.0 大氣環境下.
  
  if (firstLoop) {  
    delay(FIRST_LOOP_DELAY); // Wait for 1s, otherwise PMS3003Serial is not available to read in the first loop.
  }
  memset(PMS3003_buf, 0, PMS3003_DATALEN);
  int idx = 0;
  const int MAX_INDEX = 23;
  while (PMS3003Serial.available() && idx<=MAX_INDEX)
  {
      PMS3003_buf[idx++] = PMS3003Serial.read();
  }
  while (PMS3003Serial.available()) PMS3003Serial.read(); // Clear all the buffer read. Must do this here, otherwise it may cause "Check Failed" problem.
  
  if (PMS3003_buf[0] == 0x42 && PMS3003_buf[1] == 0x4d)
  {

    int checksum =   (PMS3003_buf[22]<<8) + PMS3003_buf[MAX_INDEX];
    int checkvalue = (PMS3003_buf[0]+PMS3003_buf[1]+PMS3003_buf[2]+PMS3003_buf[3]+PMS3003_buf[4]+PMS3003_buf[5]+PMS3003_buf[6]+PMS3003_buf[7]+PMS3003_buf[8]+PMS3003_buf[9]+PMS3003_buf[10]+
                      PMS3003_buf[11]+PMS3003_buf[12]+PMS3003_buf[13]+PMS3003_buf[14]+PMS3003_buf[15]+PMS3003_buf[16]+PMS3003_buf[17]+PMS3003_buf[18]+PMS3003_buf[19]+PMS3003_buf[20]+PMS3003_buf[21]);

    if ( checksum == checkvalue ) {
      pmat25  = ( PMS3003_buf[12] << 8 ) | PMS3003_buf[13];  // PMS3003 PM2.5  大氣環境下.
      pmat10  = ( PMS3003_buf[10] << 8 ) | PMS3003_buf[11];
      pmat100 = ( PMS3003_buf[14] << 8 ) | PMS3003_buf[15];
      
      pmcf25  = ( PMS3003_buf[6] << 8 ) | PMS3003_buf[7];   // PMS3003 PM2.5  標準顆粒物.
      pmcf10  = ( PMS3003_buf[4] << 8 ) | PMS3003_buf[5];
      pmcf100 = ( PMS3003_buf[8] << 8 ) | PMS3003_buf[9];
    } 
//    else {
//      Serial.println("Checksum failed.");
//    }
//    Serial.println(String("[1]PM2.5:")+pm25_v2 + " PM1.0:" + pm1a_v2 + " PM10.0:" + pm10_v2);
//    Serial.println(String("[2]PM2.5:")+pm2a_v2 + " PM1.0:" + pm1a_v2 + " PM10.0:" + pm10a_v2);
//    Serial.println(String("Checksum:")+checksum);
//    Serial.println(String("Checksum[1]:") + checkvalue);
  } 
//  else{
//    Serial.println("check failed");
//  }
}



//void readPMS3003(){
//  // Reset the values.
//  pmcf10 = 0; // PMS3003 PM1.0  標準顆粒物.
//  pmcf25 = 0; // PMS3003 PM2.5  標準顆粒物.
//  pmcf100 = 0; // PMS3003 PM10.0 標準顆粒物.
//  pmat10 = 0; // PMS3003 PM1.0  大氣環境下.
//  pmat25 = 0; // PMS3003 PM2.5  大氣環境下.
//  pmat100 = 0; // PMS3003 PM10.0 大氣環境下.
//
//  if (firstLoop) {  
//    delay(FIRST_LOOP_DELAY); // Wait for 1s, otherwise PMS3003Serial is not available to read in the first loop.
//  }
//  
//  // Show PMS3003
//  int count = 0;
//  unsigned char c;
//  unsigned char high;
//  
//  while (PMS3003Serial.available()) {
//    c = PMS3003Serial.read();
//    if ((count == 0 && c != 0x42) || (count == 1 && c != 0x4d)) {
//     // Serial.println("check failed");
//      break;
//    }
//    if (count > 15) {
//      //Serial.println("complete");
//      //showPMS3003onLCD();
//      break;
//    }
//    else if (count == 4 || count == 6 || count == 8 || count == 10 || count == 12 || count == 14) {
//      high = c;
//    }
//    else if (count == 5) {
//      pmcf10 = 256 * high + c;
//      //Serial.print("CF=1, PM1.0=");
//      //Serial.print(pmcf10);
//      //Serial.println(" ug/m3");
//    }
//    else if (count == 7) {
//      pmcf25 = 256 * high + c;
//      //Serial.print("CF=1, PM2.5=");
//      //Serial.print(pmcf25);
//      //Serial.println(" ug/m3"); // 微克/立方公尺
//    }
//    else if (count == 9) {
//      pmcf100 = 256 * high + c;
//      //Serial.print("CF=1, PM10=");
//      //Serial.print(pmcf100);
//      //Serial.println(" ug/m3");
//    }
//    else if (count == 11) {
//      pmat10 = 256 * high + c;
//      //Serial.print("atmosphere, PM1.0=");
//      //Serial.print(pmat10);
//      //Serial.println(" ug/m3");
//    }
//    else if (count == 13) {
//      pmat25 = 256 * high + c;
//     // Serial.print("atmosphere, PM2.5=");
//    //  Serial.print(pmat25);
//    //  Serial.println(" ug/m3");
//    }
//    else if (count == 15) {
//      pmat100 = 256 * high + c;
//     // Serial.print("atmosphere, PM10=");
//     // Serial.print(pmat100);
//     // Serial.println(" ug/m3");
//    }
//    count++;
//  }
//  while (PMS3003Serial.available()) PMS3003Serial.read(); // Must do this here, otherwise it may cause "Check Failed" problem.
//  // Serial.println();
//  // delay(PMS3003_DISPLAY_DURATION);
//
//}


void displayDataOnLCD(){
  String firstLine;
  firstLine = WELCOME_MSG_FIRST_LINE;
  if ( ENABLE_CLOCK_DS1302 && !isClockOK ) {
    firstLine = "Clock Failed.";
  } else if ( ENABLE_SD_CARD && !isSuccesToInitSD ) {
    firstLine = "SD Failed.";
  } else {
    firstLine = WELCOME_MSG_FIRST_LINE;
  }
  if(ENABLE_CLOCK_DS1302 && DISPLAY_CLOCK_DS1302_ON_LED && isSuccesToInitSD && (!ENABLE_CLOCK_DS1302 || (ENABLE_CLOCK_DS1302 && isClockOK)) ) {
    Time clockTime = readClock();
    memset(timeStr, ' ', TIME_STR_SIZE);
    //snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",clockTime.yr, clockTime.mon, clockTime.date, clockTime.hr, clockTime.min, clockTime.sec);
    snprintf(timeStr, sizeof(timeStr), " %02d/%02d %02d:%02d",clockTime.mon, clockTime.date, clockTime.hr, clockTime.min);
    firstLine += timeStr;
  }
  memset(temperatureStr, ' ', TEMPERATURE_STR_SIZE);
  memset(humidityStr, ' ', HUMIDITY_STR_SIZE);
  dtostrf(DHT.temperature, 2, 1, temperatureStr);  //format the display string (2 digits, 1 dp)
  dtostrf(DHT.humidity, 2, 1, humidityStr); 
  String secondLine ="";
  secondLine = String("TMP:") +  temperatureStr + "c " + String("HUM:") + humidityStr + "%";
  
  String thirdLine = "";
  String fourthLine = "";
  if (pmat25!=0 && pmat10!=0 && pmat100!=0 ) {
    thirdLine =  String("PM2.5: ") +  pmat25 + " ug/m3";
    fourthLine = String("PM10 : ") +  pmat100 + " PM1: " + pmat10;
  } else {
    thirdLine = "Reading PM2.5";
  }
  displayLCD(firstLine, secondLine, thirdLine, fourthLine);
}



// firstLine: LCD First Line
// secondLine: LCD Second Line
// thirdLine: LCD Third Line
// fourthLine: LCD Fourth Line
void displayLCD(String firstLine, String secondLine, String thirdLine, String fourthLine) {
 // Serial.println("LCD<BEGIN>");
//  Serial.println(firstLine);
//  Serial.println(secondLine);
//  Serial.println(thirdLine);
//  Serial.println(fourthLine);
//  Serial.println("LCD<END>");
  lcd.clear();
  if (firstLine.length()>0) {
    lcd.setCursor(0, 0);
    lcd.print(firstLine);
  }
  if ( secondLine.length()>0) {
    lcd.setCursor(0, 1);
    lcd.print(secondLine);
  }
  if ( thirdLine.length()>0) {
    lcd.setCursor(0, 2);
    lcd.print(thirdLine);
  }
  if ( fourthLine.length()>0) {
    lcd.setCursor(0, 3);
    lcd.print(fourthLine);
  }
}


