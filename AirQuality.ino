// ============================================================================================
// Wemos D1 Mini to Microsoft
// Author: Markus von Allmen, 2020
// ============================================================================================
// 
// Tested with Board Wemos D1 mini Pro - ESP8266
//
//  
// Features:
// - measures Temperature [°C] 
// - measures Humidity [%]
// - measures CO2 [ppm] concentration
// - measures eTVOC [ppb] concentration
// - Connect to local WiFi
// - Send Telemetry data to Microsoft Azure IOT Central
//
// Used hardware:
// - Wemos D1 pro V2 - ESP8266
// - CCS811
// - OLED 64x48 SSD1306
// - WS2812B - Neopixel 
//
// Pins:
//   PIN D1 (GPIO05) : SCL 
//   PIN D2 (GPIO04) : SDA
//   PIN D5 (GPIO14) : NEOPIXEL
//
//
// - CCS811 gas digital sensor for air quality monitoring
//   - eCO2 detection range: 400 ~ 32768 ppm  - Equivalent Carbon Dioxide in parts per million. Clipped to 400 to 8192ppm.
//   - TVOC detection range:  0 ~ 32768 ppb  - Total Volatile Organic Compound in parts per billion.
//   - Automatic baseline correction time for metal oxide sensitive layer: 24H. Allow configuration and read sensor time after power up: at least 20 minutes
//   - Allow configuration and read sensor time after power up: at least 20 minutes
//   - Interface: IIC(I2C)
//   library and information https://github.com/maarten-pennings/CCS811
//   Application note for baseline saving: https://www.sciosense.com/wp-content/uploads/2020/01/Application-Note-Baseline-Save-and-Restore-on-CCS811.pdf
//
// - HDC1080 Temperature and humidity sensor (only used für temperature and humidity compensation for CCS811)
//   - Temperature [°C] (-20°C bis ~ 70°C ( ±0.4°C (at 5°C -  ~ 60°C)
//   - Humidity [%] (0 ~ 100% RH ±2%)
//   - Humidity sensor response time: 15s
//   - Interface: IIC(I2C) Address: 1000000x (x read/write control bit)
//
// - Display OLED mini Screen Size: 64x48 pixels (0.66” Across) (charcter size 8x6 -> 6 lines a 10 characters)
//   - Driver IC: SSD1306 Interface: IIC(I2C), IIC Address: 0x3C or 0x3D
//   - Interface: IIC(I2C), IIC Address: 0x3C or 0x3D
//   - PIN's D1: SCL, D2: SDA
//
// Klassifizierung der Raumluftqualität nach DIN EN 13779: 2007–09 (DIN 2007–09)
//  - IDA1 : CO2 <  800 ppm      Hohe Raumlufqualität
//  - IDA2 : CO2 >  800-1000 ppm Mittlere Raumluftqualität
//  - IDA3 : CO2 > 1000-1400 ppm Mässige Raumluftqualität
//  - IDA4 : CO2 > 1400 ppm      Niedrige Raumluftqualtiät
//  - Übliche Werte Stadtzentrum ~400ppm, Land ~350ppm
const int co2_goodQualityLT = 800;
const int co2_mediumQualityLT = 1000;
const int co2_moderateQualityLT = 1400;
//const int co2_badQualtiyMT = moderateQualityLT;
//
//  
//  TVOC <   300 ppb hygienisch unbedenklich
//  TVOC <   300- 1000 ppb Hygienisch unbedenklich sofern keine Richtwertüberschreitung von Einzelstoffe
//  TVOC <  1000- 3000 ppb Hygienisch auffällig  (höchstens 12 Monate)
//  TVOC <  3000-10000 ppb Hygienisch bedenklich (höchstens 1 Monat)
//  TVOC > 10000 ppb Hygienisch inakzepteable    (höchstens 1 Tag)
//  TVOC > 25000 ppb Raumnutzung unterlassen
const int tvoc_safeQualityLT = 1000;
const int tvoc_consicuousQualtiyLT = 3000;
const int tvoc_worringLT = 10000;
//int tvoc_inacceptableMT =  tvoc_worringLT;
/*
hygienisch unbedenklich hygienisch auffällig hygienisch inakzeptabel
Grenzwerte VOC
<1 mg/m3 – unter 150 bis 400 ppb 
<1 bis 3 mg/m3 – 150 bis 1300 ppb 
>10 mg/m3 – über 1500 bis 4000 ppb
*/
//
//
//
// ============================================================================================

#include <Wire.h>
#include <SPI.h>


// --- Properties ------------------------------------------------------------
const char* deviceID = "m0006";
const char* deviceName = "AirQuality";
const char* deviceNote = "-for Lulu-";
const char* softwareVersion = "V1.0.r";
const int displayMode = 3; // [3=temp+hum+CO2; 4=temp+hum+CO2+eTVOC; 99=DEBUG; other = alternate]

#define RESET_EEPROM true

float temp = 0;
float humidity = 0;;
int co2 = 0;
int tvoc = 0;
float pressure = 0;


// --- Sensor -----------------------------------------------------------------
// --- Intervals --------------------------------------------------------------
unsigned long intervalReadSensor = 10 * 1000; // Interval read sensors in milli seconds
unsigned long lastReadSensor = 0;
unsigned long counterReadSensor = 0;

unsigned long intervalCalibrateSensor = 10 * 60 * 1000; // Interval read sensors in milli seconds
unsigned long lastCalibrateSensor = 0;

// Check 
unsigned long intervalSensorBaseline = 20 * 60 * 1000; // Interval read sensors in milli seconds - Wait 20 minutes for stabilisation before save baseline 
unsigned long lastSensorBaseline = 0;
unsigned long counterSensorBaseline = 0;
unsigned long maxIntervalSensorBaseline = 24 * 60 * 60 * 1000; // max interval for Baseline savaing - Save basline each 1-28 

// ---- CCS811 ----------------------------------------------------------------
#include "ccs811.h"  // CCS811 
#include <ESP_EEPROM.h>

// nWAKE not controlled via Arduino host, so connect CCS811.nWAKE to GND
CCS811 sensorCCS811; 
uint16_t baselineCCS811 = 0xFFFF;
uint16_t eepromBaselineCCS811;

bool isOKCCS811 = false;

int co2CCS811 = 400; // CO2 concentration from sensor CCS811
int tvocCCS811 = 0;  // tvoc concentration from sensor CCS811


// ---- HDC1080 ---------------------------------------------------------------
#include "ClosedCube_HDC1080.h"
ClosedCube_HDC1080 sensorHDC1080;

float tempHDC1080 = 0;       // Temperature from sensor HDC1080
float humidityHDC1080 = 0;   // Humidity form sensor HDC1080


// --- OLED -------------------------------------------------------------------
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//   PIN D1 (GPIO05) : SDL
//   PIN D2 (GPIO04) : SDA
#define OLED_RESET 0 // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_I2C_ADDRESS 0x3C
Adafruit_SSD1306 display(OLED_RESET);


// ---- WS2812B - Neopixel ----------------------------------------------------
// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  
// Avoid connecting on a live circuit...if you must, connect GND first.

#include <Adafruit_NeoPixel.h>  // https://github.com/adafruit/Adafruit_NeoPixel

// === PIN definitions
#define PIN_NEOPIXELS    14 // GPIO14 == D5 @ ESP8266)

// How many NeoPixels are attached in line
#define NUM_NEOPIXELS     1 // Numbers of Neopixels that are attached in line 

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(6, PIN_NEOPIXELS, NEO_GRB + NEO_KHZ800);


// Brightness
#define STRIP_BRIGHTNESS_MAX        255
#define STRIP_BRIGHTNESS_MED         48
#define STRIP_BRIGHTNESS_LOW         24

uint8_t stripBrightness = STRIP_BRIGHTNESS_MED;

// colors definitions
uint32_t COLOR_BLACK  = strip.Color(  0,   0,   0);
uint32_t COLOR_GREEN  = strip.gamma32(strip.Color(  0, 153,  51));
uint32_t COLOR_ORANGE = strip.gamma32(strip.Color(255, 153,   0));
uint32_t COLOR_YELLOW = strip.gamma32(strip.Color(255, 255,   0));
uint32_t COLOR_RED    = strip.gamma32(strip.Color(255,   0,   0));
uint32_t COLOR_MAGENTA= strip.gamma32(strip.Color(255,   0, 255));
uint32_t COLOR_BLUE   = strip.gamma32(strip.Color(  0,   0, 255));

uint32_t pixelColor = COLOR_BLACK;
bool pixelStatusOn = false;
unsigned long lastPixelToggle = 0;
unsigned int intervalPixelToggle = 250; // Interval LED blink milli seconds. 0 means no toggle


// ----------------------------------------------------------------------------


// ---- HDC1080 ---------------------------------------------------------------
void readSensorHDC1080TemperatureAndHumidity() {

  tempHDC1080 = sensorHDC1080.readTemperature(); 
  humidityHDC1080 = sensorHDC1080.readHumidity();
  
  Serial.print("HDC1080 :: "); Serial.print("T= "); Serial.print(tempHDC1080); Serial.print(" C, ");
  Serial.print("RH= "); Serial.print(humidityHDC1080); Serial.println(" %");
}


void printSensorHDC1080Register() {
  HDC1080_Registers reg = sensorHDC1080.readRegister();
  
  Serial.print(F("==> HDC1080 :: Measurement Resolution:")); 
  Serial.print("T="); Serial.print(reg.TemperatureMeasurementResolution, BIN); Serial.print(F(" (0=14 bit, 1=11 bit)"));
  Serial.print(" RH="); Serial.print(reg.HumidityMeasurementResolution, BIN); Serial.println(F(" (00=14 bit, 01=11 bit, 10=8 bit)"));
}


// ---- CCS811 ----------------------------------------------------------------

// Calibrate sensor for more accurate values
/* 
  bool CCS811::setEnvData(float fTemperature, float fHumidity) {
    Humidity is stored as an unsigned 16 bits in 1/512%RH. 
    The default value is 50% = 0x64, 0x00. As an example 48.5% humidity would be 0x61, 0x00.
  
    Temperature is stored as an unsigned 16 bits integer in 1/512 degrees; 
    there is an offset: 0 maps to -25°C. 
    The default value is 25°C = 0x64, 0x00. 
    As an example 23.5% temperature would be 0x61, 0x00.
    The internal algorithm uses these values (or default values if not set by the application) 
    to compensate for changes in relative humidity and ambient temperature.
  
    int h = (int)(fHumidity * 512.0); // convert humidity to 512th fractions
    int t = (int)((fTemperature  + 25.0) * 512.0); // offset of -25C
  
    uint8_t buf[] = { (uint8_t)(h >> 8), // high byte
                      (uint8_t)h,        // low byte
                      (uint8_t)(t >> 8), // high byte
                      (uint8_t)t};        // low byte
    wake_up();
    Serial.print("CCS811  :: Calibrate with humidity: 0x"); Serial.print(buf[0],HEX); Serial.print(buf[1],HEX);
    Serial.print(", temp: 0x"); Serial.print(buf[2],HEX); Serial.println(buf[3],HEX);
    bool ok = i2cwrite(CCS811_ENV_DATA, 4, buf);
    wake_down();
    return ok;
}

*/
void calibrateSensorCCS811(float temp, float humidity) {
  // Set compensation for humidity and temperature
  Serial.print(F("CCS811  :: Calibrate sensor: ")); Serial.print("T= "); Serial.print(temp); Serial.print(", H= "); Serial.println(humidity);
  
  // Validate temperatures and humidity values
  if ((temp < -25) || (temp > 50) || (humidity < 0) || (humidity > 100)) {
    Serial.print(F("Calibrate CCS811 :: Invalid values for callibration. Skip callibaration."));
    return;
  }
  
  sensorCCS811.setEnvData(temp, humidity); 

  // show baseline
  sensorCCS811.get_baseline(&baselineCCS811);
  Serial.print("CCS811  :: Sensor baseline: 0x"); Serial.println(baselineCCS811,HEX);
}


void readSensorCCS811() {
  uint16_t co2, tvoc, errorCode, raw;

  sensorCCS811.read(&co2,&tvoc,&errorCode,&raw); 
  
  // Print measurement results based on status
  if( errorCode == CCS811_ERRSTAT_OK ) { 
    co2CCS811 = co2;
    tvocCCS811 = tvoc;

    Serial.print("CCS811  :: ");
    Serial.print("CO2=");  Serial.print(co2CCS811);  Serial.print(" ppm, ");
    Serial.print("TVOC="); Serial.print(tvocCCS811); Serial.println(" ppb");

    //Serial.print("raw6=");  Serial.print(raw/1024); Serial.print(" uA  "); 
    //Serial.print("raw10="); Serial.print(raw%1024); Serial.print(" ADC  ");
    //Serial.print("R="); Serial.print((1650*1000L/1023)*(raw%1024)/(raw/1024)); Serial.print(" ohm");
    
  } else if( errorCode == CCS811_ERRSTAT_OK_NODATA ) {
    Serial.println("CCS811: waiting for (new) data");
  } else if( errorCode & CCS811_ERRSTAT_I2CFAIL ) { 
    Serial.println("==> ERROR CCS811: I2C error");
  } else {
    Serial.print("==> ERROR CCS811: errorCode="); Serial.print(errorCode, HEX); 
    Serial.print("="); Serial.println( sensorCCS811.errstat_str(errorCode) ); 
  }
}


// --- OLED Display -------------------------------------------------------------
void displayConfig() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.drawFastHLine(0, 0, 64 - 1, WHITE);
  display.setCursor(0, 2);
  display.println(deviceName);
  display.drawFastHLine(0, 11, 64 - 1, WHITE);
  display.setCursor(0, 13);
  display.println(deviceNote);
  display.setCursor(0, 24);
  display.print("ID="); display.println(deviceID);
  display.print("SW="); display.println(softwareVersion);
  display.display();
}


void displayDataAlternate() {
  if (counterReadSensor % 2 == 0) {
    displayTempHumidity();
  } else {
    displayCO2TVOC();
  }
}


void displayTempHumidity() {
  char strbuf[16];
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  snprintf(strbuf, 6, "%2.1fC", temp);
  display.println(strbuf);
  display.setCursor(0,24);
  snprintf(strbuf, 5, "%2.1f", humidity);
  display.print(strbuf);
  display.println("%");
  display.display();
}


void displayCO2TVOC() {
  char strbuf[8];
  
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("CO2");
  display.setCursor(0,8);
  display.setTextSize(2);
  snprintf(strbuf, 6, " %4i", co2);
  display.println(strbuf);
  display.setCursor(0,24);
  display.setTextSize(1);
  display.println("TVOC");
  display.setCursor(0,32);
  display.setTextSize(2);
  snprintf(strbuf, 6, "%5i", tvoc);
  display.println(strbuf);
  display.display();
}


void displayDataAll() {
  char strbuf[16];
   
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  snprintf(strbuf, 11, "T : %3.1f C", temp);
  display.println(strbuf);
  display.setCursor(0,11);
  snprintf(strbuf, 9, "H : %3.1f", humidity);
  display.print(strbuf);
  display.println(" %");
  display.setCursor(0,25);
  snprintf(strbuf, 11, "CO2 : %4i", co2);
  display.println(strbuf);
  display.setCursor(0,36);
  snprintf(strbuf, 11, "TVOC:%5i", tvoc);
  display.println(strbuf);  
  display.display();
}

void displayDataDebug() {
  char strbuf[16];
   
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  snprintf(strbuf, 11, "T : %3.1f C", temp);
  display.println(strbuf);
  snprintf(strbuf, 9, "H : %3.1f", humidity);
  display.print(strbuf);
  display.println(" %");
  snprintf(strbuf, 11, "CO2 : %4i", co2);
  display.println(strbuf);
  snprintf(strbuf, 11, "TVOC:%5i", tvoc);
  sensorCCS811.get_baseline(&baselineCCS811);
  snprintf(strbuf, 12, "BL: 0x%04X", baselineCCS811);
  display.println(strbuf);
  float timeSinceReboot = millis() / 1000 / 60 / 60;
  snprintf(strbuf, 12, "time : %3.1f", timeSinceReboot);
  display.println(strbuf);
  
     
  display.display();
}

void displayDataTempHumidityCO2() {
  char strbuf[16];
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  snprintf(strbuf, 6, "%2.1fC", temp);
  display.println(strbuf);
  snprintf(strbuf, 5, "%2.0f", humidity);
  display.print(strbuf);
  display.println("%");
  snprintf(strbuf, 6, " %4i", co2);
  display.println(strbuf);
  display.display();
}


void displayPixelColor(uint32_t color) {
  pixelColor = color;
  intervalPixelToggle = 0; // no toggle
  for(int i = 0 ; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void displayPixelBlink(uint32_t color, uint32_t toggleTime) {
  intervalPixelToggle = toggleTime;
  lastPixelToggle = millis();
  pixelColor = color;
  pixelStatusOn = true;
  for(int i = 0 ; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void displayPixel() {
  if (intervalPixelToggle == 0) return; // 0 means no change - no blink

  unsigned long ms = millis();
  if (ms - lastPixelToggle > intervalPixelToggle) {
    uint32_t color;
    lastPixelToggle = ms;

    if (pixelStatusOn) {
      pixelStatusOn = false;
      color = COLOR_BLACK;
    } else {
      pixelStatusOn = true;
      color = pixelColor;
    }
    
    for(int i = 0 ; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, color);
    }
    strip.show();
  }
}


// Klassifizierung der Raumluftqualität nach DIN EN 13779: 2007–09 (DIN 2007–09)
//  - IDA1 : CO2 <  800 ppm      Hohe Raumlufqualität
//  - IDA2 : CO2 >  800-1000 ppm Mittlere Raumluftqualität
//  - IDA3 : CO2 > 1000-1400 ppm Mässige Raumluftqualität
//  - IDA4 : CO2 > 1400 ppm      Niedrige Raumluftqualtiät
//  - Übliche Werte Stadtzentrum ~400ppm, Land ~350ppm
void setColorCO2(int value) {
uint32_t color;  

  if (value < co2_goodQualityLT) {
    color = COLOR_GREEN;
  } else if (value < co2_mediumQualityLT) {
      color = COLOR_YELLOW;
  } else if (value < co2_moderateQualityLT) {
      color = COLOR_ORANGE;
  } else {
      color = COLOR_RED;
  }
  
  displayPixelColor(color);
}



// ============================================================================
void setup() {
// ============================================================================

  Serial.begin(9600);
  Serial.println();
  
  Wire.begin(); //Inialize I2C Harware
  delay(1000);

  Serial.print("DeviceID  ="); Serial.println(deviceID);
  Serial.print("DeviceName="); Serial.println(deviceName);
  Serial.print("SW Version="); Serial.println(softwareVersion);
  
  EEPROM.begin(16);

  // DEBUG: Erase stored EEPROM baseline
  if ( RESET_EEPROM ) {
    uint16_t z = 0;
    EEPROM.put(0, z);
    boolean ok = EEPROM.commit();   // commit (write) the data to EEPROM - only actually writes if there has been a change
    Serial.print(F("EEPROM  :: Clear stored baseline ")); Serial.print(z,HEX); Serial.println((ok) ? " OK" : " FAILED");
  }
  
  EEPROM.get(0, eepromBaselineCCS811);
  Serial.print(F("EEPROM  :: Retrieved sensor baseline: 0x")); Serial.println(eepromBaselineCCS811,HEX);

  
  // --- Initalize OLED Display
  Serial.println(F("--> Initalizing OLED..."));
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);  // initialize with the I2C addr 0x3C (for the 64x48)

  displayConfig();

  
  // --- Initalize Neopixel
  Serial.println(F("--> Initalize Neopixel"));
  strip.begin(); // This initializes the NeoPixel library.
  
  strip.setBrightness(stripBrightness);
  displayPixelColor(COLOR_ORANGE);
  strip.show(); // Initialize all pixels to 'off' 
  


  // --- Initalize HDC1080
  Serial.println(F("--> Initalize HDC1080..."));
  // Default settings: Heater off; 14 bit Temperature and Humidity Measurement Resolutions
  // conversion time depends on resolution 2.5ms for 8-bit, 6.5ms for 14-bit
  sensorHDC1080.begin(0x40);
  delay(500);
  sensorHDC1080.setResolution(HDC1080_RESOLUTION_14BIT, HDC1080_RESOLUTION_14BIT);
  printSensorHDC1080Register();

  displayPixelColor(COLOR_YELLOW);


  // --- Initalize CS811 
  Serial.println(F("--> Initalize CCS811..."));
  display.print(F("CCS811:"));
  display.display();

  sensorCCS811.set_i2cdelay(50); // Needed for ESP8266 because it doesn't handle I2C clock stretch correctly
  isOKCCS811 = sensorCCS811.begin(); 
  if ( ! isOKCCS811) {
    Serial.println(F("==> WARN: CS811 sensor begin FAILED!"));
  } else {
      // Print CCS811 versions
    Serial.print(F("CCS811  :: hardware    version: ")); Serial.println(sensorCCS811.hardware_version(),HEX);
    Serial.print(F("CCS811  :: bootloader  version: ")); Serial.println(sensorCCS811.bootloader_version(),HEX);
    Serial.print(F("CCS811  :: application version: ")); Serial.println(sensorCCS811.application_version(),HEX);

    sensorCCS811.get_baseline(&baselineCCS811);
    Serial.print(F("CCS811  :: sensor baseline: 0x")); Serial.println(baselineCCS811,HEX);
  }

  isOKCCS811 = sensorCCS811.start(CCS811_MODE_1SEC);
  if ( isOKCCS811) {
    // wait for a while before calibrateSensorCCS811(sensorHDC1080.readTemperature(), sensorHDC1080.readHumidity());
    display.println("OK");
    display.display();
    displayPixelColor(COLOR_BLUE);
    
  } else {
    Serial.println(F("==> WARN: CS811 sensor start FAILED!"));
    Serial.println(F("======> ERROR: CS811 sensor FAILED to start!"));

    display.println("ERROR");
    display.display();
    displayPixelBlink(COLOR_MAGENTA, 250);
  }

  char strbuf[16];
  snprintf(strbuf, 12, "BL: 0x%04X", eepromBaselineCCS811);
  display.println(strbuf);
}



// ============================================================================
void loop() {
// ============================================================================

  // ms after starting. 
  // define as unsigned long in order to avoid rollover problems
  unsigned long ms = millis();

  if (ms - lastCalibrateSensor > intervalCalibrateSensor) { 
    lastCalibrateSensor = ms;
    
    readSensorHDC1080TemperatureAndHumidity();
    calibrateSensorCCS811(tempHDC1080, humidityHDC1080);
  }


  if (ms - lastReadSensor > intervalReadSensor) { 
    lastReadSensor = ms;
    counterReadSensor++;
    
    readSensorHDC1080TemperatureAndHumidity();
    readSensorCCS811();

    temp = tempHDC1080;
    humidity = humidityHDC1080;
    co2 = co2CCS811;
    tvoc = tvocCCS811;

    if ( isOKCCS811 ) {
      setColorCO2(co2CCS811);
    } else {
      displayPixelBlink(COLOR_MAGENTA, 250);
    }


    if (displayMode == 3) {
      displayDataTempHumidityCO2();
    } else if (displayMode == 4) {
      displayDataAll();
    } else if (displayMode == 99) {
      displayDataDebug();
    } else {
      displayDataAlternate();
    }
  }


  if (ms - lastSensorBaseline > intervalSensorBaseline) { 
    lastSensorBaseline = ms;
    counterSensorBaseline++;

    if ( intervalSensorBaseline < maxIntervalSensorBaseline) {
      intervalSensorBaseline = 2 * intervalSensorBaseline;
    }
    
    sensorCCS811.get_baseline(&baselineCCS811);
    Serial.print(F("CCS811  :: Sensor baseline: ")); Serial.println(baselineCCS811,HEX);

    if (counterSensorBaseline == 1 && eepromBaselineCCS811 != 0) {
      // after 20 minutes the sensor is stabilized and wie update the baseline for more accurate readings
      Serial.print(F("CCS811  :: Update sensor baseline with stored value: ")); Serial.println(eepromBaselineCCS811,HEX);
      sensorCCS811.set_baseline(eepromBaselineCCS811);
    
    } else {
      // Persists the latest and greatest baseline  
      EEPROM.put(0, baselineCCS811);
      boolean ok = EEPROM.commit();   // commit (write) the data to EEPROM - only actually writes if there has been a change
      Serial.print(F("EEPROM  :: Store sensor baseline ")); Serial.print(baselineCCS811,HEX); Serial.println((ok) ? " OK" : " FAILED");
    }

  }


  displayPixel();

}
