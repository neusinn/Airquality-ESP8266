# AirQuality
Measures temperature, humidity, CO2 and eTVOC

Author: Markus von Allmen, 2020

## Features
- measures Temperature [°C] 
- measures Humidity [%]
- measures CO2 [ppm] concentration
- measures eTVOC [ppb] concentration

## Hardware
- WeMos D1 mini PRO 32Mbit ESP8266
- HDC1080
- CCS811
- WS2812B - Neopixel 
- OLED 64x48 SSD1306

## Wireing:
- PIN D1 (GPIO05) : SCL 
- PIN D2 (GPIO04) : SDA
- PIN D5 (GPIO14) : NEOPIXEL

- CCS811 WAK to GND

## Sensor
### CCS811 gas digital sensor for air quality monitoring
- eCO2 detection range: 400 ~ 32768 ppm  - Equivalent Carbon Dioxide in parts per million. Clipped to 400 to 8192ppm.
- TVOC detection range:  0 ~ 32768 ppb  - Total Volatile Organic Compound in parts per billion.
- Automatic baseline correction time for metal oxide sensitive layer: 24H. Allow configuration and read sensor time after power up: at least 20 minutes
- Allow configuration and read sensor time after power up: at least 20 minutes
- Interface: IIC(I2C)
- library and information https://github.com/maarten-pennings/CCS811
- Application note for baseline saving: https://www.sciosense.com/wp-content/uploads/2020/01/Application-Note-Baseline-Save-and-Restore-on-CCS811.pdf

### HDC1080 Temperature and humidity sensor (only used für temperature and humidity compensation for CCS811)
- Temperature °C (-20°C bis ~ 70°C ( ±0.4°C (at 5°C -  ~ 60°C)
- Humidity % (0 ~ 100% RH ±2%)
- Humidity sensor response time: 15s
- Interface: IIC(I2C) Address: 1000000x (x read/write control bit)

### Display OLED mini Screen Size: 64x48 pixels (0.66” Across) (charcter size 8x6 -> 6 lines a 10 characters)
- Driver IC: SSD1306 Interface: IIC(I2C), IIC Address: 0x3C or 0x3D
- Interface: IIC(I2C), IIC Address: 0x3C or 0x3D
- PIN's D1: SCL, D2: SDA

# Airquality
## Klassifizierung der Raumluftqualität nach DIN EN 13779: 2007–09 (DIN 2007–09)
### CO2
- IDA1 : CO2 <  800 ppm      Hohe Raumlufqualität
- IDA2 : CO2 >  800-1000 ppm Mittlere Raumluftqualität
- IDA3 : CO2 > 1000-1400 ppm Mässige Raumluftqualität
- IDA4 : CO2 > 1400 ppm      Niedrige Raumluftqualtiät
- Übliche Werte Stadtzentrum ~400ppm, Land ~350ppm

### eTVOC
- TVOC <   300 ppb hygienisch unbedenklich
- TVOC <   300- 1000 ppb Hygienisch unbedenklich sofern keine Richtwertüberschreitung von Einzelstoffe
- TVOC <  1000- 3000 ppb Hygienisch auffällig  (höchstens 12 Monate)
- TVOC <  3000-10000 ppb Hygienisch bedenklich (höchstens 1 Monat)
- TVOC > 10000 ppb Hygienisch inakzepteable    (höchstens 1 Tag)
- TVOC > 25000 ppb Raumnutzung unterlassen

# Note
Der CCS811 misst die CO2 Konzentration. Der Sensor verändert seine Wert während des Betriebs und auch über die Zeit. 
- Beachten sollte man dass er ca 500 Stunden Burn-In Time benötigt. 
- Nache einem neu Einschalten benätigt er ca. 20 Minuten zum stabilisieren. 
- Es ist ein relativer Sensor. Er eicht sich anhand der geringsten CO2 Konzentration die er innerhalb von 24 Stunden misst.

