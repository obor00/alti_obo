// Alti OBO
// libraries used:
// WIFI and AP: https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/examples/WiFiAccessPoint/WiFiAccessPoint.ino
// BMP280: https://github.com/embeddedadventures/BME280
// Kalmann filter: https://github.com/denyssene/SimpleKalmanFilter/tree/master/examples/AltitudeKalmanFilterExample
//
// Hardware: Wemos D1 Mini  + BMP 280
//     SCL -> D1
//     SDA -> D2
//     VCC -> 3.3v
//     GND -> GND

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// ! reset  BMP :  power off/on
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#include <BME280_MOD-1022.h>
#include "avr/dtostrf.h"
#include <Wire.h>
#include <float.h>

#include <SimpleKalmanFilter.h>

#include <EEPROM.h>

#ifndef APSSID
#define APSSID "ESP_ALTI"
#define APPSK  "12345678"
#endif


/* Set these to your desired credentials. */
const char *ssid = APSSID;
const char *password = APPSK;

ESP8266WebServer server(80);

#define ALTI_K 10
unsigned long start_time = 0;
unsigned long cur_time = 0;
unsigned long delta_time = 0;

float temp, humidity,  pressure, pressureMoreAccurate;
double tempMostAccurate, humidityMostAccurate, pressureMostAccurate;
float altitudeMostAccurate, start_altitude;
float height_filtered = 0;
float height_max = FLT_MIN;
float height_min = FLT_MAX;
float eeprom_last_height;
#define ADDR_EEPROM_LAST_HEIGHT 0x10

#define WARMUP  100
#define DISCARD_FIRST 500
int discard_first = DISCARD_FIRST;

/*
   Constants for simple Kalmann filter
 */
#define k_e_mea  1  // e_mea: Measurement Uncertainty - How much do we expect to our measurement vary
#define k_e_est  1  // e_est: Estimation Uncertainty - Can be initilized with the same value as e_mea since the kalman filter will adjust its value.
#define k_q  0.01   // q: Process Variance - usually a small number between 0.001 and 1 - how fast your measurement moves. Recommended 0.01. Should be tunned to your needs

SimpleKalmanFilter pressureKalmanFilter(k_e_mea, k_e_est, k_q);

void getCompensatedMeasurements(void) 
{
	temp      = BME280.getTemperature();
	humidity  = BME280.getHumidity();
	pressure  = BME280.getPressure();

	pressureMoreAccurate = BME280.getPressureMoreAccurate();  // t_fine already calculated from getTemperaure() above

	tempMostAccurate     = BME280.getTemperatureMostAccurate();
	humidityMostAccurate = BME280.getHumidityMostAccurate();
	pressureMostAccurate = BME280.getPressureMostAccurate();
}

void compute()
{
	float _altitude = 44330.0 * (1.0 - pow(pressureMostAccurate / 1013.25 , 0.19026));

	altitudeMostAccurate = pressureKalmanFilter.updateEstimate(_altitude);

	if (discard_first > 0) {
		start_altitude = altitudeMostAccurate;
		discard_first--;
	}

	height_filtered = altitudeMostAccurate - start_altitude ;

	if (height_filtered > height_max) {
		height_max = height_filtered ; 
		EEPROM.put (ADDR_EEPROM_LAST_HEIGHT, height_max);
		EEPROM.commit();
	}
	else if (height_filtered < height_min) 
		height_min = height_filtered;

#if 0
	Serial.print (altitudeMostAccurate,2);
	Serial.print (",");
	Serial.print (start_altitude,2);
	Serial.print (",");
	Serial.print (height_filtered,2);
	Serial.println ("");
#endif
}

void handleRoot() 
{
	String page = String(F("<H1>Alti OBO</H1><H2>"));
	page +=  String(F("<br>Hauteur max = "));
	page +=  String(height_max,1);
	page +=  String(F(" m"));
	page +=  String(F("<br>Hauteur estimée = "));
	page +=  String(height_filtered,1);
	page +=  String(F(" m"));
	page +=  String(F("<br>Hauteur min = "));
	page +=  String(height_min,1);
	page +=  String(F(" m"));

	page +=  String(F("<br>Derniere hauteur max = "));
	page +=  String(eeprom_last_height,1);
	page +=  String(F(" m"));

	page +=  String(F("</H2><H3>"));
	page +=  String(F("<br>Temperature = "));
	page +=  String(tempMostAccurate,2);
	page +=  String(F(" C"));
	page +=  String(F("<br>humidity    = "));
	page +=  String(humidityMostAccurate,2);
	page +=  String(F(" %"));  
	page +=  String(F("<br>Pressure    = "));
	page +=  String(pressureMostAccurate,2);
	page +=  String(F(" mBar"));  
	page +=  String(F("<br>Altitude    = "));
	page +=  String(altitudeMostAccurate,2);
	page +=  String(F(" m"));  
	page +=  String(F("<br>temps entre mesures = "));
	page +=  String(delta_time,8);
	page +=  String(F(" ms"));
	page +=  String(F("<br>temps depuis poweron = "));
	page +=  String(millis()/1000,8);
	page +=  String(F(" s"));
	page +=  String(F("</H3>"));
	page +=  String(F("\r\n</html>\r\n"));

	server.send(200, "text/html; charset=utf-8", page);
}

void setup() 
{
	delay(1000);
	Serial.begin(115200);
	Serial.println();
	Serial.print("Configuring access point...");
	/* You can remove the password parameter if you want the AP to be open. */
	WiFi.softAP(ssid, password);

	IPAddress myIP = WiFi.softAPIP();
	Serial.print("AP IP address: ");
	Serial.println(myIP);
	server.on("/", handleRoot);
	server.begin();
	Serial.println("HTTP server started");

	Wire.begin();
	start_time = millis();

	// init bmp280
	uint8_t chipID;

	Serial.println("Welcome to ALTI Obo");
	chipID = BME280.readChipId();

	// find the chip ID out just for fun
	Serial.print("ChipID = 0x");
	Serial.print(chipID, HEX);


	// need to read the NVM compensation parameters
	BME280.readCompensationParams();

	BME280.writeStandbyTime(tsb_0p5ms);        // tsb = 0.5ms
	BME280.writeFilterCoefficient(fc_16);      // IIR Filter coefficient 16
	BME280.writeOversamplingPressure(os16x);    // pressure x16
	BME280.writeOversamplingTemperature(os2x);  // temperature x2
	BME280.writeOversamplingHumidity(os1x);     // humidity x1

	BME280.writeMode(smNormal);

	// warmup
	for (int i=0; i < WARMUP; i++) {
		delay (10);
		BME280.readMeasurements();
	}

	// EEPROM
	EEPROM.begin(512);
	EEPROM.get (ADDR_EEPROM_LAST_HEIGHT, eeprom_last_height);
}

void loop() 
{
	delay(10);  // implicit yield

	delta_time = millis() - cur_time;
	cur_time = millis();

	//while (BME280.isMeasuring()) ; 
	//while (BME280.doingIMUpdate()) ; 

	// read out the data 
	BME280.readMeasurements();
	getCompensatedMeasurements();
	compute();

	server.handleClient();
}
