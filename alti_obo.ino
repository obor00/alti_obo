// Alti OBO
// libraries used:
// WIFI and AP: https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/
// BMP280: https://github.com/embeddedadventures/BME280
// Kalmann filter: https://github.com/denyssene/SimpleKalmanFilter
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
// #include "avr/dtostrf.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <float.h>
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

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

float temp, pressure, altitude;
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

void compute()
{
	altitudeMostAccurate = pressureKalmanFilter.updateEstimate(altitude);

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

}

void handleRoot()
{
	String page = String(F("<H1>Alti OBO</H1><H2>"));
	page +=  String(F("<br>Hauteur max = "));
	page +=  String(height_max,2);
	page +=  String(F(" m"));
	page +=  String(F("<br>Hauteur courante = "));
	page +=  String(height_filtered,2);
	page +=  String(F(" m"));
	page +=  String(F("<br>Hauteur min = "));
	page +=  String(height_min,2);
	page +=  String(F(" m"));

	page +=  String(F("<br>Derniere hauteur max = "));
	page +=  String(eeprom_last_height,2);
	page +=  String(F(" m"));

	page +=  String(F("</H2><H3>"));
	page +=  String(F("<br>Altitude reference   = "));
	page +=  String(start_altitude,2);
	page +=  String(F(" m"));
	page +=  String(F("<br>Temperature = "));
	page +=  String(temp,2);
	page +=  String(F(" C"));
//	page +=  String(F("<br>Humidité    = "));
//	page +=  String(humidityMostAccurate,2);
//	page +=  String(F(" %"));
	page +=  String(F("<br>Pression   = "));
	page +=  String(pressure,2);
	page +=  String(F(" mBar"));
	page +=  String(F("<br>Altitude    = "));
	page +=  String(altitude,2);
	page +=  String(F(" m"));
	page +=  String(F("<br>Altitude corrigée   = "));
	page +=  String(altitudeMostAccurate,2);
	page +=  String(F(" m"));
	page +=  String(F("<br>Délai entre mesures = "));
	page +=  String(delta_time,8);
	page +=  String(F(" ms"));
	page +=  String(F("<br>Délai depuis poweron = "));
	page +=  String(millis()/1000,8);
	page +=  String(F(" s"));
	page +=  String(F("</H3>"));
	page +=  String(F("\r\n</html>\r\n"));

	server.send(200, "text/html; charset=utf-8", page);
}

void setup()
{
    bool status;

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

    TwoWire theWire;
    status = bme.begin(0x76 , &theWire);
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

    Serial.println();
    // indoor navigation
    Serial.println("-- Indoor Navigation Scenario --");
    Serial.println("normal mode, 16x pressure / 2x temperature / 1x humidity oversampling,");
    Serial.println("0.5ms standby period, filter 16x");
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
            Adafruit_BME280::SAMPLING_X2,  // temperature
            Adafruit_BME280::SAMPLING_X16, // pressure
            Adafruit_BME280::SAMPLING_X1,  // humidity
            Adafruit_BME280::FILTER_X16,
            Adafruit_BME280::STANDBY_MS_0_5 );

    start_time = millis();

    Serial.println("Welcome to ALTI Obo");


    // warmup
    // for (int i=0; i < WARMUP; i++) {
    // 	delay (10);
    // 	BME280.readMeasurements();
    // }

    // EEPROM
    EEPROM.begin(512);
    EEPROM.get (ADDR_EEPROM_LAST_HEIGHT, eeprom_last_height);

    while (1)
    {
        myloop();
    }
}

void myloop()
{
    delta_time = millis() - cur_time;
    cur_time = millis();

    temp = bme.readTemperature();
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    compute();

#ifdef TRACE
    Serial.print(temp);
    Serial.print(",");
    Serial.print(pressure);
    Serial.print(",");
    Serial.println(altitude);
#endif
    server.handleClient();
    delay(10);
}

void loop() {}
