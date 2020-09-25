This is another altimeter for height measurement. Its purpose is to measure height and provide various other information such as pressure, humidity, temperature.
It has no LCD or display, all interaction are done through Wifi and Web access.
The first time, connect to the alimeter from  a PC or smartphone and search the  wifi network named ESP_ALTI. Connect to it and enter password  "12345678".
Do not forget to change the  Wifi network name AND password to your favorites.. 


Hardware
--------
* Wemos D1 Mini or D1 mini pro [Wemos D1](https://www.wemos.cc/en/latest/)
* BME/BMP 280 sensor  [BMP280 board](https://www.amazon.com/GY-BMP280-3-3-Precision-Atmospheric-Pressure-Barometric/dp/B087ZRGSC8/ref=sr_1_5?dchild=1&keywords=BMP280&qid=1601064022&sr=8-5)

Wiring
------
Hardware: Wemos D1 Mini  + BMP 280
* SCL -> D1
* SDA -> D2
* VCC -> 3.3v
* GND -> GND

Libraries
---------
It is based on the following libraries
* Wifi Accespoint and web server: [WifiAccessPont}](https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi)
* Pressure sensor [BMP280](https://github.com/embeddedadventures/BME280)
* Kalmann filter [Kalmann](https://github.com/denyssene/SimpleKalmanFilter)

Repository Contents
-------------------

* **alti_obo** - Main source file for the altimeter
* **README.md** - This file

Basic Usage
-----------
 * Power on the Altimeter
 * Search Wifi network named "ESP_ALTI"
 * connect and enter password "12345678"
 * From web browser, enter the address: "http://192.168.4.1" and hit <return>
 * The altimeter page should be displayed showing height, height max, altitude, pressure etc...
 * Refresh the web page as often as you need to get new datas

Additional Documentation
-------------------------

* **[Installing Additional Arduino Libraries](https://www.arduino.cc/en/Guide/Libraries)** - Basic information on how to install an Arduino library.


Version History
---------------

* [V 0.1.0](https://github.com/obor00/alti_obo) -- Initial commit


License Information
-------------------

This is an _**open source**_ project! 

Please review the LICENSE file for license information. 

If you have any questions or concerns on licensing, please contact olivier@obordes.com
