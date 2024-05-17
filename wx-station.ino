//// Logging temperature, relative humidity, wind, and rain ////
//// W.K. Petry
//// Sensors:
//// 1) Adafruit data logging shield w/ real-time clock & SD card (https://learn.adafruit.com/adafruit-data-logger-shield/using-the-real-time-clock-3)
//// 2) DHT22 temperature & relative humidity
//// 3) Argent Data Systems rain gauge (https://www.instructables.com/Arduino-Rain-Gauge-Calibration/; https://projects.raspberrypi.org/en/projects/build-your-own-weather-station/8)
//// 4) Calypso ULP-STD w/ UART protocol (https://docs.arduino.cc/learn/communication/uart/)
//// TODO:
//// - check that rain bucket is counting tips at all times, not just during the logging interval
//// - calibrate RTC
//// - calibrate rain gauge

//// PRELIMINARIES ////
#include <RTClib.h>                 // real-time clock

#include <Adafruit_Sensor.h>        // temp + rh support
#include <DHT.h>                    // temp + rh support
#include <DHT_U.h>                  // temp + rh support
#include <Wire.h>                   // I2C communications (rain gauge & SD card)
#include <SD.h>                     // SD card support
// Sensor pins //
#define PIN_DHT 4                   // DHT sensor (digital)
#define greenLEDpin 2               // green LED (digital)
#define redLEDpin 3                 // red LED (digital)
#define PIN_RAIN 7                  // rain gauage (digital)
#define PIN_WIND 0                  // wind UART (digital)
const int chipSelect = 10;          // digital pin for the SD cs line
// Logging settings //
#define LOG_INTERVAL 1000           // logging interval (milliseconds)
#define ECHO_TO_SERIAL 1            // echo data to serial port
#define SYNC_INTERVAL 60000         // write interval (milliseconds)
uint32_t syncTime = 0;              // time of last sync()
// Objects //
RTC_PCF8523 rtc;                    // real-time clock model
#define DHTTYPE DHT22               // temp + rh sensor model: DHT 22 (AM2302)
DHT_Unified dht(PIN_DHT, DHTTYPE);  // map pin to temp + rh
File logfile;                       // logging file
bool bucketPositionA = false;       // one of the two positions of tipping-bucket               
const double bucketTip = 0.2794;    // millimeters rain equivalent to trip tipping-bucket
double accumRain = 0.0;             // rain accumulated during the last LOG_INTERVAL period     
//bool first;                         // as we want readings of the (MHz) loops only at the 0th moment

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while(1);
}

//// SETUP ////
void setup() {
  // INIT: serial output //
  Serial.begin(9600);    // USB
  //Serial1.begin(38400);  // RX/TX pins
  Serial.println();
  // use debugging LEDs //
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  // INIT: SD card //
  Serial.print("Initializing SD card...");
  pinMode(10, OUTPUT);
  // check card is present and can be initialized //
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  // INIT real-time clock //
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }
  if (! rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // Note: allow 2 seconds after inserting battery or applying external power
    // without battery before calling adjust()
  }
  // clear STOP bit, if present, from RTC
  rtc.start();
  // calibrate PCF8523 per https://www.nxp.com/docs/en/application-note/AN11247.pdf
  float drift = 0; // seconds plus or minus over oservation period - set to 0 to cancel previous calibration.
  float period_sec = (7 * 86400);  // total obsevation period in seconds (86400 = seconds in 1 day:  7 days = (7 * 86400) seconds )
  float deviation_ppm = (drift / period_sec * 1000000); //  deviation in parts per million (μs)
  float drift_unit = 4.34; // use with offset mode PCF8523_TwoHours
  // float drift_unit = 4.069; //For corrections every min the drift_unit is 4.069 ppm (use with offset mode PCF8523_OneMinute)
  int offset = round(deviation_ppm / drift_unit);
  // rtc.calibrate(PCF8523_TwoHours, offset); // Un-comment to perform calibration once drift (seconds) and observation period (seconds) are correct
  // rtc.calibrate(PCF8523_TwoHours, 0); // Un-comment to cancel previous calibration
  Serial.print("RTC offset is "); Serial.println(offset); // Print to control offset
  // INIT file //
  char filename[] = "WXLOG000.CSV";
  for (uint8_t i = 0; i < 1000; i++) {
    filename[5] = i/100 + '0';
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  if (! logfile) {
    error("couldn't create file");
  }
  Serial.print("Logging to: "); Serial.println(filename);  // echo log filename to serial output
  logfile.println("millis,stamp,datetime,temp_degC,rh_perc,windspeed_mps,winddir_deg");  // column headers
  // INIT: DHT //
  dht.begin();
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Serial No:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Serial No:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // INIT: rain gauge //
  pinMode(PIN_RAIN, INPUT);                       // set the Rain Pin as input
  // INIT: anemometer //
  // {TBD}
  // INIT: serial monitor headers //
  #if ECHO_TO_SERIAL
    // Serial.println("millis,stamp,datetime,temp_degC,rh_perc,windspeed_mps,winddir_deg");
  #endif
}

//// Loop
void loop() {
  // READ: real-time clock //
  DateTime now = rtc.now();
  delay((LOG_INTERVAL - 1) - (millis() % LOG_INTERVAL));  // wait until logging interval
    // MAINT: LED logging in progress //
  digitalWrite(greenLEDpin, HIGH);
  // LOG: milliseconds since starting //
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(",");
  #if ECHO_TO_SERIAL
    Serial.print("ms: "); Serial.print(m); Serial.print(", ");         // milliseconds since start 
  #endif
  // LOG: timestamp //
  logfile.print(now.unixtime()); logfile.print(",");  // unix time
  logfile.print(now.year(), DEC); logfile.print("-");
  logfile.print(now.month(), DEC); logfile.print("-");
  logfile.print(now.day(), DEC); logfile.print(" ");
  logfile.print(now.hour(), DEC); logfile.print(":");
  logfile.print(now.minute(), DEC); logfile.print(":");
  logfile.print(now.second(), DEC); logfile.print(",");
  #if ECHO_TO_SERIAL
    Serial.print("unixtime: "); Serial.print(now.unixtime()); Serial.print(", ");
    Serial.print("timestamp: "); Serial.print(now.year(), DEC); Serial.print("-");
    Serial.print(now.month(), DEC); Serial.print("-");
    Serial.print(now.day(), DEC); Serial.print(" ");
    Serial.print(now.hour(), DEC); Serial.print(":");
    Serial.print(now.minute(), DEC); Serial.print(":");
    Serial.print(now.second(), DEC); Serial.print(", ");
  #endif 
  // LOG: temperature & relative humidity //
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    logfile.print("NA,");
    #if ECHO_TO_SERIAL
      Serial.print("temp_degC: "); Serial.print("NA, ");
    #endif
  }
  else {
    logfile.print(event.temperature); logfile.print(",");
    #if ECHO_TO_SERIAL
      Serial.print("temp_degC: "); Serial.print(event.temperature); Serial.print(", ");
    #endif
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    logfile.print("NA,");
    #if ECHO_TO_SERIAL
      Serial.print("rh_perc: "); Serial.print("NA, ");
    #endif
  }
  else {
    logfile.print(event.relative_humidity);
    logfile.print(",");
    #if ECHO_TO_SERIAL
      Serial.print("rh_perc: ");
      Serial.print(event.relative_humidity);
      Serial.print(", ");
    #endif
  }
  // LOG: rain //
  if ((bucketPositionA==false)&&(digitalRead(PIN_RAIN)==HIGH)){
    bucketPositionA=true;
    accumRain+=bucketTip;  // increment interval rain amount
  }
  if ((bucketPositionA==true)&&(digitalRead(PIN_RAIN)==LOW)){
    bucketPositionA=false;  
  }
  logfile.print(accumRain, 8); logfile.print(", ");  // maintain volumetric precision
  #if ECHO_TO_SERIAL
    Serial.print("rain_mm: "); Serial.print(accumRain, 8); Serial.print(", ");
  #endif
  accumRain = 0.0;
  // LOG: wind speed & direction //
  // if (Serial1.available()) {
  //   wind = Serial1.read();  // read the UART sentence from anemometer stream
  //   windspd = wind.substring(13, 16);
  //   logfile(windspd); logfile(",");
  //   winddir = wind.substring(7, 9);
  //   logfile(winddir); logfile(",");
  //   #if ECHO_TO_SERIAL
  //     Serial.print(windspd); Serial.print(", ");
  //     Serial.print(winddir); Serial.print(", ");
  //   #endif
  // }
  // MAINT: line feed //
  logfile.println();
  #if ECHO_TO_SERIAL
    Serial.println();
  #endif
  // MAINT: LED logging complete //
  digitalWrite(greenLEDpin, LOW);
  // WRITE: data to SD //
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  digitalWrite(redLEDpin, HIGH);  // LED write in progress
  logfile.flush();
  digitalWrite(redLEDpin, LOW);   // LED write complete
}