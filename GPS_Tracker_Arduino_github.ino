/*
 * Who: Simon Ries
 * What: Informatik Project - GPS Tracker with ESP32 
 * using:   ESP32
 *          Blynk App
 *          Beitian BN-880
 */



#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Blynk.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#define BLYNK_PRINT Serial

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;


TinyGPSPlus gps;  // output gps data

WidgetMap myMap(V0);   // declaring map to virtual pin v0 - Blynk App

SoftwareSerial ss(RXPin, TXPin); // the serial connection to the GPS device

BlynkTimer timer; 

float spd; // Variable to store the speed
float sats; // Variable to store the number of satellites
String Direction; // Variable to store the direction

char auth[] = ""; // insert project authentication key from Blynk App
char ssid[] = ""; // insert name of your network (HotSpot or Router name)                                            
char pass[] = ""; // insert password                                                             

unsigned  int move_index = 1; // fixing location


void setup()
{
  Serial.begin(115200); // set data rate to 115200
  ss.begin(GPSBaud);  
  Blynk.begin(auth, ssid, pass); // start Bylnk App 

  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  
  // Initialise the sensor 
  if(!mag.begin())                
  {
        Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
 
}


void loop()
{
    while (ss.available() > 0)  // RX & TX available -> output of received data 
    if (gps.encode(ss.read()))  
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10) // RX & TX not available
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
  Blynk.run();
  timer.run();
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    float latitude = (gps.location.lat()); // store Lat. & Lon.
    float longitude = (gps.location.lng());
    
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);

    Blynk.virtualWrite(V1, String(latitude, 6));    // output on Bylnk App
    Blynk.virtualWrite(V2, String(longitude, 6));   // output on Bylnk App
    myMap.location(move_index, latitude, longitude, "GPS_Location"); // with the help of lat. & lng the location is shown on the map
  }
  else
  {
    Serial.print(F("INVALID"));
  }

 
  Serial.print(F(" Speed: "));
  if (gps.speed.isValid())
  {
    Serial.print(gps.speed.kmph());
    Serial.print(F("km/h "));
    
    spd = gps.speed.kmph(); // output speed
    Blynk.virtualWrite(V3, spd);  // output on Bylnk App
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" Number of Satellites: "));
  if (gps.satellites.isValid())
  {
    Serial.print(gps.satellites.value());
    Serial.print(F(" "));

    sats = gps.satellites.value(); // output satellites
    Blynk.virtualWrite(V5, sats); // output on Bylnk App
  else
  {
    Serial.print(F("INVALID"));
  }

  // Get a new sensor event 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  float heading = atan2(event.magnetic.y, event.magnetic.x); 
  
  float declinationAngle = 0.05235987755983;      
  heading += declinationAngle;
  
  // if heading is negative
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // conversion radians to degrees
  float headingDegrees = heading * 180/M_PI; 

   
  Serial.print(" Heading (degrees): "); Serial.println(headingDegrees);
  
  if (headingDegrees == 0 || headingDegrees == 360 ) 
  {
  Serial.println("N");
  Direction = "N";
  }
  if (headingDegrees > 0 && headingDegrees < 90 ) 
  {
  Serial.print("N");
  // Degree from north towards east direction
  Serial.println("E");
  Direction = "NE";
  }
  if (headingDegrees == 90) 
  {
  Serial.println("E");
  Direction = "E";
  }
  if (headingDegrees > 90 && headingDegrees < 180 ) 
  {
  // Degree from south towards east direction
  headingDegrees = 180 - headingDegrees;
  Serial.print("S");
  Serial.println("E");
  Direction = "SE";
  }
  if (headingDegrees == 180) 
  {
  Serial.println("S");
  Direction = "S";
  }
  if (headingDegrees > 180 && headingDegrees < 270 ) 
  {
  // Degree from south towards west direction
  headingDegrees = headingDegrees - 180;
  Serial.print("S");
  Serial.println("W");
  Direction = "SW";
  }
  if (headingDegrees == 270) 
  {
  Serial.println("W");
  Direction = "W";
  }
  if (headingDegrees > 270 && headingDegrees < 360 ) 
  {
  // Degree from North towards west direction
  headingDegrees = 360 - headingDegrees;
  Serial.print("N");
  Serial.println("W");
  Direction = "NW";
  }
    Blynk.virtualWrite(V4, Direction); // output direction on Bylnk App
    delay(500);

  
}

 
  
