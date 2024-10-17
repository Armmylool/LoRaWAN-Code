#include "LoRaWanMinimal_APP.h"
#include "Arduino.h"
#include "Adafruit_BMP280.h"
#include <NMEAGPS.h>
#include <GPSport.h>

int latitude, longitude, speed;
int GPSData[3] ;
NMEAGPS gps ;
gps_fix fix ;

void setup() {
  DEBUG_PORT.begin(9600);
  DEBUG_PORT.print( F("NMEAsimple.INO: started\n") );
  gpsPort.begin(9600);
}

void loop() {
  while (gps.available(gpsPort)) {
    DEBUG_PORT.println("AVAILABLE") ;
    fix = gps.read();

    DEBUG_PORT.print( F("Location: ") );
    if (fix.valid.location) {
      latitude = fix.latitudeL() / 10  ;
      longitude = fix.longitudeL() / 10  ;
      DEBUG_PORT.print( latitude);
      DEBUG_PORT.print( ',' );
      DEBUG_PORT.print( longitude);
    }
    DEBUG_PORT.print( F(", Speed: ") );   
    if (fix.valid.speed) {
      speed = fix.speed_kph() * 100 ;
      DEBUG_PORT.print( speed ) ;
    }
    DEBUG_PORT.println() ;
  }
  delay(1000) ;
}