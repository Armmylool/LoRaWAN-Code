#include "LoRaWanMinimal_APP.h"
#include "Arduino.h"
#include <NMEAGPS.h>
#include "Adafruit_BMP280.h"
#include <GPSport.h>

int latitude, longitude, speed, appDataSize = 0 ;
uint8_t appDataGPS[11] ;
NMEAGPS gps ;
gps_fix fix ;

uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

TimerEvent_t sleepTimer;

bool sleepTimerExpired;

void GPSRead() ;
void GPSPrepareframe() ;                                                                                                                          

static void wakeUp()
{
  sleepTimerExpired=true;
}

static void lowPowerSleep(uint32_t sleeptime)
{
  sleepTimerExpired=false;
  TimerInit( &sleepTimer, &wakeUp );
  TimerSetValue( &sleepTimer, sleeptime );
  TimerStart( &sleepTimer );
  while (!sleepTimerExpired) lowPowerHandler();
  TimerStop( &sleepTimer );
}

void setup() {
  Serial.begin(9600);
  LoRaWAN.begin(LORAWAN_CLASS, ACTIVE_REGION);
  LoRaWAN.setAdaptiveDR(true);
  gpsPort.begin(9600);
  Serial.print( F("NMEAsimple.INO: started\n") ) ;
  while (1) {
    Serial.print("Joining... ");
    LoRaWAN.joinOTAA(appEui, appKey, devEui);
    if (!LoRaWAN.isJoined()) {
      Serial.println("JOIN FAILED! Sleeping for 30 seconds");
      lowPowerSleep(30000);
    } else {
      Serial.println("JOINED");
      break;
    }
  }
}

void loop() {
  GPSRead() ;
  GPSPrepareframe() ;
  if (LoRaWAN.send(11,appDataGPS,1,true)) {
    Serial.println("Send OK");
  } else {
    Serial.println("Send FAILED");
  }
  for (int i = 0 ; i < sizeof(appDataGPS) ; i ++) {
    appDataGPS[i] = 0 ;
  }
  lowPowerSleep(5000) ;
}

void GPSRead() {
  while (gps.available(gpsPort)) {
    DEBUG_PORT.println("AVAILABLE") ;
    fix = gps.read();

    if (fix.valid.location) {
      latitude = fix.latitudeL() / 10  ;
      longitude = fix.longitudeL() / 10  ;
      DEBUG_PORT.print( F("Location: ") ) ;
      DEBUG_PORT.print( latitude);
      DEBUG_PORT.print( ',' );
      DEBUG_PORT.print( longitude);
    }

    if (fix.valid.speed) {
      speed = fix.speed_kph() * 100 ;
      DEBUG_PORT.print( F(", Speed: ") );
      DEBUG_PORT.print( speed );
    }
    DEBUG_PORT.println();
  }
  delay(1000) ;
}

void GPSPrepareframe() {
  appDataSize = 0 ;
  appDataGPS[appDataSize++] = 0x01 ; // Mark for GPS
  appDataGPS[appDataSize++] = speed >> 8;
  appDataGPS[appDataSize++] = speed ;
  appDataGPS[appDataSize++] = longitude >> 24;
  appDataGPS[appDataSize++] = longitude >> 16;
  appDataGPS[appDataSize++] = longitude >> 8;
  appDataGPS[appDataSize++] = longitude ;
  appDataGPS[appDataSize++] = latitude >> 24 ;
  appDataGPS[appDataSize++] = latitude >> 16 ;
  appDataGPS[appDataSize++] = latitude >> 8 ;
  appDataGPS[appDataSize++] = latitude ;
}