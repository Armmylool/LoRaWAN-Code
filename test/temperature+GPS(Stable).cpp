#include "LoRaWanMinimal_APP.h"
#include "Arduino.h"
#include "Adafruit_BMP280.h"
#include <NMEAGPS.h>
#include <GPSport.h>

TimerEvent_t sleepTimer;
uint8_t appDataGPS[11], appDataTemp[3] ;
bool sleepTimerExpired;
int latitude, longitude, speed, temperature, appDataSize = 0 ;

Adafruit_BMP280 bmp ;
NMEAGPS gps ;
gps_fix fix ;

uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

void GPSRead() ;
void BMPRead() ;
void GPSPrepareframe() ;
void BMPPrepareframe() ;                                                                                                                          

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
  gpsPort.begin(9600);
  Wire.begin(SDA, SCL) ;
  digitalWrite(Vext, LOW) ;
if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different addreGPS!"));
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  LoRaWAN.begin(LORAWAN_CLASS, ACTIVE_REGION);
  LoRaWAN.setAdaptiveDR(true);
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
  for (int i = 0 ; i < sizeof(appDataGPS) ; i ++) {
    appDataGPS[i] = 0 ;
  }
  lowPowerSleep(5000) ;
  BMPRead() ;
  BMPPrepareframe() ;
  lowPowerSleep(5000) ;
}

void GPSRead() {
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
      DEBUG_PORT.print( speed );
    }
    DEBUG_PORT.println();
  }
  delay(1000) ;
}

void BMPRead() {
  temperature = bmp.readTemperature() * 100 ;
  Serial.print(F("Temperature = "));
  Serial.print(temperature);
  Serial.println(F(" *C"));
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
  if (LoRaWAN.send(11,appDataGPS,1,true)) {
    Serial.println("Send OK");
  } else {
    Serial.println("Send FAILED");
  }
}

void BMPPrepareframe() {
  appDataSize = 0 ;
  appDataTemp[appDataSize++] = 0x02 ; // Mark for Temperature
  appDataTemp[appDataSize++] = temperature >> 8;
  appDataTemp[appDataSize++] = temperature ;
  if (LoRaWAN.send(3,appDataTemp,1,true)) {
    Serial.println("Send OK");
  } else {
    Serial.println("Send FAILED");
  }
}