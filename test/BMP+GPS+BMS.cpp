#include "LoRaWanMinimal_APP.h"
#include "Arduino.h"
#include "Adafruit_BMP280.h"
#include "softSerial.h"
#include <NMEAGPS.h>
#include <GPSport.h>

TimerEvent_t sleepTimer;
uint8_t message[] = {0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00};
uint8_t appDataGPS[11], appDataTemp[3], appDataBMS[51], buffer[140] ;

bool sleepTimerExpired, BMSDatacollect;
int latitude, longitude, speed, temperature, bufferIndex, checksum_real, checksum, count, temp_buf, appDataSize = 0 ;

Adafruit_BMP280 bmp ;
NMEAGPS gps ;
gps_fix fix ;
softSerial mySerial(GPIO5,GPIO6) ;

uint8_t devEui[] = { 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x52, 0x04, 0x6C, 0xEF, 0xBE, 0x9D, 0x51, 0x70, 0x83, 0xAC, 0x69, 0x00, 0x71, 0x6D, 0xEE, 0x6C };

uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

void GPSRead() ;
void BMPRead() ;
void BMSRead() ;
void GPSPrepareframe() ;
void BMPPrepareframe() ;   
void BMSPrepareframe() ;                                                                                                                       

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
  mySerial.begin(19200) ;
  Wire.begin(SDA, SCL) ;
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
  count ++ ;
  if (count > 3600) {
    HW_Reset(0) ;
  }
  GPSRead() ;
  GPSPrepareframe() ;
  lowPowerSleep(5000) ;
  BMPRead() ;
  BMPPrepareframe() ;
  lowPowerSleep(5000) ;
  BMSRead() ;
  BMSPrepareframe() ;
  for (int i = 0 ; i < sizeof(appDataGPS) ; i ++) {
    appDataGPS[i] = 0 ;
  }
  lowPowerSleep(15000) ;
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
      DEBUG_PORT.print( speed ) ;
    }
    DEBUG_PORT.println() ;
  }
  delay(1000) ;
}

void BMPRead() {
  temperature = bmp.readTemperature() * 100 ;
  if (temperature > 10000) {
    temperature = temp_buf ;
  }
  temp_buf = temperature ;
  Serial.print(F("Temperature = "));
  Serial.print(temperature);
  Serial.println(F(" *C"));
}

void BMSRead() {
  bufferIndex = 0;  
  for (int i = 0; i < sizeof(message); i++) {
    mySerial.write(message[i]);
  }
  while (mySerial.available() > 0) {
      buffer[bufferIndex++] = mySerial.read();
  }
  int16_t checksum = 0 ;
  for (int i = 4 ; i < 138 ; i ++) {
    checksum += buffer[i] ;
  }
  for (int i = 0 ; i < 140 ; i ++) {
    Serial.print(buffer[i], HEX) ;
    Serial.print(", ") ;
  }
  Serial.println() ;
  checksum_real = (buffer[138] << 8) + buffer[139] ;
  Serial.print("Checksum Cal : ") ;
  Serial.println(checksum) ;
  Serial.print("Checksum Real : ") ;
  Serial.println(checksum_real) ;
  BMSDatacollect = false ;
  if ((buffer[0] == 170) && (buffer[1] == 85) && (buffer[2] == 170) && (buffer[3] == 255) && (checksum == checksum_real)) {
    BMSDatacollect = true ;
    Serial.print("Total Voltage : ") ;
    Serial.println(((buffer[4] << 8) + buffer[5]) * 0.1) ;
    for (int i = 0; i < 15; ++i) {
      Serial.print("Voltage Cell ") ;
      Serial.print(i + 1) ;
      Serial.print(" : ") ;
      Serial.println((((buffer[6+i*2] << 8) + buffer[7+i*2])) * 0.001);
    }

    Serial.print("Current : ") ;
    Serial.println((((((buffer[70] << 8) + buffer[71] << 8) + buffer[72]) << 8) + buffer[73]) * 0.1) ;

    Serial.print("SoC: ") ;
    Serial.println(buffer[74]);

    Serial.print("Temperature 1 : ") ;
    Serial.println((buffer[91] << 8) + buffer[92]) ;
    Serial.print("Temperature 2 : ") ;
    Serial.println((buffer[93] << 8) + buffer[94]) ;
    Serial.print("Temperature 3 : ") ;
    Serial.println((buffer[95] << 8) + buffer[96]) ;
    Serial.print("Temperature 4 : ") ;
    Serial.println((buffer[97] << 8) + buffer[98]) ;

    Serial.print("Power : ") ;
    Serial.println((((((buffer[111] << 8) + buffer[112] << 8) + buffer[113]) << 8) + buffer[114])) ;
  }
}

void GPSPrepareframe() {
  count ++ ;
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
  if (LoRaWAN.send(11,appDataGPS,2,true)) {
    Serial.println("Send OK");
  } else {
    Serial.println("Send FAILED");
  }
  delay(1000) ;
}

void BMPPrepareframe() {
  count ++ ;
  appDataSize = 0 ;
  appDataTemp[appDataSize++] = 0x02 ; // Mark for Temperature
  appDataTemp[appDataSize++] = temperature >> 8;
  appDataTemp[appDataSize++] = temperature ;
  if (LoRaWAN.send(3,appDataTemp,2,true)) {
    Serial.println("Send OK");
  } else {
    Serial.println("Send FAILED");
  }
  delay(1000) ;
}

void BMSPrepareframe() {
  if (BMSDatacollect == true) {
    appDataSize = 0 ;
    appDataBMS[appDataSize++] = 0x03; //Mark for BMS
	// Total Voltage
    appDataBMS[appDataSize++] = buffer[4];
    appDataBMS[appDataSize++] = buffer[5];
	// Voltage Cell 
	for (int i = 0 ; i < 15 ; i ++) {
		appDataBMS[appDataSize++] = buffer[6+i*2] ;
		appDataBMS[appDataSize++] = buffer[7+i*2] ;
	}
	// Current
    appDataBMS[appDataSize++] = buffer[70] ;
    appDataBMS[appDataSize++] = buffer[71] ;
    appDataBMS[appDataSize++] = buffer[72] ;
    appDataBMS[appDataSize++] = buffer[73] ;
	// SOC
	appDataBMS[appDataSize++] = buffer[74] ;
	// Temperature
	for (int j = 0 ; j < 4 ; j ++) {
		appDataBMS[appDataSize++] = buffer[91 + j*2] ;
		appDataBMS[appDataSize++] = buffer[92 + j*2] ;
	}
	// Power
	appDataBMS[appDataSize++] = buffer[111] ;
	appDataBMS[appDataSize++] = buffer[112] ;
	appDataBMS[appDataSize++] = buffer[113] ;
	appDataBMS[appDataSize++] = buffer[114] ;
  if (LoRaWAN.send(50,appDataBMS,2,true)) {
    Serial.println("Send OK");
  } else {
    Serial.println("Send FAILED");
    }
  }
  delay(1000) ;
}