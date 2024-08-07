/*****************************************************
 * Date: abril 2022
 * Written by: McOrts
 * 
 * ***************************************************/
#include "LoRaWanMinimal_APP.h"
#include <CayenneLPP.h>//the library is needed https://github.com/ElectronicCats/CayenneLPP
#include "Arduino.h"
#include "settings.h"

int loops; // number of readings
float cycles; // number of read and transmisions cycles.
float icycles; // delta for cycles counter.
float wind_avg; // wind level average
unsigned int wind_peak; // wind peak value
unsigned int wind_min; // wind minimun value
unsigned int wind; // current wind value
unsigned long wind_sum; // wind level addition

/* Configuración sensor */
unsigned int WindSpeed ;      

unsigned long tmp_ini; 
long CountStart = 0;
long now_DutyCycle = 0;

CayenneLPP lpp(LORAWAN_APP_DATA_MAX_SIZE);//if use AT mode, don't modify this value or may run dead https://github.com/HelTecAutomation/CubeCell-Arduino/search?q=commissioning.h

///////////////////////////////////////////////////
//Some utilities for going into low power mode
TimerEvent_t sleepTimer;
//Records whether our sleep/low power timer expired
bool sleepTimerExpired;

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
  //Low power handler also gets interrupted by other timers
  //So wait until our timer had expired
  while (!sleepTimerExpired) lowPowerHandler();
  TimerStop( &sleepTimer );
}

///////////////////////////////////////////////////
void setup() {
	Serial.begin(115200);

  tmp_ini = millis(); 

  wind_avg = 0;
  wind_peak = 0;
  wind_min = 10;  
  wind_sum = 0;
  loops = 0;
  cycles = 50;
  icycles = 1;

  if (ACTIVE_REGION==LORAMAC_REGION_AU915) {
    //TTN uses sub-band 2 in AU915
    LoRaWAN.setSubBand2();
  }
 
  LoRaWAN.begin(LORAWAN_CLASS, ACTIVE_REGION);
  
  //Enable ADR
  LoRaWAN.setAdaptiveDR(true);

  while (1) {
    Serial.print("Joining... ");
    LoRaWAN.joinOTAA(appEui, appKey, devEui);
    if (!LoRaWAN.isJoined()) {
      //In this example we just loop until we're joined, but you could
      //also go and start doing other things and try again later
      Serial.println("JOIN FAILED! Sleeping for 60 seconds");
      lowPowerSleep(60000); 
    } else {
      Serial.println("JOINED");
      break;
    }
  }
}

void transmitRecord()
{
  /*
  * set LoraWan_RGB to Active,the RGB active in loraWan
  * RGB red means sending;
  * RGB purple means joined done;
  * RGB blue means RxWindow1;
  * RGB yellow means RxWindow2;
  * RGB green means received done;
  */
 
  // Cycles control
  if (cycles>99) {
    icycles = -1 ;
  } else if (cycles<1){
    icycles = +1 ;
  }
  cycles += icycles;
   
  // Cayenne
  lpp.reset();
  lpp.addAnalogInput(1,SensorId);
  lpp.addAnalogInput(2,cycles);
  lpp.addLuminosity(1, wind_avg);
  lpp.addLuminosity(2, wind_peak);
  lpp.addLuminosity(3, wind_min);  
  lpp.addGPS(2, latitude, longitude, alt);

  Serial.println("Transmiting...");
  Serial.print("lpp data size: ");
  Serial.print(lpp.getSize());
  Serial.println();
  
  // Request ACK only for the 3 firts transmision of a cycle
  bool requestack=cycles<3?true:false;
  
  // if (LoRaWAN.send(1, lpp.getBuffer(), lpp.getSize(), requestack)) {
  if (LoRaWAN.send(lpp.getSize(), lpp.getBuffer(), 2, requestack)) {
    Serial.println("Send OK");
  } else {
    Serial.println("Send FAILED");
  }
}
///////////////////////////////////////////////////
void loop()
{

  // Wind reading each second
  wind = (6 * analogRead(ADC) * (5.0 / 1023.0) - 33);
  if (wind > 4500) {
    Serial.println("outlier removed");
  } else {
    if (millis() - tmp_ini > 1000) {
      wind_sum += wind;
      loops ++;
      Serial.print("Wind speed: ");
      Serial.print(wind);
      Serial.println(" m/s");
      Serial.print(" loop: ");
      Serial.print(loops);
      Serial.print(" cycles: ");
      Serial.println(cycles);
      tmp_ini = millis(); 
    }
    if (wind > wind_peak) {
      wind_peak = wind;
      Serial.print("wind peak: ");
      Serial.println(wind_peak);
    }
    if (wind < wind_min) {
      wind_min = wind;
      Serial.print("wind min: ");
      Serial.println(wind_min);
    }
  }

  long now_DutyCycle = millis();
  if (now_DutyCycle - CountStart > DutyCycle) {
    CountStart = now_DutyCycle;

    // wind calculations
    wind_avg = int(wind_sum / loops);
    Serial.print("wind average: ");
    Serial.println(wind_avg);
    Serial.println(wind_sum);
    Serial.println(loops);

    transmitRecord();

    wind_peak = 0;
    wind_min = 1000;
    wind_sum = 0;
    loops = 0;
  
  }
  
}

///////////////////////////////////////////////////
//Example of handling downlink data
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  Serial.printf("Received downlink: %s, RXSIZE %d, PORT %d, DATA: ",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
  for(uint8_t i=0;i<mcpsIndication->BufferSize;i++) {
    Serial.printf("%02X",mcpsIndication->Buffer[i]);
  }
  Serial.println();
}