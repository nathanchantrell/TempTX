/*
//--------------------------------------------------------------------------------------
// TempTX Wireless Temperature Sensor Node
// By Nathan Chantrell http://nathan.chantrell.net
// Based on emontx code by Glyn Hudson and Trystan Lea at openenergymonitor.org 
// and JeeLabs RFM12B library http://jeelabs.org/2009/02/10/rfm12b-library-for-arduino/ 
//
// GNU GPL V3
//--------------------------------------------------------------------------------------
*/

//JeeNodes librarys 
#include <Ports.h>
#include <RF12.h>
#include <avr/eeprom.h>
#include <util/crc16.h>  //cyclic redundancy check

ISR(WDT_vect) { Sleepy::watchdogEvent(); } 	 // interrupt handler: has to be defined because we're using the watchdog for low-power waiting

#define myNodeID 11         //in the range 1-30
#define network     210      //default network group (can be in the range 1-250). All nodes required to communigate together must be on the same network group
#define freq RF12_433MHZ     //Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.

// set the sync mode to 2 if the fuses are still the Arduino default
// mode 3 (full powerdown) can only be used with 258 CK startup fuses
#define RADIO_SYNC_MODE 2

#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

//--------------------------------------------------------------------------------------------------
// One wire temperature sensing
//--------------------------------------------------------------------------------------------------

#include <OneWire.h>
#include <DallasTemperature.h>

// Data pin is connected to Digital 5
#define ONE_WIRE_BUS 5

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
int numberOfDevices; // Number of temperature devices found

// arrays to hold device address
DeviceAddress tempDeviceAddress;

double temp[8];

//--------------------------------------------------------------------------------------------------
//Data Structure to be transmitted
//--------------------------------------------------------------------------------------------------

typedef struct {
  	  int temp1;		// One-wire temperature 1
  	  int supplyV;		// temptx voltage
	  int temp2;		// One-wire temperature 2
	  int temp3;		// One-wire temperature 3
} Payload;

Payload temptx;

//--------------------------------------------------------------------------------------------------
//SETUP
//--------------------------------------------------------------------------------------------------

void setup() {

  Serial.begin(9600);
  
  Serial.println("temptx Wireless Temperature Sensor");
  
//--------------------------------------------------------------------------------------------------
// RFM12B Initialize
//--------------------------------------------------------------------------------------------------
  randomSeed(analogRead(0));                //initiate random function from noise 
  rf12_initialize(myNodeID,freq,network);   //Initialize RFM12 with settings defined above 
  rf12_sleep(0);                             //Put the RFM12 to sleep - Note: This RF12 sleep interupt method might not be 100% repiable. Put RF to sleep: RFM12B module can be kept off while not used – saving roughly 15 mA
//------------------------------------------
  
  Serial.print("Node: "); 
  Serial.print(myNodeID); 
  Serial.print(" Freq: "); 
  Serial.print(freq); 
  Serial.print(" Network: "); 
  Serial.println(network);
  
  sensors.begin();
  
  numberOfDevices = sensors.getDeviceCount();
  for(int i=0;i<numberOfDevices; i++)
  {
    if (sensors.getAddress(tempDeviceAddress, i)) sensors.setResolution(tempDeviceAddress, 12);
  }
 
}

//--------------------------------------------------------------------------------------------------
//LOOP
//--------------------------------------------------------------------------------------------------

void loop() {
    
  sensors.requestTemperatures(); // Send the command to get temperatures

  for(int i=0;i<numberOfDevices; i++)
    {
      if(sensors.getAddress(tempDeviceAddress, i))
      {
        temp[i] = sensors.getTempC(tempDeviceAddress);
      }
    }
  
  temptx.supplyV = readVcc(); 
  temptx.temp1 = temp[0] * 100;
  temptx.temp2 = temp[1] * 100;
  temptx.temp3 = temp[2] * 100;

   
//--------------------------------------------------------------------------------------------------
// Send data via RF 
//--------------------------------------------------------------------------------------------------

  rfwrite() ;
  
//for debugging 
  Serial.print(temptx.supplyV); 
  Serial.print(' '); 
  Serial.print(temptx.temp1); 
  Serial.print(' '); 
  Serial.print(temptx.temp2); 
  Serial.print(' '); 
  Serial.println(temptx.temp3); 
  delay(10);

Sleepy::loseSomeTime(10000);      //JeeLabs power save function: enter low power mode and update Arduino millis 
//only be used with time ranges of 16..65000 milliseconds, and is not as accurate as when running normally.http://jeelabs.org/2010/10/18/tracking-time-in-your-sleep/
   
}

//--------------------------------------------------------------------------------------------------
// Send payload data via RF
//--------------------------------------------------------------------------------------------------
static void rfwrite(){
  rf12_sleep(-1);     //wake up RF module
  while (!rf12_canSend())
  rf12_recvDone();
  rf12_sendStart(0, &temptx, sizeof temptx); 
  rf12_sendWait(2);    //wait for RF to finish sending while in standby mode
  rf12_sleep(0);    //put RF module to sleep
}

//--------------------------------------------------------------------------------------------------
// Read current supply voltage
//--------------------------------------------------------------------------------------------------
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}
//--------------------------------------------------------------------------------------------------

