//--------------------------------------------------------------------------------------
// TempTX Wireless Temperature Sensor Node
// By Nathan Chantrell http://nathan.chantrell.net
//
// GNU GPL V3
//--------------------------------------------------------------------------------------

#include <OneWire.h> // http://www.pjrc.com/teensy/arduino_libraries/OneWire.zip
#include <DallasTemperature.h> // http://download.milesburton.com/Arduino/MaximTemperature/DallasTemperature_371Beta.zip
#include <JeeLib.h> // https://github.com/jcw/jeelib

//#define DEBUG // uncomment for serial monitor debug output

ISR(WDT_vect) { Sleepy::watchdogEvent(); } // interrupt handler for JeeLabs Sleepy power saving

#define myNodeID 14      // RF12 node ID in the range 1-30
#define network 210      // RF12 Network group
#define freq RF12_433MHZ // Frequency of RFM12B module

#define ONE_WIRE_BUS A5 // DS18B20 Temperature sensor is connected on pin A5
#define ONE_WIRE_POWER A4 // DS18B20 Power pin is connected on pin A4

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance

DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature

double temp; // Double precision variable for temperature reading

//########################################################################################################################
//Data Structure to be sent
//########################################################################################################################

 typedef struct {
  	  int temp;	// Temperature reading
  	  int supplyV;	// Supply voltage
 } Payload;

 Payload temptx;

//########################################################################################################################

void setup() {

  #ifdef DEBUG // Only run if DEBUG is defined above.
   Serial.begin(9600);
   Serial.println("temptx Wireless Temperature Sensor");
  #endif

  rf12_initialize(myNodeID,freq,network); // Initialize RFM12 with settings defined above 
  rf12_sleep(0);                          // Put the RFM12 to sleep

  pinMode(ONE_WIRE_POWER, OUTPUT); // set power pin for DS18B20 to output
  
}

void loop() {

  digitalWrite(ONE_WIRE_POWER, HIGH); // turn DS18B20 sensor on
  delay(1); // delay to allow sensor to turn on
  sensors.begin(); //start up temp sensor
  sensors.requestTemperatures(); // Get the temperature
  temp=(sensors.getTempCByIndex(0));
  digitalWrite(ONE_WIRE_POWER, LOW); // turn DS18B20 off
 
  temptx.temp = temp * 100; // Convert temperature to an integer, reversed at receiving end
  
  temptx.supplyV = readVcc(); // Get supply voltage

  rfwrite(); // Send data via RF 
  
 //for debugging 
  #ifdef DEBUG 
   Serial.println(" "); 
   Serial.print("Voltage: "); 
   Serial.println(temptx.supplyV); 
   Serial.print("Temperature: "); 
   Serial.println(temptx.temp); 
   delay(500);
  #endif

  Sleepy::loseSomeTime(60000);  //JeeLabs power save function: enter low power mode for 60 seconds (valid range 16-65000 ms)

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


