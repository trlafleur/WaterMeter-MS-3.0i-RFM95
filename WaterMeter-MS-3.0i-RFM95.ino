

/*  
 *  Water flow and pressure meter
 *  
 *  
 *  Water flow from a Dwyer 1gal/pulse meter
 *  We measure the period of the pulse to determine the short term flow rate
 *    We also count the pulses to obtain the total gallons used
 *    We filter the input to debounce the reed switch in the meter the best we can, with a cutoff of ~80 gal/min
 *   
 *  Water from a pressure transducer, 0 to 5v
 *    Input fron sensor is scaled by .6577, 5.0v * .6577 = 3.288v
 *    (10.2k and a 19.6k resistor, flitered with a .1uf cap)
 *   
 *  Output: 0.5V – 4.5V linear voltage output. 0 psi outputs 0.5V, 50 psi outputs 2.5V, 100 psi outputs 4.5V 
 *    0   psi = 0.33v after scalling 5.0v to 3.3v
 *    50  psi = 1.65v
 *    100 psi = 2.97v
 *    3.3v/1024 = .0032266 volt per bit
 *    
 *    TX Led was on D3, now use for pulse-in on rev 2a board
 *      TX Led moved to D17
 *
 *    Max6816 is used as an option on Rev3a board to de-bounce the reed switch on the meter
 *    The switch must be stable for 40ms to get an output, this limits the max
 *    rate this device can support to less than 15Hz or so.
 *
 * CHANGE LOG:
 *
 *  DATE         REV  DESCRIPTION
 *  -----------  ---  ----------------------------------------------------------
 *  06-May-2016 1.2c  TRL - Change of Network ID
 *  09-May-2016 1.2d  TRL - added ACK's to send for testing RFM69_ATC driver
 *                          added real frequency set
 *  03-Dec-2016 2.0f  TRL - Moved to MySensor 2.1b and RFM95 radio
 *  04-Dec-2016 3.0g  TRL - Added Si7021
 *  14-Dec-2016 3.0h  TRL - Setup for rev 3a board and MCP9800
 *  19-Dec-2016 3.0i  TRL - added 1hr flow and flow alarm, set temp to F
 *  30-Dec-2016 3.0j  TRL - Change freq to 928.5MHz
 *
 *  Notes:  1)  Tested with Arduino 1.8.0
 *          2)  Testing using Moteino LR Rev4 with RFM95
 *          3)  Sensor 1 board, Rev2a and Rev3a
 *          4)  MySensor 2.1 30 Dec 2016
 *    
 *  Some information on the water meter and flow rated
 *         Gal/per/min            period
 *  ---------------------------------------------------
 *  1   Pulse = 1   gal     60  sec    .01667 Hz
 *  60  Pulse = 60  gal     1   sec    1 Hz
 *  30  Pulse = 30  gal     2   sec    .5 Hz
 *  10  Pulse = 10  gal     6   sec    .16667 Hz
 *  5   Pulse = 5   gal     12  sec    .08333 Hz
 *  2   Pulse = 2   gal     2   sec    .03333 Hz
 *  .5  Pulse = .5  gal     120 sec    .00833 Hz
 *
 *    So, we will set the range to be .5gpm to 60gpm
 *    
 *    MCP9800   base I2C address 0x40
 *    Si7021    base I2C address 0x48
 *    
 *    TODO:
 *    
 *    
 */

/* ************************************************************************************** */
#include <Arduino.h>
#include <Wire.h>

#include "i2c_SI7021.h"
SI7021 si7021;

#include <MCP980X.h>      // http://github.com/JChristensen/MCP980X
MCP980X MCP9800(0);

#include <avr/wdt.h>      // for watch dog timer support

/* ************************************************************************************** */
// Define the board options
#define R2A                 // Rev 2a of the board
//#define R3A                   // Rev 3a of the board
#define Sensor_SI7021       // Using the Si7021 Temp and Humidity sensor
//#define Sensor_MCP9800        // Using the MCP9800 temp sensor


/* ************************************************************************************** */
// Most of these items below need to be prior to #include <MySensor.h> 

/*  Enable debug prints to serial monitor on port 0 */
//#define MY_DEBUG            // used by MySensor
//#define MY_SPECIAL_DEBUG
//#define MY_DEBUG_VERBOSE_RFM95 
//#define MY_DEBUG1           // used in this program, level 1 debug
//#define MY_DEBUG2           // used in this program, level 2 debug

#define SKETCHNAME      "Water and Pressure Meter"
#define SKETCHVERSION   "3.0j"

/* ************************************************************************************** */
// Enable and select radio type attached, coding rate and frequency
#define MY_RADIO_RFM95

/*  
 *   Pre-defined MySensor radio config's
 *   
 * | CONFIG           | REG_1D | REG_1E | REG_26 | BW    | CR  | SF   | Comment
 * |------------------|--------|--------|--------|-------|-----|------|-----------------------------
 * | BW125CR45SF128   | 0x72   | 0x74   | 0x04   | 125   | 4/5 | 128  | Default, medium range SF7
 * | BW500CR45SF128   | 0x92   | 0x74   | 0x04   | 500   | 4/5 | 128  | Fast, short range     SF7
 * | BW31_25CR48SF512 | 0x48   | 0x94   | 0x04   | 31.25 | 4/8 | 512  | Slow, long range      SF9
 * | BW125CR48SF4096  | 0x78   | 0xC4   | 0x0C   | 125   | 4/8 | 4096 | Slow, long range      SF12
 */
 
#define MY_RFM95_MODEM_CONFIGRUATION    RFM95_BW125CR45SF128
#define MY_RFM95_TX_POWER               23 // max is 23
//#define MY_RFM95_ATC_MODE_DISABLED
#define MY_RFM95_ATC_TARGET_RSSI        (-60)
#define MY_RFM95_FREQUENCY              (928.5f)


/* ************************************************************************************** */
// Select correct defaults for the processor we are using
#ifdef __AVR_ATmega1284P__        // use for Moteino Mega Note: LED on Mega are 1 = on, 0 = off
#warning Using MoteinoMega
// MoteinoMEGA)
//#define MY_RFM95_RST_PIN        RFM95_RST_PIN
#define MY_RFM95_IRQ_PIN          2   // IRQ
#define MY_RFM95_SPI_CS           4   // NSS
#define MY_DEFAULT_TX_LED_PIN     12  // external LED's
#define MY_DEFAULT_ERR_LED_PIN    13
#define MY_DEFAULT_RX_LED_PIN     14
#define MoteinoLED                15
#define MY_WITH_LEDS_BLINKING_INVERSE


#elif  defined (__AVR_ATmega328P__) &&  defined (R3A)
#warning Using Moteino R3a
// Moteino 
//#define MY_RFM95_RST_PIN          RFM95_RST_PIN
#define MY_RFM95_IRQ_PIN            2   // IRQ
#define MY_RFM95_SPI_CS             10  // NSS
#define MY_DEFAULT_TX_LED_PIN       5   // 17 on 2a PCB as Tx   5 on 3a
#define MY_DEFAULT_ERR_LED_PIN      7   // 7  on 2a PCB as Err  7 on 3a
#define MY_DEFAULT_RX_LED_PIN       6   // 6  on 2a PCB as Rx   6 on 3a
#define MoteinoLED                  9
#define LED3                        4
#define MY_WITH_LEDS_BLINKING_INVERSE

#define ID0                         17  // ID0 pin
#define ID1                         9
//#define ID2                         


#elif  defined (__AVR_ATmega328P__) &&  defined (R2A)
#warning Using Moteino R2a
// Moteino 
//#define MY_RFM95_RST_PIN          RFM95_RST_PIN
#define MY_RFM95_IRQ_PIN            2   // IRQ
#define MY_RFM95_SPI_CS             10  // NSS
#define MY_DEFAULT_TX_LED_PIN       17  // 17 on 2a PCB as Tx   5 on 3a
#define MY_DEFAULT_ERR_LED_PIN      7   // 7  on 2a PCB as Err  7 on 3a
#define MY_DEFAULT_RX_LED_PIN       6   // 6  on 2a PCB as Rx   6 on 3a
#define MoteinoLED                  9
#define LED3                        4
#define MY_WITH_LEDS_BLINKING_INVERSE


#else
  #error Processor not defined
#endif


/* ************************************************************************************** */

#define MY_REPEATER_FEATURE

/* ************************************************************************************** */
// Set node defaults
#define NodeID_Base          15         // My Node ID base... this plus IDx bits if Rev3
int8_t myNodeID;
#define MY_NODE_ID myNodeID             // Set at run time from jumpers on rev 3a PCB
#define MY_PARENT_NODE_ID     0         // GW ID
#define CHILD_ID              1         // Id of my Water sensor child
#define CHILD_ID2             2         // Id of my 2nd sensor child


/* ************************************************************************************** */
/* These are use for local debug of code, hwDebugPrint is defined in MyHwATMega328.cpp */
#ifdef MY_DEBUG1
#define debug1(x,...) hwDebugPrint(x, ##__VA_ARGS__)
#else
#define debug1(x,...)
#endif

#ifdef MY_DEBUG2
#define debug2(x,...) hwDebugPrint(x, ##__VA_ARGS__)
#else
#define debug2(x,...)
#endif


/* ************************************************************************************** */
// All #define above need to be prior to #include <MySensors.h> below
#include <MySensors.h>
/* ************************************************************************************** */


#define DIGITAL_INPUT_SENSOR 3                    // The digital input you attached your sensor.  (Only 2 and 3 generates interrupt!)
#define SENSOR_INTERRUPT DIGITAL_INPUT_SENSOR-2   // Usually the interrupt = pin -2 (on uno/nano anyway)
#define PressPin      A0                          // Pressure sensor is on analog input A0, 0 to 100psi

#define PULSE_FACTOR  1                           // Nummber of pulse per gal of your meter (One rotation/gal)
#define MAX_FLOW      80                          // Max flow (gal/min) value to report. This filters outliers.


/* ************************************************************************************** */
MyMessage flowMsg         (CHILD_ID,V_FLOW);      // 34 0x22      // Send's current flow rate       
MyMessage volumeMsg       (CHILD_ID,V_VOLUME);    // 35 0x23      // Send's Volume for last interval
MyMessage lastCounterMsg  (CHILD_ID,V_VAR1);      // 24 0x18      // Send's Total Volume (Pulse Count)
MyMessage VAR2Msg         (CHILD_ID,V_VAR2);      // 25 0x19      // Send's Total Volume for last 24 hr's
MyMessage VAR3Msg         (CHILD_ID,V_VAR3);      // 26 0x1A      // Send's Total Volume for last hour
MyMessage VAR4Msg         (CHILD_ID,V_VAR4);      // 27 0x1B      // Send's 
MyMessage pressureMsg     (CHILD_ID,V_PRESSURE);  // 04 0x04      // Send's current Water Pressure
MyMessage TextMsg         (CHILD_ID,V_TEXT);      // 47 0x2F      // Send's status Messages
MyMessage HumMsg          (CHILD_ID,V_HUM);       // 01 0x01      // Send's current Humidity 
MyMessage TempMsg         (CHILD_ID,V_TEMP);      // 00 0x00      // Send's current Temperature

/* *************************** Forward Declaration ************************************* */
void SendPressure();
void onPulse();
void Send24Hr ();
void Send1Hr ();
void CheckExcessiveFlow();
void SendKeepAlive();
void getTempSi7021();
void getTempMCP9800 ();
void receive(const MyMessage &message);
void hexdump(unsigned char *buffer, unsigned long index, unsigned long width);

/* ************************************************************************************** */
unsigned long SEND_FREQUENCY        = 15000;      // Minimum time between send (in milliseconds). We don't want to spam the gateway.
unsigned long FLOW_RESET_FREQUENCY  = 180000;     // Time to reset flow to zero if no pulse 121000 = 2 min + 1sec
unsigned long KEEPALIVE_FREQUENCY   = 600000;     // Send Keep Alive message at this rate 600,000 = 10 min
unsigned long DAY_FREQUENCY         = 86400000;   // 24 hr in ms
unsigned long HOUR_FREQUENCY        = 3600000;    // 1 hr in ms
unsigned long ContFlowRateFREQUENCY = 7200000;    // this is the time interval to check and see if we have excessive flow. 7200000 = 2 hr

double ppg = ((double) PULSE_FACTOR) ;            // Pulses per gal 

// Uses in interrupt routine, so shoud be volatile
volatile unsigned long pulseCount     = 0;        // count of water meter pulses
volatile unsigned long lastPulseTime  = 0;        // time in us of last pulse, used to check for zero flow
volatile unsigned long newPulse       = 0;        // time in us of current pulse   
volatile unsigned long PulseInterval  = 0;        // delta time in us from last pulse to current pulse
volatile double flow                  = 0;        // curent flow rate 

boolean pcReceived  = false;                      // true, if we have receive old pulse count from controler
unsigned long oldPulseCount           = 0;
unsigned long old1HRPulseCount        = 0;
unsigned long old24HRPulseCount       = 0;

double oldflow          = 0;
double volume           = 0;                     
double oldvolume        = 0;
boolean volumeFlag      = false;
double flow_pulseCount  = 0;

double continuousFlowRate  = 1.0;                 // this is the threshold flow rate for a flow alarm
boolean flowFlag = false;                         // this is used to tell us if we have a continuous flow rate over threshold
unsigned long ContFlowRateTime = 0;

unsigned long currentTime         = 0;
unsigned long lastSendTime        = 0;
unsigned long keepaliveTime       = 0;
unsigned long flowresetTime       = 0;
unsigned long last24HRTime        = 0;
unsigned long last1HRTime         = 0;           

int pressure      = 0;                            // Current value from ATD
float PSI         = 0;                            // Current PSI
float PSI_CAL     = 2.0;                          // Calibration of sensor

int floatMSB      = 0;                            // used to convert float to int for printing
int floatR        = 0;

static float humi, temp;                          // used by Si7021, MCP9800


/* ************************************************************************************** */
/* A function to print a buffer in hex format
 *
 *  hexdump (buffer, length, 16);
 */
//#include <stdio.h>
//#include <stdlib.h>
//// debug1(PSTR(" %s \n"), compile_file);
// void hexdump(unsigned char *buffer, unsigned long index, unsigned long width)
// {
//    unsigned long i;
//    for (i=0; i<index; i++)
//    {
//      debug1(PSTR("%02x "), buffer[i]); 
//    }
//    
//    for (unsigned long spacer=index;spacer<width;spacer++)
//      debug1(PSTR("  ")); 
//      debug1(PSTR(": "));
//      for (i=0; i<index; i++)
//      {
//        if (buffer[i] < 32) debug(PSTR("."));
//        else debug(PSTR("%c"), buffer[i]);
//      }
//    debug1(PSTR("\n")); 
// }

 
/* **************************************************************************** */
/* ************************** Before ****************************************** */
/* **************************************************************************** */
 // this is part of MySensor core 
void before() 
{ 
  // this looks like the best place to start the WDT, but MySensor may take too long to start
     debug1(PSTR("***In before\n"));
 
#if defined R2A
      myNodeID = NodeID_Base;
#elif defined R3A
 // need to set up pins prior to reading them...
     pinMode(ID0, INPUT_PULLUP);
     pinMode(ID1, INPUT_PULLUP);
     myNodeID  = !digitalRead(ID0);                     // ID bit are 0 = on, so we invert them
     myNodeID |= !digitalRead(ID1) << 1;
     myNodeID += NodeID_Base; 
#else
#error ID Not defined
#endif
}


/* **************************************************************************** */
/* ************************** Setup ******************************************* */
/* **************************************************************************** */
void setup()  
{  
    wdt_enable(WDTO_8S);                // lets set WDT in case we have a problem...
      
    send(TextMsg.set("Starting"), false);  wait(200);   
    debug1(PSTR("***In Setup\n"));

#ifdef __AVR_ATmega1284P__      // we are useing a Moteino Mega 
  debug1(PSTR(" ** Hello from the Water Meter on a MoteinoMega **\n") );
  
  // de-select on board Flash 
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);
  
  #elif  __AVR_ATmega328P__     // we must have a Moteino
  debug1(PSTR(" ** Hello from the Water Meter on a Moteino **\n") );
  
  // de-select on board Flash 
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  
  #else
    #error wrong processor defined 
#endif

/* ************************************************************************************** */
  const char compile_file[]  = __FILE__ ;
  debug1(PSTR(" %s %s\n"), SKETCHNAME, SKETCHVERSION);
  debug1(PSTR(" %s \n"), compile_file);
  
  const char compile_date[]  = __DATE__ ", " __TIME__;
  debug1(PSTR(" %s \n\n"), compile_date);
  debug1(PSTR(" My Node ID: %u\n\n"), MY_NODE_ID);
  
  // initialize our digital pins internal pullup resistor so one pulse switches from high to low (less distortion) 
  pinMode(DIGITAL_INPUT_SENSOR, INPUT_PULLUP);
  
  // set up reference for ATD to use: (DEFAULT(3.3v), INTERNAL, INTERNAL1V1, INTERNAL2V56, or EXTERNAL). 
  analogReference(DEFAULT);

  // Fetch last known pulse count value from gw if needed
  // request(CHILD_ID, V_VAR1, false);

  lastSendTime = lastPulseTime = millis();          // save current time
  pulseCount = oldPulseCount = 0;                   // reset counters

  // setup interrupt from meter
  attachInterrupt(SENSOR_INTERRUPT, onPulse, FALLING);

#if defined Sensor_SI7021
    si7021.initialize();
#endif

#if defined Sensor_MCP9800
    MCP9800.writeConfig(ADC_RES_12BITS);       // max resolution, 0.0625 °C
#endif
  
} // end setup()


/* **************************************************************************** */
/* *********************** Presentation *************************************** */
/* **************************************************************************** */
void presentation()  
{
 // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCHNAME, SKETCHVERSION, false);   wait(250);
 
  // Register this device as Waterflow sensor
  present(CHILD_ID, S_WATER, "Water Flow", false); wait(250);       // S_WATER = 21 
 }


/* ***************** Send Pressure ***************** */
void SendPressure()
{
/* We will read the analog input from the presure transducer 
 *  and convert it from an analog voltage to a pressure in PSI
 * 
 *  Output: 0.5V – 4.5V linear voltage output. 0 psi outputs 0.5V, 50 psi outputs 2.5V, 100 psi outputs 4.5V 
 *  0   psi = .33v after scalling 5.0v to 3.3v
 *  50  psi = 1.65v
 *  100 psi = 2.97v
 *
 *  3.3v/1024 = .0032266 volt per bit
 */
    pressure  = analogRead    (PressPin) ;                    // this is a junk read to clear ATD
    wait(25);
 
    pressure  = analogRead    (PressPin) ;

    if (pressure < 106) pressure = 106;                       // we have a minimum of .5v = 0 PSI
    PSI = (pressure - 106 ) * .1246;                          // where did we get this?? was .119904
    PSI = PSI + PSI_CAL;                                      // calibration adjustment if needed
    
    floatMSB = PSI * 100;
    floatR = floatMSB % 100;
    debug1(PSTR("PSI:  %0u.%02u\n"), floatMSB/100, floatR);

    send(pressureMsg.set(PSI, 2), false);  wait(200);          // Send water pressure to gateway   
}


/* ***************** Send Si7021 Temp & Humidity ***************** */
#if defined Sensor_SI7021
void getTempSi7021()
{
      si7021.triggerMeasurement();
      wait (25);
      si7021.getHumidity    (humi);
      si7021.getTemperature (temp);
      
      temp = (temp * 9.0)/ 5.0 + 32.0;                           // to get deg F

      floatMSB = humi * 100;                                     // we donot have floating point printing in debug print
      floatR = floatMSB % 100; 
      debug1(PSTR("Humi: %0u.%02u%% \n"), floatMSB/100, floatR);
      
      send(HumMsg.set(humi, 2), false);  wait(200);

      floatMSB = temp * 100;                                     // we donot have floating point printing in debug print
      floatR = floatMSB % 100; 
      debug1(PSTR("Temp: %0u.%02uF \n"), floatMSB/100, floatR);
      
      send(TempMsg.set(temp, 2), false);  wait(200);
}
#endif

/* ***************** Send MCP9800 Temp ***************** */
#if defined Sensor_MCP9800
void  getTempMCP9800 ()
{
  
//    temp = MCP9800.readTempC16(AMBIENT) / 16.0;               // In deg C
      temp = MCP9800.readTempF10(AMBIENT) / 10.0;               // In deg F
      
      floatMSB = temp * 100;                                     // we donot have floating point printing in debug print
      floatR = floatMSB % 100; 
      debug1(PSTR("Temp: %0u.%02uF \n"), floatMSB/100, floatR);
      
      send(TempMsg.set(temp, 2), false);  wait(200);
}
#endif

/* ***************** Send Keep Alive ***************** */
void SendKeepAlive()
{
  if (currentTime - keepaliveTime >= KEEPALIVE_FREQUENCY)
    {
          debug1(PSTR("***Sending Heart Beat\n"));
          sendHeartbeat();  wait(200);
          send(lastCounterMsg.set(pulseCount), false); wait(200);       // Send  pulse count value to gw in VAR1
          send(volumeMsg.set (volume, 0), false);  wait(200);           // Send total volume to gateway
          send(flowMsg.set(flow, 2), false);  wait(200);                // Send current flow value to gateway
          SendPressure();                                               // send water pressure to GW
          keepaliveTime = currentTime;                                  // reset timer

// send excessive flow message if needed
          if ( (flowFlag) && (currentTime - ContFlowRateTime >= ContFlowRateFREQUENCY) )      // see if we have an excessive flow issue over time    
          {
            debug1(PSTR("***Excessive Flow\n"));
            send(TextMsg.set("Excessive Flow"), false);  wait(200); 
          }        

#if defined Sensor_SI7021
          getTempSi7021();
#endif

#if defined Sensor_MCP9800
          getTempMCP9800();
#endif
          
    }  
}

/* ***************** Send Usage per day ***************** */
void Send24Hr ()
{
  if (currentTime - last24HRTime >= DAY_FREQUENCY)
    {
      debug1(PSTR("***24Hr Voulme\n"));
      send(VAR2Msg.set(pulseCount - old24HRPulseCount), false); wait(200);       // Send  pulse count value to gw in VAR2
      old24HRPulseCount = pulseCount;
      last24HRTime = currentTime; 
    } 
}

/* ***************** Send Usage per hour ***************** */
void Send1Hr ()
{
  if (currentTime - last1HRTime >= HOUR_FREQUENCY)
    {
      debug1(PSTR("***1Hr Voulme\n"));
      send(VAR3Msg.set(pulseCount - old1HRPulseCount), false); wait(200);       // Send  pulse count value to gw in VAR2
      old1HRPulseCount = pulseCount;
      last1HRTime = currentTime; 
    } 
}

/* ***************** Check for Excessive Flow ***************** */
void CheckExcessiveFlow()
{
    if (flow > continuousFlowRate )             // check to see if we still have flow over our threshold
    {
        if (!flowFlag) 
        {
          flowFlag= true;                       // we have high flow
          ContFlowRateTime = currentTime;       // save the current time
        }   
    }  
      
    else 
    {
      flowFlag = false;   
     // ContFlowRateTime = currentTime;         // save the current time
    }
}


/* **************************************************************************** */
/* ************************** Loop ******************************************** */
/* **************************************************************************** */
void loop()     
{ 
  wdt_reset();
  currentTime = millis();                                   // get the current time

 /* ***************** Send Pulse Count, Flow and Volume ***************** */
  if (currentTime - lastSendTime >= SEND_FREQUENCY)         // Only send values at a maximum rate  
    {    
      /* ***************** Pulse Count ***************** */
      // Lets see if Pulse Count has changed
      if (pulseCount != oldPulseCount)                                // only send if pulse count has changed
      {
          unsigned int deltaPulseCount = pulseCount - oldPulseCount;  // for test only
          oldPulseCount = pulseCount;
    
          debug1(PSTR("Pulse Count:  %u\n"), (unsigned int) pulseCount );
          debug1(PSTR("Delta Count:  %u\n"), (unsigned int) deltaPulseCount );
          
          send(lastCounterMsg.set(pulseCount), false); wait(200);     // Send  pulse count value to gw in VAR1
          
        /* ***************** Flow Rate ***************** */
          flow = ((60000.0 / (double) PulseInterval) ) / ppg;
    
          debug1(PSTR("**Pulse Interval: %lu\n"), PulseInterval );
    
          if (flow < 0)      // check for less than zero flow rate...
            {
              flow = 0; 
              debug1(PSTR("Flow rate low,  set to zero \n"));
            }                     
    
          // Check that we don't get unresonable large flow value. 
          // could happen when long wraps or false interrupt triggered
          if (flow < ((unsigned long) MAX_FLOW)) 
            {
              floatMSB = flow * 100;                                     // we donot have floating point printing in debug print
              floatR = floatMSB % 100;                                   // so we will breakup float to integer parts
              debug1(PSTR("gal/min: %0u.%02u \n"), floatMSB/100, floatR);
              send(flowMsg.set(flow, 2), false); wait(200);              // Send flow value to gateway
                    
            }  // end if (flow < ((unsigned long) MAX_FLOW)) 
    
          else
            {
              flow = MAX_FLOW+1;                                        // Setmax flow 
              debug1(PSTR("Flow rate too large \n"));
              send(flowMsg.set(flow, 2), false); wait(200);
              send(TextMsg.set("High Flow"), false);  wait(200);  
            } 
          
          flowresetTime = currentTime; 
      }  // end if (pulseCount != oldPulseCount) 
  
        lastSendTime = currentTime; 
            
    }   // end of if (currentTime - lastSendTime > SEND_FREQUENCY)


      CheckExcessiveFlow();

      SendKeepAlive();

      Send24Hr ();

      Send1Hr ();

/* ***************** check if we need to reset flow to Zero ***************** */
    if (currentTime - flowresetTime >= FLOW_RESET_FREQUENCY)              // after a period of time, we will reset flow to zero if we have not
      {                                                                   //   receive any pulses during that time. This keep the old flow from re sending
          if (flow > 0) 
          {
            debug1(PSTR("Setting flow to zero\n"));
            flow = 0;
            send(TextMsg.set("Reset Flow"), false);  wait(200); 
            send(flowMsg.set(flow, 2), false);  wait(200);                // Send zero flow value to gateway 
            
            // its time to send total volume for this run                
            double volume = ((double) (pulseCount - flow_pulseCount) / ((double) PULSE_FACTOR));     
            debug1(PSTR("Volume (gal): %u\n"), (unsigned int) volume );
            send(volumeMsg.set (volume, 0), false); wait(200);            // Send total volume this period to gateway
            flow_pulseCount = pulseCount;        
          }
                 
          flowresetTime = currentTime;
      }     
}   // end of loop


/****************** Message Receive Loop ***************** */
/* 
 * This is the message receive loop, here we look for messages address to us 
 */
void receive(const MyMessage &message) 
{
   //debug2(PSTR("Received message from gw:\n"));
   //debug2(PSTR("Last: %u, Sender: %u, Dest: %u, Type: %u, Sensor: %u\n"), message.last, message.sender, message.destination, message.type, message.sensor);
   
// Make sure its for our child ID
  if (message.sensor == CHILD_ID )
  {
    if  (message.type==V_VAR1)                                                   // Update Pulse Count to new value
      {
        unsigned long gwPulseCount=message.getULong();
        pulseCount += gwPulseCount;
        flow=oldflow=0;
        debug2(PSTR("Received last pulse count from gw: %u\n"), pulseCount);
        pcReceived = true;
      }
    
     if ( message.type==V_VAR2)                                                  // ??
      {
        pulseCount = message.getULong();
        flow=oldflow=0;
        debug2(PSTR("Received V_VAR2 message from gw: %u\n"), pulseCount );
      }
    
     if ( message.type==V_VAR3)                                                   // send all values now
      {
        SendKeepAlive();        
        debug2(PSTR("Received V_VAR3 message from gw"));

     if ( message.type==V_VAR4)                                                   // Calabrate offset for PSI
      {
        PSI_CAL = message.getULong();
        debug2(PSTR("Received V_VAR4 message from gw: %u\n"), PSI_CAL);
      }
      }
    }  // end if (message.sensor == CHILD_ID )


 // Check for any messages for child ID = 2
  if (message.sensor == CHILD_ID2 )
    {
      debug2(PSTR("Received Child ID-2 message from gw. \n"));
    }
}

/* ***************** Interrupt Service ***************** */
/* As this is an interrupt service, we need to keep the code here as short as possable 
 * as we count the pulses from the meter
 */
void onPulse()     
{
    newPulse = millis();                      // get the current time in microseconds
    PulseInterval = newPulse-lastPulseTime;   // get the delta time in us from last interrupt
                                              // we will use this delta (PulseInterval) to compute flow rate                               
    lastPulseTime = newPulse;                 // setup for next pulse, time use by zero flow rate 
    pulseCount++;                             // we also want to count the pulses from meter
 }

/* ***************** The End ***************** */


