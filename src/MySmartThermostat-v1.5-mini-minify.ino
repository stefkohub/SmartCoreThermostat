/*
 * Project MySmartThermostat
 * Description: Main code to manage home boiler via Particle Spark Core
 * Author: Stefano Falsetto
 * Date: 05/25/2019 (25th of May 2018)
 */

#define LOCAL_PARAMS 1

#define ONEWIRE_CRC 0
#define ONEWIRE_CRC16 0

 #include "DS18.h"
 #include "SparkIntervalTimer.h"
 #include "rest_client.h"

 STARTUP(System.enableFeature(FEATURE_RESET_INFO));

#ifdef LOCAL_PARAMS
  #include "MySmartThermostat.h"
#else
 #define ONE_WIRE_BUS D6
 #define TEMPERATURE_PRECISION 9
 #define POTENTIOMETER_PIN A0
 #define RELAY_SIGNAL_PIN D4
 #define MANUALSETTING_PUSHBUTTON A3
 #define MANUALSETTING_LED A2

 #ifdef POTENTIOMETER_CALIBRATION
  #define EEPROM_POTEMTIOMETER_MAX_ADDRESS 0
  #define EEPROM_POTEMTIOMETER_MIN_ADDRESS 10
 #endif

 #define TEMP_SCALE_FACTOR 100

 #define DEFAULT_SETPOINT_VALUE 60
 // INTERVALLI DA VEDERE DIREI A REGIME DI FARLI DIVENTARE MOLTO PIU' LENTI
 // TIPO 60 SECONDI
 // THERMOMETER TEMPERATURE POLLING PERIOD
 #define GET_TEMP_PERIOD 5000
 // NETWORK (HUE) POLLING DATA PERIOD
 #define NETWORK_UPDATE_PERIOD 10000

 // QUESTO MI SA CHE DEVE RIMANERE COSI'
 // TEMPERATURE CONTROLLER. I THINK MUST BE
 // THE SAME OF GET_TEMP_PERIOD
 #define TEMP_CONTROLLER_PERIOD 5000

 // POTENTIOMETER READING PERIOD
 #define POTENTIOMETER_UPDATE_PERIOD 2000

 // PUSHBUTTON BOUNCING DELAY TIME
 #define DEBOUNCE_DELAY 50

 #define MAX_STARVATION_TIME 20000
 #define KEEPALIVE_PERIOD 5000
 #define RESET_TIMEOUT 240

 #define HUE_HOST_STRING "your-HUE-hostname"
 #define HUE_API_BASEURL "/api/your-user-API-key"

 #define UDP_LOG_IP your,log,server,ip
 #define UDP_LOG_PORT your-log-server-port
 #define INFOLVL "INFO"
 #define ERRORLVL "ERROR"
 #define TRACELVL "TRACE"

#endif

 IntervalTimer watchdogTimer;		// 3 for the Core

uint8_t manualsetting_ledState = LOW;         // the current state of the output pin
uint8_t manualsetting_buttonState = LOW;             // the current reading from the input pin
uint8_t manualsetting_lastButtonState = LOW;   // the previous reading from the input pin
uint8_t periodicResetCount=0;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long lastGetTemp = millis();
unsigned long lastTempController = millis();
unsigned long lastNetworkUpdate = millis();
unsigned long lastPotentiometerUpdate = millis();
unsigned long lastKeepaliveCheck = millis();


 DS18 Thermometer(ONE_WIRE_BUS, true);
 static uint8_t Taddr[8]={};

 RestClient Rclient = RestClient(HUE_HOST_STRING);

 bool manualSettingControl;
 bool blocking_error=FALSE;
 #ifdef POTENTIOMETER_CALIBRATION
  int PMAX=0;
  int PMIN=0;
 #else
  #define PMIN 1
  #define PMAX 4093
 #endif

int T=0;
int setPoint=DEFAULT_SETPOINT_VALUE;

// SerialLogHandler logHandler(LOG_LEVEL_ALL); // normal: LOG_LEVEL_INFO

static UDP Udp;

void UDPstart() {
  Udp.begin(UDP_LOG_PORT);
}

void UDPstop() {
  Udp.stop();
}

void UDPLog(String t, String msg) {
  msg=String(millis())+" - "+t+": "+msg;
  if (Udp.sendPacket(msg.c_str(), msg.length(), IPAddress(UDP_LOG_IP), UDP_LOG_PORT)<0) {
    Log.trace("UDP rst");
    UDPstop();
    delay(1000);
    UDPstart();
    UDPLog(ERRORLVL, "UDP rst");
  }
}

class ExternalRGB {
  public:
    ExternalRGB(pin_t r, pin_t g, pin_t b) : pin_r(r), pin_g(g), pin_b(b) {
      pinMode(pin_r, OUTPUT);
      pinMode(pin_g, OUTPUT);
      pinMode(pin_b, OUTPUT);
      RGB.onChange(&ExternalRGB::handler, this);
    }

    void handler(uint8_t r, uint8_t g, uint8_t b) {
      analogWrite(pin_r, /*255 -*/ r);
      analogWrite(pin_g, /*255 -*/ g);
      analogWrite(pin_b, /*255 -*/ b);
    }

    private:
      pin_t pin_r;
      pin_t pin_g;
      pin_t pin_b;
};

// Connect an external RGB LED to D0, D1 and D2 (R, G, and B)
ExternalRGB myRGB(A4, A6, A5);

 int getTemp() {
   if (Thermometer.read(Taddr)) {
     return (int) Thermometer.celsius()*TEMP_SCALE_FACTOR;
   } else {
     return -1;
   }
}

void updateTemp(int t) {
   String b = String::format("{\"temperature\":%d}",t);
   if (Rclient.put(String(HUE_API_BASEURL)+"sensors/11/state", b)!=200) {
	   UDPLog(ERRORLVL, String("Cannot update T"));
   }
 }

 int getData() {
   String response = "";
   int16_t setPoint = 0;
   if (Rclient.get(String(HUE_API_BASEURL)+"sensors/12", &response)==200) {
      uint8_t q1 = response.indexOf("\"on\":");
      uint8_t q2 = response.indexOf(",",q1);
      // Log.trace("(ON/OFF) %d, %d: %s",q1,q2,(const char*)response.substring(q1+5,q2));
      if ((q1>0) && (q2>0) && (!response.substring(q1+5,q2).equalsIgnoreCase("true"))) {
        setPoint=0;
      } else {
    	  q1 = response.indexOf("\"status\":");
        q2 = response.indexOf(",",q1);
        //Log.trace("q1=%d, q2=%d", q1, q2);
        // Log.trace("(STATUS) %d, %d: %s",q1,q2,(const char*)response.substring(q1+9,q2));
    	  if ((q1>0) && (q2>0)) {
          setPoint = (int16_t) response.substring(q1+9,q2).toInt();
          //Log.trace("Substring=%s", response.substring(q1+9, q2).c_str());
        }
      }
      UDPLog(TRACELVL, String::format("%s - %d", (const char*)response, setPoint));
	} else {
      UDPLog(ERRORLVL, String("getData network error"));
      // return 0;
   }
   //Log.info("setpoint=%d",setPoint);
   return setPoint;
 }

 void tempController(int sp, int t) {
  if (t<sp*TEMP_SCALE_FACTOR) {
     //UDPLog(INFOLVL, String::format("T:%d<SP:%d - ON",t,sp*TEMP_SCALE_FACTOR));
     digitalWrite(RELAY_SIGNAL_PIN, HIGH);
  } else {
     //UDPLog(INFOLVL, String::format("T:%d>=SP:%d - OFF",t,sp*TEMP_SCALE_FACTOR));
     digitalWrite(RELAY_SIGNAL_PIN, LOW);
  }
 }

volatile bool watchdogFirstRun = true;

void starvationFunction() {
  if (!watchdogFirstRun) {
    UDPLog(ERRORLVL, String("STARVATION!!"));
    System.reset();
  }
  watchdogFirstRun=false;
}

 void setup()
 {
   // Serial.begin(115200);
   // NOT NEEDED: SEE PARTICLE DOCS pinMode(POTENTIOMETER_PIN, INPUT);
   pinMode(RELAY_SIGNAL_PIN, OUTPUT);
   // pinMode(MANUALSETTING_PUSHBUTTON, INPUT);
   pinMode(MANUALSETTING_LED, OUTPUT);

   if (!Thermometer.read()) {
     Log.error("No Thermometer");
     blocking_error = TRUE;
     return;
   } else {
     T=Thermometer.celsius();
     Thermometer.addr(Taddr);
   }
   UDPstart(); //Udp.begin(UDP_LOG_PORT);
   UDPLog(INFOLVL, String::format("Ver: %s - LastReset: %d", (const char*)System.version(), System.resetReason()));
   delay(5000);
   watchdogTimer.begin(starvationFunction, MAX_STARVATION_TIME, hmSec);
 }

 int normalize_temperature_reading() {
   return map(analogRead(POTENTIOMETER_PIN), PMIN, PMAX, 0, 100);
 }

 uint8_t debouncing_pushbutton_read() {
   // read the state of the switch into a local variable:
  uint8_t reading = digitalRead(MANUALSETTING_PUSHBUTTON);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != manualsetting_lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != manualsetting_buttonState) {
      manualsetting_buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (manualsetting_buttonState == HIGH) {
        manualsetting_ledState = !manualsetting_ledState;
      }
    }
  }

  // set the LED:
  digitalWrite(MANUALSETTING_LED, manualsetting_ledState);

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  manualsetting_lastButtonState = reading;
  return manualsetting_ledState;
 }

 void loop()
 {
  //T=0;
  //setPoint=last_setPoint; //DEFAULT_SETPOINT_VALUE;

   // starvationCounter++;
   if (blocking_error) {
      UDPLog(ERRORLVL, String("Cannot run."));
      delay(20000);
      return;
   }

   manualSettingControl=debouncing_pushbutton_read();

   // Maybe we can still force a Core reset each X minutes (hours)
   // just to be sure all those millis counters doesn't overflow
   if ((millis() - lastKeepaliveCheck) > KEEPALIVE_PERIOD) {
	   lastKeepaliveCheck=millis();
     periodicResetCount++;
     if (periodicResetCount>=RESET_TIMEOUT) {
       UDPLog(INFOLVL, "Periodic Reset");
       System.reset();
     }

     watchdogTimer.resetPeriod_SIT(MAX_STARVATION_TIME, hmSec);
	   Log.trace("-- MARK %s --",watchdogFirstRun?"True":"False");
     //Log.info("T: %d, M: %s, SP: %d",T,manualSettingControl==HIGH?"Manual":"Network",setPoint);
     UDPLog(INFOLVL, String::format("T: %d, M: %s, SP: %d, RC: %d", \
        T, manualSettingControl==HIGH?"Manual":"Network", \
        setPoint, periodicResetCount));
   }

   if ((millis() - lastGetTemp) > GET_TEMP_PERIOD) {
     lastGetTemp=millis();
     T = getTemp();
   }
   // UDPLog(TRACELVL, String::format("Button: %s",manualSettingControl==HIGH?"HIGH":"LOW"));
   if (manualSettingControl==HIGH) {
    // QUI POTREI USARE SYSTEM.SLEEP PER SPEGNERE IL WIFI
    // ED USARE UN TRIGGER SUL PIN DEL PUSHBUTTON
     if (millis() - lastPotentiometerUpdate > POTENTIOMETER_UPDATE_PERIOD) {
       lastPotentiometerUpdate = millis();
       setPoint = normalize_temperature_reading();
     }
   } else {
     if ((millis() - lastNetworkUpdate) > NETWORK_UPDATE_PERIOD) {
       lastNetworkUpdate = millis();
       updateTemp(T);
       setPoint = getData();
     }
   }
   if ((millis() - lastTempController) > TEMP_CONTROLLER_PERIOD) {
     lastTempController = millis();
     tempController(setPoint, T);
   }
 }
