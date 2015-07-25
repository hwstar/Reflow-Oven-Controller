/*******************************************************************************
* Title: Reflow Oven Controller
* Version: 1.20
* Date: 26-11-2012
* Company: Rocket Scream Electronics
* Author: Lim Phang Moh
* Website: www.rocketscream.com
* 
* Brief
* =====
* This is an example firmware for our Arduino compatible reflow oven controller. 
* The reflow curve used in this firmware is meant for lead-free profile 
* (it's even easier for leaded process!). You'll need to use the MAX31855 
* library for Arduino if you are having a shield of v1.60 & above which can be 
* downloaded from our GitHub repository. Please check our wiki 
* (www.rocketscream.com/wiki) for more information on using this piece of code 
* together with the reflow oven controller shield. 
*
* Temperature (Degree Celcius)                 Magic Happens Here!
* 245-|                                               x  x  
*     |                                            x        x
*     |                                         x              x
*     |                                      x                    x
* 200-|                                   x                          x
*     |                              x    |                          |   x   
*     |                         x         |                          |       x
*     |                    x              |                          |
* 150-|               x                   |                          |
*     |             x |                   |                          |
*     |           x   |                   |                          | 
*     |         x     |                   |                          | 
*     |       x       |                   |                          | 
*     |     x         |                   |                          |
*     |   x           |                   |                          |
* 30 -| x             |                   |                          |
*     |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
*     | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
*  0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
*                                                                Time (Seconds)
*
* This firmware owed very much on the works of other talented individuals as
* follows:
* ==========================================
* Brett Beauregard (www.brettbeauregard.com)
* ==========================================
* Author of Arduino PID library. On top of providing industry standard PID 
* implementation, he gave a lot of help in making this reflow oven controller 
* possible using his awesome library.
*
* ==========================================
* Limor Fried of Adafruit (www.adafruit.com)
* ==========================================
* Author of Arduino MAX6675 library. Adafruit has been the source of tonnes of
* tutorials, examples, and libraries for everyone to learn.
*
* Disclaimer
* ==========
* Dealing with high voltage is a very dangerous act! Please make sure you know
* what you are dealing with and have proper knowledge before hand. Your use of 
* any information or materials on this reflow oven controller is entirely at 
* your own risk, for which we shall not be liable. 
*
* Licences
* ========
* This reflow oven controller hardware and firmware are released under the 
* Creative Commons Share Alike v3.0 license
* http://creativecommons.org/licenses/by-sa/3.0/ 
* You are free to take this piece of code, use it and modify it. 
* All we ask is attribution including the supporting libraries used in this 
* firmware. 
*
* Required Libraries
* ==================
* - Arduino PID Library: 
*   >> https://github.com/br3ttb/Arduino-PID-Library
* - MAX31855 Library (for board v1.60 & above): 
*   >> https://github.com/rocketscream/MAX31855
*
* Revision  Description
* ========  ===========
* 1.20			Adds supports for v1.60 (and above) of Reflow Oven Controller 
*           Shield:
*					  - Uses MAX31855KASA+ chip and pin reassign (allowing A4 & A5 (I2C)
*             to be used for user application).
*					  - Uses analog based switch (allowing D2 & D3 to be used for user 
*						  application).	
*						Adds waiting state when temperature too hot to start reflow process.
*						Corrected thermocouple disconnect error interpretation (MAX6675).
* 1.10      Arduino IDE 1.0 compatible.
* 1.00      Initial public release.
*******************************************************************************/

// ***** INCLUDES *****
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <MAX31855.h>
#include <PID_v1.h>

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
	REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR,
  REFLOW_STATE_TEST
} reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;


typedef struct
{
	uint8_t closedStatus;
	uint8_t lastClosedStatus;
	uint8_t switchReleased;
	unsigned long lastDebounceTime;
} switches_t;




#define SWITCH_1 0x1
#define SWITCH_2 0x2

// ***** CONSTANTS *****
///#define TC_CALIB 0
///#define TEMPERATURE_ROOM 50
///#define TEMPERATURE_SOAK_MIN 150
///#define TEMPERATURE_SOAK_MAX 180
///#define TEMPERATURE_REFLOW_MAX 250
///#define TEMPERATURE_COOL_MIN 100
///#define SENSOR_SAMPLING_TIME 1000
///#define SOAK_TEMPERATURE_STEP 5
///#define SOAK_MICRO_PERIOD 9000
///#define DEBOUNCE_PERIOD_MIN 50



// ***** CONSTANTS *****
#define TC_CALIB 0
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_SOAK_MAX 180
#define TEMPERATURE_REFLOW_MAX 225
#define TEMPERATURE_COOL_MIN 210
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 3
#define SOAK_MICRO_PERIOD 13000
#define DEBOUNCE_PERIOD_MIN 50


// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
  "Ready",
  "Pre-heat",
  "Soak",
  "Reflow",
  "Cool",
  "Complete",
	"Wait,hot",
  "Error",
  "Test"
};

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {
  140,146,146,140,128,128,128,128};

// ***** PIN ASSIGNMENT *****
uint8_t ElementPin = 2;
uint8_t LEDPin = 3;
uint8_t thermocoupleSOPin = 12;
uint8_t thermocoupleCSPin = 9;
uint8_t thermocoupleCLKPin = 13;
uint8_t lcdRSTPin = 4;
uint8_t lcdCEPin = 8;
uint8_t lcdDCPin = 7;
uint8_t lcdSOPin = 11;
uint8_t lcdCLKPin = 13;
uint8_t buzzerPin = 10;
uint8_t convectionFanPin = 5;
uint8_t switchPin1 = A0;
uint8_t switchPin2 = A1;



// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
unsigned windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long timerSoak;
unsigned long buzzerPeriod;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;

// Switch debounce timer
long lastDebounceTime;
// Seconds timer
int timerSeconds;
// Phase timer
int phaseSeconds;
// Switches data structure
switches_t switches;


// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
// Specify LCD interface
///LiquidCrystal lcd(lcdRsPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);
// Specify MAX6675 thermocouple interface

MAX31855 thermocouple(thermocoupleSOPin, thermocoupleCSPin, 
		thermocoupleCLKPin);
// Specify LCD display interface
Adafruit_PCD8544 display = Adafruit_PCD8544(lcdCLKPin, lcdSOPin,
		lcdDCPin, lcdCEPin, lcdRSTPin );

void setup()
{
  // Element pin initialization to ensure reflow oven is off
  digitalWrite(ElementPin, LOW);
  pinMode(ElementPin, OUTPUT);
  digitalWrite(LEDPin, HIGH);
  pinMode(LEDPin, OUTPUT);
  digitalWrite(convectionFanPin, LOW);
  pinMode(convectionFanPin, OUTPUT);
  
  // Buzzer pin initialization to ensure annoying buzzer is off
  digitalWrite(buzzerPin, LOW);
  pinMode(buzzerPin, OUTPUT);

  // LED pins initialization and turn on upon start-up (active low)
  ///digitalWrite(ledRedPin, LOW);
  ///pinMode(ledRedPin, OUTPUT);


  // Start-up splash
 
  display.begin(); // Flash ADI logo as quickly as possible
  delay(100);
  display.clearDisplay();
  display.setContrast(50);
  
  // Our splash
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0,0);
  display.println("Reflow V0.0");
  display.display();
  delay(2000);
  digitalWrite(buzzerPin, HIGH);
  delay(250);
  digitalWrite(buzzerPin, LOW);
  display.clearDisplay();
  display.display();
  // Splash done
 

  // Serial communication at 57600 bps
  Serial.begin(9600);
  Serial.println("R");

  // Turn off LED (active low)
  ///digitalWrite(ledRedPin, HIGH);
	#ifdef  USE_MAX6675
		digitalWrite(ledGreenPin, HIGH);
	#endif
  // Set window size
  windowSize = 2000;
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();
}


/*
 * Service front panel buttons
 */

void serviceButtons(void)
{
	
	unsigned long now = millis();
	
	// Clear last released state
	switches.switchReleased = 0;
	
	// If this is the first time called, we need to set lastDebounceTime to a positive value.
	if(!switches.lastDebounceTime)
		switches.lastDebounceTime = now;
		
	// Don't proceed further unless the debounce period has expired.
	if((now - switches.lastDebounceTime) < DEBOUNCE_PERIOD_MIN){
		return;
	}
		
	// Remember last sample time
	switches.lastDebounceTime = now;
	
	// Save previous state
	switches.lastClosedStatus = switches.closedStatus;
	
	// Update the closed state	
	if(analogRead(switchPin1) < 512)
		switches.closedStatus |= SWITCH_1;
	else
		switches.closedStatus &= ~SWITCH_1;
		
	if(analogRead(switchPin2) < 512)
		switches.closedStatus |= SWITCH_2;
	else
		switches.closedStatus &= ~SWITCH_2;
		
	// Check for switch 1 release
	
	if(((switches.lastClosedStatus & SWITCH_1) > 0) &&
		((switches.closedStatus & SWITCH_1) == 0 ))
			switches.switchReleased |= SWITCH_1;
		
	// Check for switch 2 release	
	if(((switches.lastClosedStatus & SWITCH_2) > 0) &&
		((switches.closedStatus & SWITCH_2) == 0))
			switches.switchReleased |= SWITCH_2;

}

void loop()
{
  // Current time
  unsigned long now;

 
  
  // Time to read thermocouple?
  if (millis() > nextRead)
  {
	uint8_t trys = 10;
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    
    // Send state info
    Serial.print("S:");
    Serial.print(lcdMessagesReflowStatus[reflowState]); 
    Serial.print(":");
    Serial.println(phaseSeconds);
    
redo_measurement:
    // Read current temperature
	input = thermocouple.readThermocouple(CELSIUS);
	
		
    // If thermocouple problem detected

	if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || 
		(input == FAULT_SHORT_VCC)){
      // Illegal operation
      if(FAULT_OPEN == input) // DEBUG
		Serial.println("E:TC open");
	  else if(FAULT_SHORT_GND == input)
		Serial.println("E:TC gnd");
	  else if(FAULT_SHORT_VCC == input)
		Serial.println("E:TC VCC");  

	  if(trys){
		trys--;
		goto redo_measurement;
	  }
	  
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
    }
    else
		input += TC_CALIB; //DEBUG Thermocouple calibration
		
		// Send time and temperature info
		Serial.print("T:");
		Serial.print(timerSeconds);
		Serial.print(":");
		Serial.print(input);
		Serial.print(":");
		Serial.print(setpoint);
		Serial.print(":");
		Serial.println(output);

		// Update display
		digitalWrite(LEDPin, HIGH);
		
		display.setContrast(50);
		display.clearDisplay();
		display.setTextSize(1);
		display.setCursor(0,0);
		display.println(lcdMessagesReflowStatus[reflowState]);
		display.setTextSize(2);
		display.print(input, 1);
		display.println('C');

		display.setTextSize(1);
		char buf[10];
		sprintf(buf,"%3d %3d", timerSeconds, phaseSeconds);
		display.println(buf);
		
		digitalWrite(lcdCLKPin, HIGH); // Crude way of sharing the clock line
		digitalWrite(lcdCLKPin, LOW); 
		digitalWrite(lcdCLKPin, HIGH); 
		display.display();
		digitalWrite(lcdCLKPin, LOW);
		digitalWrite(lcdCLKPin, HIGH); // Crude way of sharing the clock line
		digitalWrite(lcdCLKPin, LOW); 
		
		digitalWrite(LEDPin, LOW);
  }
  

  if (millis() > nextCheck)
  {
    // Check input in the next seconds
    nextCheck += 1000;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Increase seconds timer for reflow curve analysis
      timerSeconds++;
      // Increase phase seconds timer for display purposes
      phaseSeconds++;
    }
    else
    {
      // Turn off red LED
      ///digitalWrite(ledRedPin, HIGH);
    }

    // Clear LCD
    ///lcd.clear();
    // Print current system state
    ///lcd.print(lcdMessagesReflowStatus[reflowState]);
    // Move the cursor to the 2 line
    ///lcd.setCursor(0, 1);

    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {
      // No thermocouple wire connected
      ///lcd.print("TC Error!");
    }
    else
    {
      // Print current temperature
      ///lcd.print(input);

			#if ARDUINO >= 100
				// Print degree Celsius symbol
				///lcd.write((uint8_t)0);
			#else
				// Print degree Celsius symbol
				///lcd.print(0, BYTE);
			#endif
      ///lcd.print("C ");
    }
  }

  // Reflow oven controller state machine
  switch (reflowState)
  {
  case REFLOW_STATE_IDLE:
		timerSeconds = phaseSeconds = 0;
		// If oven temperature is still above room temperature
		if (input >= TEMPERATURE_ROOM)
		{
			reflowState = REFLOW_STATE_TOO_HOT;
		}
		else
		{
			// If switch is pressed to start reflow process
			if (switches.switchReleased & SWITCH_1){
				// Initialize PID control window starting time
				windowStartTime = millis();
				// Ramp up to minimum soaking temperature
				setpoint = TEMPERATURE_SOAK_MIN;
				// Tell the PID to range between 0 and the full window size
				reflowOvenPID.SetOutputLimits(0, windowSize);
				reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
				// Turn the PID on
				reflowOvenPID.SetMode(AUTOMATIC);
				// Proceed to preheat stage
				reflowState = REFLOW_STATE_PREHEAT;
			}
    }
    break;

  case REFLOW_STATE_PREHEAT:
    reflowStatus = REFLOW_STATUS_ON;
    // If minimum soak temperature is achieve       
    if (input >= TEMPERATURE_SOAK_MIN)
    {
      // Chop soaking period into smaller sub-period
      timerSoak = millis() + SOAK_MICRO_PERIOD;
      // Set less agressive PID parameters for soaking ramp
      reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
      // Ramp up to first section of soaking temperature
      setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;   
      // Proceed to soaking state
      phaseSeconds = 0;
      reflowState = REFLOW_STATE_SOAK; 
    }
    break;

  case REFLOW_STATE_SOAK:     
    // If micro soak temperature is achieved       
    if (millis() > timerSoak)
    {
      timerSoak = millis() + SOAK_MICRO_PERIOD;
      // Increment micro setpoint
      setpoint += SOAK_TEMPERATURE_STEP;
      if (setpoint > TEMPERATURE_SOAK_MAX)
      {
        // Set agressive PID parameters for reflow ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_REFLOW_MAX;   
        // Proceed to reflowing state
        phaseSeconds = 0;
        reflowState = REFLOW_STATE_REFLOW; 
      }
    }
    break; 

  case REFLOW_STATE_REFLOW:
    // We need to avoid hovering at peak temperature for too long
    // Crude method that works like a charm and safe for the components
    if (input >= (TEMPERATURE_REFLOW_MAX - 5))
    {
      // Set PID parameters for cooling ramp
      reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
      // Ramp down to minimum cooling temperature
      setpoint = TEMPERATURE_COOL_MIN;   
      // Proceed to cooling state
      phaseSeconds = 0;
      reflowState = REFLOW_STATE_COOL; 
    }
    break;   

  case REFLOW_STATE_COOL:
    // If minimum cool temperature is achieve       
    if (input <= TEMPERATURE_COOL_MIN)
    {
      // Retrieve current time for buzzer usage
      buzzerPeriod = millis() + 1000;
      // Turn on buzzer and green LED to indicate completion

	  digitalWrite(buzzerPin, HIGH);
	  digitalWrite(convectionFanPin, HIGH);
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;                
      // Proceed to reflow Completion state
      phaseSeconds = 0;
      reflowState = REFLOW_STATE_COMPLETE; 
    }         
    break;    

  case REFLOW_STATE_COMPLETE:
    if (millis() > buzzerPeriod)
    {
      // Turn off buzzer and green LED
      digitalWrite(buzzerPin, LOW);
	  // Reflow process ended
      reflowState = REFLOW_STATE_IDLE; 
    }
    break;
	
	case REFLOW_STATE_TOO_HOT:
		// If oven temperature drops below room temperature
		if (input < TEMPERATURE_ROOM)
		{
			// Ready to reflow
			digitalWrite(convectionFanPin, LOW);
			reflowState = REFLOW_STATE_IDLE;
		}
		break;
		
  case REFLOW_STATE_ERROR:
    // If thermocouple problem is still present
	
	
	if((input == FAULT_OPEN) || (input == FAULT_SHORT_GND) || 
		(input == FAULT_SHORT_VCC)){
      // Wait until thermocouple wire is connected
      reflowState = REFLOW_STATE_ERROR; 
    }
    else
    {
      // Clear to perform reflow process
      reflowState = REFLOW_STATE_IDLE; 
    }
    break;    
    
  case REFLOW_STATE_TEST: // For testing purposes
  
	break;

    
  }  
  

  // If switch 1 is pressed
  if (switches.switchReleased & SWITCH_1)
  {
    // If currently reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Button press is for cancelling
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;
      // Reinitialize state machine
      reflowState = REFLOW_STATE_IDLE;
    }
  } 


  // Tend to front panel buttons
  
  serviceButtons();

  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    now = millis();

    reflowOvenPID.Compute();

    if((now - windowStartTime) > windowSize)
    { 
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if(output > (now - windowStartTime)){
		digitalWrite(ElementPin, HIGH);
	}
    else{
		digitalWrite(ElementPin, LOW);  
	} 
  }
  // Reflow oven process is off, ensure oven is off
  else 
  {
    digitalWrite(ElementPin, LOW);
  
  }
}
