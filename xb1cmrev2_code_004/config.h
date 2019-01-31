//Variables, starting values and constants for the Controller Monitor

//Digital button masks as they are mapped to Port C
#define aMask 0x02
#define samplingOffsetDefault 400

//Pin defines
#define logFreq     24            //Where the AREF interrupt edge is found (rising)
#define LIGHT7comp  25            //Where the Light Sensor 7 interrupt edge is found (falling)
#define LXsense   A2      //What the ADCs are attached to
#define LYsense   A3
#define RXsense   A4
#define RYsense   A5
#define LTsense   A12
#define RTsense   A13
#define camSense    0     //Signals coming back from camera
#define camControl  1     //Signal to control camera
#define FTMselect   4     //1 = Lights 4-7, 0 = Secondary FTM triggers
#define muxDir      29
#define userButton  3
#define GPIO0       30
//XB1 Controller Monitor 2018 Model
//Benjamin J Heckendorn
//For use with the Teensy 3.6 MCU and Arduino Enviroment

//Defines and variables

#define GPIO1       4
#define fanControl	46


//Bitmask defines for the (4) indicator lights on front of unit
#define lightRecord   0x01
#define lightPlayback 0x02
#define lightCamera   0x04
#define lightConfig   0x08
#define lightOff      0x00

//Defines for RAM save, playback etc
#define CSVfile       0               //Default is CSV values
#define RAWfile       1               //Can also save raw data
#define sendSD        0               //Default is to save to SD card using current filename
#define sendUART      1               //Data can also be dumped to UART (with handshake/ack from PC)
#define fromSD        0               //When importing data where does it come from?
#define fromUART      1
#define aComma        44              //Comma value in ASCII

#define cameraStart		0x01				//If this bit is set in Camera Control, a pulse is sent to the camera
#define cameraStop		0x02				//If this bit is set the camera pulse is pulled low

uint16_t fanSpeed = 2900;				//Default duty cycle for fan. Avoid multiples of the sample frequency such as 2000 or 4000. Do not exceed 7500. 

uint8_t streamSticks = 0;				//If 1, stream the left and right stick values over serial port

uint8_t sendWhere = sendSD;
uint8_t sendType = CSVfile;
uint8_t printFlags = 1;               //By default do NOT print flags at end of each CSV line
uint8_t userCode = 0;				  //Is User Code active? If so, execute polled functions

uint16_t eventTimer = 0;			 //When autopressing buttons this timer is incremented every sample/control cycle to trigger on certain events
uint8_t eventCounter = 0;			 //Which eventEdge will be triggered next. Starts at 0
uint32_t eventASCII[64];			 //Stores the 3 byte ASCII representation of the opcode
uint16_t eventEdge[64];				 //Stores the offset of each of the 16 possible events. 65535 = end of cycle
void (* event[64]) ();				 //Holds the function pointers for what to do at each of the 16 event points
uint32_t whatCode = 0;				//Programming code parsed from serial input
uint16_t timerEntry = 0;			//Tick counter parsed from serial input

uint8_t debounceTimer = 0;

const int chipSelect = BUILTIN_SDCARD;  // Teensy 3.5 & 3.6 on-board: BUILTIN_SDCARD

//Mapping of segment displays and ports-to-IO-pins

const byte mapLight[8] = {2, 14, 7, 8, 6, 20, 21, 5};   //What pins the 8 light sensors are mapped to
const byte mapC[12] = {15, 22, 23, 9, 10, 13, 11, 12, 35, 36, 37, 38};  //What pins the digital buttons attached to

//Define hexidecimal numbers for 7 segment display 0-F, blank space and special characters
const byte seg7[21] = {0xC0, 0xF9, 0xA4, 0x4B0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E, 0xFF, 0x8C, 0xAF, 0xA3, 0xC2};

volatile uint16_t digits[4] = {0, 0, 0, 0};

uint8_t lightMaskNumber = 7;            //Integer # of which light we're using
uint16_t lightBitMask = 0x80;           //Default is to use Light Bit 7 as the edge flag for timer


uint8_t hexDecimal = 1;                   //When showing a frame timer 0 = hex 1 = decimal

uint32_t masterTimer = 0;                 //Cycle timer not tied to logging routines.
volatile uint16_t timer = 0;
volatile uint16_t seconds = 0;
volatile uint8_t edgeSource = 0;          //0 = Controller AREF (default) 1 = Light 7 Sensor
volatile uint8_t subMultiplier = 0;       //The frequency multiplier to apply to input trigger. 500Hz default
volatile uint8_t subCounter = 0;          //Counts how many sub multiples are left (variable)
volatile uint16_t subInterval = 2000;     //The interval between sub divisions. 8000us/interval (500Hz default)
volatile uint16_t samplingOffset = samplingOffsetDefault;  //The gap between getting the interrupt trigger and actually sampling the data
uint16_t secondsDivider = 125;

uint8_t autoNumber = 0;                   //Flag if we're going to auto number or not
uint16_t autoValue = 0;                   //What current auto number we are on
uint16_t autoStart = 0;					  //The starting automatic number. Use this when auto loading files
uint8_t seqFile = 0;					 //If sequential file loading is enabled or not (Filename must be specified first)

uint16_t leftAnalogs = 0;                 //To build data for the left analog LED translater
uint16_t rightAnalogs = 0;                //To build data for the right analog LED translater

uint8_t playBackRepeat = 0;       //Whether or not automatic playback is enabled
uint8_t playBackDelay = 2;        //The # of seconds between automatic playbacks

uint8_t writeAfterLogging = 1;    //Flag if we should write RAM to a CSV file after we capture it

uint16_t start = 0;

const int shiftLoad = 26;         //Referred to as /CS0 on schematic

uint8_t cameraState = 0;          //If we're triggering a camera or not. Stores current state as well
uint8_t cameraType = 0;           //The way in which we're controlling camera (index)

uint8_t controlSample = 0;     //Which mode we're in. Sample = 0 Playback Control = 1 Event Control = 2

uint8_t loggingState = 0;         //If the system is logging and how
uint8_t displayState = 0;         //What should appear on the 7 segment displays every interrupt
uint8_t displayFrames = 0;        //1 = show counters in frames 0 = show counter in seconds
uint8_t userState = 0;            //What happens when you press the USER button on front of unit
uint8_t menuWhich = 0;            //Which menu we should be in (if any) 0 = No active menu
uint8_t menuState = 0;            //Which menu we're in (for free running mode)
uint8_t aState = 0;               //What state the A button control is in. 1=Waiting for A, 2 = A pressed 0 = Acknowledge press/clear-reset

//0 = Timer is incremented every interrupt and divided by secondsDivider to display a seconds counter
//1 = ???



//Variables for analog reads
uint8_t RX = 128;
uint8_t RY = 128;
uint8_t LX = 128;
uint8_t LY = 128;
uint8_t LT = 0;
uint8_t RT = 0;


uint8_t userLights = 0xF0;  //By default the 4 user lights are off (high)

//Single port value for button status
uint16_t buttons = 0;     //ABXY UDLR LB RB L3 R3 12 bits of button data, top 4 bits used as Event Flags

//Variables for when to log analog trigger values
volatile uint32_t triggerCounter = 0;        //Edge delay for sampling triggers in the active range
volatile uint8_t triggered = 0;

uint16_t logCounter = 0;            //Use to count down from a number of samples without clearing the sample counter
uint16_t logSamples = 0;            //How many valid samples are currently in RAM
uint32_t logP = 0;                  //Array pointer of which frame of the log we're looking at
uint8_t dataLog[240000];            //Stores data logged from the controller or meant to be played back to it

char f;                             //Used for reading single chars from a file
char fileString[32];                //Temp storage for what we read from files. Much longer than it ever needs to be

char fileName[] = {"CONTL000.CSV"}; //Default file name for saving to SD card
char input[] = {"XXXXXXXX.CSV"};      //Array used to input and parse filenames
uint8_t sPointer = 0;				//A global pointer of where we are in a string interpretation

uint8_t LTedges[3];                 //Edge dividers at which point the analog LED's will turn on or off
uint8_t RTedges[3];
uint8_t LXedges[6];
uint8_t LYedges[6];
uint8_t RXedges[6];
uint8_t RYedges[6];

uint8_t delta = 0;

int g = 0;                      //It's a classic! Use INT so negatives can be used as indicators

//EEPROM locations for stored data--------------------------------------

uint8_t LTr = 0; //EEPROM 2
uint8_t LTp = 0; //EEPROM 3

uint8_t RTr = 0; //EEPROM 0
uint8_t RTp = 0; //EEPROM 1

uint8_t RXl = 0; //EEPROM
uint8_t RXc = 0; //EEPROM
uint8_t RXr = 0; //EEPROM

uint8_t RYu = 0; //EEPROM
uint8_t RYc = 0; //EEPROM
uint8_t RYd = 0; //EEPROM

uint8_t LXl = 0; //EEPROM
uint8_t LXc = 0; //EEPROM
uint8_t LXr = 0; //EEPROM

uint8_t LYu = 0; //EEPROM
uint8_t LYc = 0; //EEPROM
uint8_t LYd = 0; //EEPROM


//Analog Stick Digital Pot Ranges
#define upRange	230			//225			//7.8k = 1.4v parallel
#define downRange 30		//1.3k = .4v parallel

uint8_t stickRangeLXr = upRange; //150;
uint8_t stickRangeLXl = downRange; //20;
uint8_t stickRangeLYd = upRange; //150;
uint8_t stickRangeLYu = downRange; //20;

uint8_t stickRangeRXr = upRange; //175;
uint8_t stickRangeRXl = downRange; //20;
uint8_t stickRangeRYd = upRange; //175;
uint8_t stickRangeRYu = downRange; //20;

//Analog Trigger Digital Pot Ranges
uint8_t triggerRangeH = 10; //10;
uint8_t triggerRangeL = 1; //1;

//Stuff from 2013 version
unsigned char flightType = 0;		//0 = None 1 = Face Buttons
unsigned long flight = 0;       //Count of us elapsed since timer started
unsigned long averageUS = 0;    //Total # of us counted
unsigned long sampleCount = 0;  //How many samples we've taken so we can divide and show an average

void (* userFunc)();			//Holds the address of the function to call when User Button is tapped


