//XB1 Controller Monitor 2018 Model
//Benjamin J Heckendorn
//For use with the Teensy 3.6 MCU and Arduino Enviroment

IntervalTimer myTimer;      //For testing purposes without a controller attached
#include <TeensyDelay.h>    //Delayed timer used to sub-divide the AREF based 125Hz interrupt
#include <SD.h>             //For file system
#include <SPI.h>            //Hardware serial loading of light data
#include <Wire.h>           //I2C for controlling the digital potentiometers
#include <EEPROM.h>         //Stores default controller values to non-volatile memory

#include "config.h"         //Variables for Controller Monitor software

File myFile;

void printDirectory(File dir, int numTabs);

void setup() {

  userFunc = &startLogging;           //Default function that's called when you tap USER button

  //Begin I2C bus on standard Teensy 3.6 pins
  Wire.setSCL(33);
  Wire.setSDA(34);
  Wire.begin();
  
  //Uses the 1V8 regulator for both analog triggers and sticks
  analogReference(EXTERNAL);      

  //Set up MUX control
  pinMode(muxDir, OUTPUT);        //Read or write
  digitalWrite(muxDir, 1);        //Set as POT/BUTTON SENSE (1)

  //Set up SPI for loading analog stick LED displays
  pinMode(shiftLoad, OUTPUT);
  SPI.setMOSI(28);    //Do not use the standard SPI0 as we're using it for something else
  SPI.setMISO(39);
  SPI.setSCK(27);
  SPI.begin(); 
  SPISettings(8000000, MSBFIRST, SPI_MODE0);

//PDDR: Port Data Direction Register: 0 means this bit/pin is a input, 1 means output.
//PDIR: Port Data Input Register: 0/1 according to the pin's state.
//(If any pin is an output it can still be read by PDIR and will simply read back the same value last output on that pin).
//PDOR: Port Data Output Register: 0 sets the bit/pin to 0, 1 to 1.

  setPortC(0, 0); //Set lower 12 bits of PortC as Recording Inputs, no pullup (default on boot)
  setLight(0, 0); //Set all 8 bits of Light sensors to input

  pinMode(GPIO1, OUTPUT);				//GPIO 1

  pinMode(fanControl, OUTPUT);				//Fan control
  //digitalWrite(fanControl, 1);
  
  Serial.begin(9600);

  pinMode(camSense, INPUT);         //For getting signals back from camera
  pinMode(camControl, OUTPUT);      //Signal to control camera
  digitalWrite(camControl, 0);      //Invert control for rising edge
  pinMode(userButton, INPUT);       //User button on front of unit
  
  pinMode(GPIO0, OUTPUT);
  pinMode(GPIO1, OUTPUT);

  //myTimer.begin(edge, 2083.33);   //Run interrupt at 480Hz for testing purposes

  drawLights();   
  
  //setEdgeMode(0);					//Edge mode reserved for future use
  
  attachInterrupt(digitalPinToInterrupt(logFreq), edge, RISING);
  set1X();                           //Default sample rate is 125Hz based off controller timing
  
  displayState = 0;
  
  TeensyDelay::begin();
  TeensyDelay::addDelayChannel(subEdge, 0); // setup a delay channel and attach the callback function to it
  //TeensyDelay::trigger(40000, 0);       //Set up the next sub-unit

  TeensyDelay::addDelayChannel(fanOff, 1); //Setup a delay channel for PWMing the fan since we ran out of I/O
 
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  
  controlSample = 0;                //Default is to start up in sampling mode 0. 1 = control mode

	clearEvents();					//Set all event memory to END statements

	delay(10);
	getConfig();						//Get defaults from EEPROM

}

void edge() {       //Master clock triggered by the 125Hz AREF coming from controller's analog triggers

	//Set sub-triggers here (if need be)

	if (subMultiplier) {                       //Are we sampling faster than the controller base rate? 
		TeensyDelay::trigger(subInterval, 0);       //Set up the next sub-unit
		subCounter = subMultiplier;              //# of sub units to call
	}

	//We need to wait samplingOffset before the hall effect sensor value will be valid for the ADC's
	//So we set a software timer and a flag to do the actual sampling 400us after the interrupt

	triggerCounter = micros() + samplingOffset;  //Target is current us time + a value to make sure signal is valid
	triggered = 2;                    //Set flag to include analog triggers in sample

	if (debounceTimer) {              //Decrement this at 125Hz (if set)
		debounceTimer--;
	}

	digitalWrite(fanControl, 1);				//Pulse fan...
	TeensyDelay::trigger(fanSpeed, 1);				//...and set the timer

}

void subEdge() {    //Sub clock interrupt if you want a sampling rate faster than 125Hz

  if (--subCounter) {                      //Still have sub-units left?
    TeensyDelay::trigger(subInterval, 0);     //Set up the next sub-unit
    //Even though we don't sample analog triggers on the sub edges, still wait 400us to do the sampling so timing stays consistent
    triggerCounter = micros() + samplingOffset;       //Target is current us time + a value to make sure signal is valid
    triggered = 1;                         //Set flag for no trigger sample 
  }

}

void fanOff() {

	digitalWrite(fanControl, 0);
	
}

void loop() {		//The main code loop that runs at appx 550kHz

  //digitalWrite(GPIO1, 1);	//Get main loop timing for scope
  
  if (Serial.available() > 1) {	//A command?
    checkCommand();
  }
  
  switch (aState) {
    
    case 1 :        //Waiting for A to be pressed
      if (!(GPIOC_PDIR & aMask)) {
        aState = 2;
        debounceTimer = 15; //Appx 1/8th of a second
      }
    break;
    case 2 :        //Waiting for A to be released (debounce)
      if ((GPIOC_PDIR & aMask) and debounceTimer == 0) {    //Has X amount of time passed and the button is released?
        aState = 255; //Set button released state
      }
    break;   
    case 10:
      if (debounceTimer == 0 and digitalRead(userButton) == 1) {  //Has time elapsed AND button is released?
          aState = 0;                                             //OK to clear
      }
     break;
    
  }
 
  switch (flightType) {				//Right now just one type checked here but that could change in future
    
    case 1:       //Waiting for a button press?
      if (GPIOC_PDIR != 0x0FFF) {	//Did anything go low? (pressed)
        getTimeDigital();				//Jump to timer
      }
    break;

  }

  //Serial.print(GPIOD_PDIR, BIN);    //Read - WORKS
  //Serial.write(9);
  //Serial.println(GPIOC_PDIR, BIN);

  //GPIOx_PDOR   Set
  
  if (streamSticks) {
	  Serial.print(LX);
	  Serial.write(9);
	  Serial.print(LY);
	  Serial.write(9);
	  Serial.print(RX);
	  Serial.write(9);
	  Serial.print(RY);
	  Serial.println("\ttype SS to exit");
  }
  
  if (menuWhich) {            //Are we currently in a menu?
    switch (menuWhich) {
    
      case 1:
        rangeAnalogs();
      break;
      
      default:
      
      break;

    }    
  }

  if (playBackRepeat > 1) {  //Flag to repeat after a set # of seconds?
    
    if (micros() > masterTimer) {	//Time to do it?
		
		switch(playBackRepeat) {	//OK what are we repeating?
		
			case 2:					//Captured sample RAM playback?
				playBackRepeat = 1;   //Flag so this won't re-trigger
				startPlayback();
			break;
			case 3:					//Programmed event playback?
				playBackRepeat = 1;   //Flag so this won't re-trigger
				beginEvent();			
			break;
		
		}

    }

  }
 
  if (digitalRead(userButton) == 0 and aState == 0 and debounceTimer == 0) {  //User button pressed?
    debounceTimer = 50;
    aState = 10;
    userFunc();							//Call whatever function button is currently assigned to
  }
  
  if (triggered) {                      //Time to sample data?

	switch(controlSample) {
	
		case 1:								//Playback control?
			if (userCode) {
				userSampled();
			}		
			digitalWrite(GPIO0, 1);
			controlCycle();                 //Do this immediately (no delay)
			digitalWrite(GPIO0, 0);
			triggered = 0;                    //Reset and wait for next controller cycle					
		break;
		
		case 2:								//Event Control?
			if (--timer == 0) {				//Did we reach an event point?				
				Serial.print("Event #");
				Serial.println(eventCounter + 1);
				event[eventCounter]();				//Jump to specified function and do changes
				eventCycle();						//Send changes to controller
				timer = eventEdge[eventCounter];   //Set timer for how long before event changes
				
				eventCounter++;						//Increment counter
				
				if (flightType == 2) {				//Are we looking for flight time per event?
					getTimeDigitalEvent();			//Wait for that
				}	
			}	
			eventTimer++;						//Once done, increment this. THis could also execute after mode is ended but that makes no difference
			triggered = 0;					//Reset and wait for next controller cycle
		break;

		default:							//Sampling mode 0? (default state)
		  if (micros() > triggerCounter) {    //Has samplingOffsetus passed?
			//digitalWrite(GPIO0, 1);
			if (triggered == 2) {             //Is there valid analog trigger data on this sampling cycle? Get it!
			  LT = analogRead(LTsense) >> 2;    //Get samples and divide by 4
			  RT = analogRead(RTsense) >> 2;
			}      
			sampleCycle();                     //Now sample the rest (happens every cycle)
			triggered = 0;                    //Reset and wait for next controller cycle
			//digitalWrite(GPIO0, 0);         //Turn off timing waveform  
			if (userCode) {					//Call this last so it won't interfere with our sample edge timing
				userSampled();
			}
		  } 		
		break;
	
	}

  }

	if (userCode) {						//User code that happens every system cycle, if enabled
		userPolled();
	}
 
  //digitalWrite(GPIO1, 0);
 
}  

void userBegin() {	//Command UB calls this function to start executing User Code

	Serial.println("Begin User Code");
	
}

void userPolled() {	//If User Code is enabled this function will be called every system cycle appx 550kHz
	
}

void userSampled() {	//If User Code is enabled this function will be called every sample frame (synced to controller)
	
}

void userEnd() {	//Command UE calls this function to stop executing User Code. Do you clean-up here

	Serial.println("End User Code");	
	
}

void showPorts() {
  
  Serial.print(GPIOC_PDIR, BIN);
  Serial.print("/t");
  Serial.println(GPIOD_PDIR, BIN);
  
}

void drawLights() {
  
  digitalWrite(shiftLoad, LOW);     //Do this early so we can time the conversion(s) using a scope

  leftAnalogs = 0;                  //Rebuild this every time
  rightAnalogs = 0;

  //Draw left trigger
  if (LT < LTedges[2]) {
    leftAnalogs |= (1 << 12);
    if (LT < LTedges[1]) {
      leftAnalogs |= (1 << 13);
      if (LT < LTedges[0]) {
        leftAnalogs |= (1 << 14);
      }        
    }         
  }

  //Draw right trigger
  if (RT < RTedges[2]) {
    rightAnalogs |= (1 << 12);
    if (RT < RTedges[1]) {
      rightAnalogs |= (1 << 13);
      if (RT < RTedges[0]) {
        rightAnalogs |= (1 << 14);
      }          
    }        
  }
  
  //Draw left analog stick 
  if (LX < LXedges[2]) {
    leftAnalogs |= (1 << 2);
    if (LX < LXedges[1]) {
      leftAnalogs |= (1 << 1);
      if (LX < LXedges[0]) {
        leftAnalogs |= 1;
      }
    }
  }  
  if (LX > LXedges[3]) {
    leftAnalogs |= (1 << 3);
    if (LX > LXedges[4]) {
      leftAnalogs |= (1 << 4);
      if (LX > LXedges[5]) {
        leftAnalogs |= (1 << 5);
      }
    }
  }     
  if (LY < LYedges[2]) {
    leftAnalogs |= (1 << 8);
    if (LY < LYedges[1]) {
      leftAnalogs |= (1 << 7);
      if (LY < LYedges[0]) {
        leftAnalogs |= (1 << 6);
      }
    }
  }  
  if (LY > LYedges[3]) {
    leftAnalogs |= (1 << 9);
    if (LY > LYedges[4]) {
      leftAnalogs |= (1 << 10);
      if (LY > LYedges[5]) {
        leftAnalogs |= (1 << 11);
      }
    }
  }

  //Draw right analog stick
  if (RX < RXedges[2]) {
    rightAnalogs |= (1 << 2);
    if (RX < RXedges[1]) {
      rightAnalogs |= (1 << 1);
      if (RX < RXedges[0]) {
        rightAnalogs |= 1;
      }
    }
  } 
  if (RX > RXedges[3]) {
    rightAnalogs |= (1 << 3);
    if (RX > RXedges[4]) {
      rightAnalogs |= (1 << 4);
      if (RX > RXedges[5]) {
        rightAnalogs |= (1 << 5);
      }
    }
  }   
  if (RY < RYedges[2]) {
    rightAnalogs |= (1 << 8);
    if (RY < RYedges[1]) {
      rightAnalogs |= (1 << 7);
      if (RY < RYedges[0]) {
        rightAnalogs |= (1 << 6);
      }
    }
  } 
  if (RY > RYedges[3]) {
    rightAnalogs |= (1 << 9);
    if (RY > RYedges[4]) {
      rightAnalogs |= (1 << 10);
      if (RY > RYedges[5]) {
        rightAnalogs |= (1 << 11);
      }
    }
  }

  //Serial.print(LX);
  //Serial.write(9);
  //Serial.println(LY);
  
  doDisplay();                    //Before we send the SPI data, build the 7 segment display (whatever state it's meant to be in)

  SPI.transfer(seg7[digits[3]]);  //Send the 7 segment display characters
  SPI.transfer(seg7[digits[2]]);  
  SPI.transfer(seg7[digits[1]]);
  SPI.transfer(seg7[digits[0]]);   

  SPI.transfer16(buttons | ~userLights << 12);     //Send the digital button states 
  SPI.transfer16(~rightAnalogs);  //Send the inverse values for analogs (since we're doing common anode LED's)
  SPI.transfer16(~leftAnalogs);    

  digitalWrite(shiftLoad, 1);     //Done sending
  
}
 
void sampleCycle() {
  
  buttons = GPIOC_PDIR;            //Get the state of the digital buttons. This also clears the top 4 bits so we can add flags
  
  LX = analogRead(LXsense) >> 2;    //Divide the 10 bit ADC by 4 to fit within a convienent byte
  LY = analogRead(LYsense) >> 2;
  RX = analogRead(RXsense) >> 2;
  RY = analogRead(RYsense) >> 2;
  
  drawLights();                   //This function will also add flags, if any
  
  if (loggingState == 1) {        //Are we supposed to be logging this? If so, log data into RAM and increment pointer
    dataLog[logP++] = LX;         //Offset 0 
    dataLog[logP++] = LY;         //Offset 1
    dataLog[logP++] = RX;         //Offset 2
    dataLog[logP++] = RY;         //Offset 3
    dataLog[logP++] = LT;         //Offset 4
    dataLog[logP++] = RT;         //Offset 5
    dataLog[logP++] = buttons >> 8;//Offset 6 high byte of buttons
    dataLog[logP++] = buttons;    //Offset 7 low byte of buttons
    dataLog[logP++] = 0;          //Offset 8 Clear the control byte
    dataLog[logP++] = GPIOD_PDIR; //Offset 9 Store the value of the light bar in the last byte
    
    if (logP > 239980) {          //Only one memory slot left? (which we need for EOF)
      Serial.print("Out of RAM - ");
      stopLogging();
    }
    
  }
  
}

void controlCycle() {	//When playing back a recording, execture this code

  if (logP == 0xFFFFFFFF) {                //Special command to manually set the digital pots?
    
    //Set pots manully with direct values
    
    Wire.beginTransmission(0x2C);

    Wire.write(B00000000);            //Write to RX 
    Wire.write(RX);
    Wire.write(B00010000);            //Write to RY
    Wire.write(RY); 

    Wire.write(B01100000);            //Write to LX
    Wire.write(LX);
    Wire.write(B01110000);            //Write to LY
    Wire.write(LY);  

    Wire.endTransmission();     // stop transmitting

    Wire.beginTransmission(0x2E);

    Wire.write(B00000000);            //Write to LT
    //Wire.write(LT);  
    Wire.write(LT);
    Wire.write(B00010000);            //Write to RT
    //Wire.write(RT);
    Wire.write(RT);    

    Wire.endTransmission();     // stop transmitting      
    //and don't do anything else
    return;
  }

  if (dataLog[logP] == 255 and dataLog[logP + 1] == 255) {           //End of recording?   
    stopPlayback();
    return;
  }

  LX = dataLog[logP];         //Offset 0 
  LY = dataLog[logP + 1];         //Offset 1
  RX = dataLog[logP + 2];         //Offset 2
  RY = dataLog[logP + 3];         //Offset 3
  LT = dataLog[logP + 4];         //Offset 4
  RT = dataLog[logP + 5];         //Offset 5
  buttons = (dataLog[logP + 6] << 8) | dataLog[logP + 7]; //Load and shift offset 6 high low and OR in low byte

  GPIOC_PDOR = ~buttons;  //Set the digital buttons (PORTC transistor control) Need to inverse as they were sampled active low but the transistors are active high

  sendPots();         //Send the data to the digital potentiometers
  drawLights();       //Draw the lights on the monitor

  if (dataLog[logP + 8] & cameraStart) {		//Bit set to start camera?
	digitalWrite(camControl, 1);	
	userLights |= lightCamera;     //Turn on Playback LED indicator
  	
  }
  else {
	digitalWrite(camControl, 0);
	userLights &= ~lightCamera;
  }
  
  //logP + 8 Camera Control?
  
  dataLog[logP + 9] = GPIOD_PDIR; //Store what's on the light sensor bar this frame. This will overwrite the logged values so we can look for changes.
  
  logP += 10;         //Jump to next index
  logSamples++;       //Increment # of samples

} 

void eventCycle() {		//Event-based controller replay mode
	

  sendPots();         //Send the data to the digital potentiometers
  drawLights();       //Draw the lights on the monitor
  GPIOC_PDOR = ~buttons;  //Set the digital buttons (PORTC transistor control) Need to inverse as they were sampled active low but the transistors are active high

  //dataLog[logP + 9] = GPIOD_PDIR; //Store what's on the light sensor bar this frame
	
}
  
int getFilename() {  //Waits for a 8 character filename from serial port
  
  int pointer = 0;
  
  while(pointer < 12) {            //Get 8.3 filename (12 characters)
  
    if (Serial.available()) {
      input[pointer++] = Serial.read();
    }

  }
  
	flush();

  if (checkFilename()) {
    return 1;
  }
  else {
    Serial.println("Invalid filename");
    return 0;
  }
  
}

int getFilenameUART() { //Gets a filename from the serial port
  
  for (int x = 0 ; x < 12 ; x++) {
    input[x] = Serial.read();
  }
  
  while(Serial.available()) {     //Flush whatever's left
    Serial.read();
  }

  if (checkFilename()) {
    return 1;
  }
  else {
    return 0;
  }
  
}

int checkFilename() {
  
  int error = 0;
  int pointer = 0;
  
  for (pointer = 0 ; pointer < 12 ; pointer++) {  //Convert lower to uppercase
    if (input[pointer] > 95) {  
      input[pointer] -= 32;
    }
  }

  if (input[8] != '.') {            //Make sure there's a period
    error++;
  }
  
  for (pointer = 0 ; pointer < 12 ; pointer++) {  //Check for valid characters
    
    if (input[pointer] > 57 and input[pointer] < 65) {
      error++;
    }
    
    if (input[pointer] < 46 or input[pointer] == 37) {
      error++;
    }    
    
  }

  if (error) {
    return 0;
  }
  
  for (pointer = 0 ; pointer < 13 ; pointer++) {  //Copy it all including terminator
    fileName[pointer] = input[pointer];
  }

  //Check and see if the filename has autonumbering and if it does, store the values into variables
  
  error = 0;
  
  for (pointer = 5 ; pointer < 8 ; pointer++) {  //Copy it all including terminator
    if (fileName[pointer] > 47 and fileName[pointer] < 58) {	//Check if the last 3 characters of the filename are numerals
		error++;
	}
  }
  
  if (error < 3) {	//They aren't all numerals. OK we're done
	  return 1;
  }
  
  //If they are all numerals convert to integer
  
	autoStart = (fileName[5] - 48) * 100;
	autoStart += (fileName[6] - 48) * 10;
	autoStart += (fileName[7] - 48);

	autoValue = autoStart;

	//Serial.println(autoValue);

  return 1;  //Return success status

}

void getTimeDigital() {				//Manual time of flight testing

    flight = micros();            //Get current us time

    while(!(GPIOD_PDIR & lightBitMask)) {  //Wait for sensor to go dark (high)     
    }
    
    flight = (micros() - flight);  //Find difference

    Serial.print(flight / 1000);            //40MHz clock, 2 cycles per count
    Serial.print(" ms\t");	
	
    Serial.print(flight);            //40MHz clock, 2 cycles per count
    Serial.print(" us\t");
    
    averageUS += (flight);
    sampleCount += 1;
    
    Serial.print("average us:");    
    Serial.println(averageUS / sampleCount);
    
    while(GPIOC_PDIR != 0x0FFF) { //Wait for button release
      delayMicroseconds(1);
    }

    Serial.println("Ready for Next Sample. Type TX or tap USER button to exit mode");
    Serial.println(" "); 
  
}

void getTimeDigitalEvent() {				//Time of flight reporting during event playback
   
    flight = micros();            //Get current us time

    while(!(GPIOD_PDIR & lightBitMask)) {  //Wait for sensor to go dark (high)     
    }
    
    flight = (micros() - flight);  //Find difference

    Serial.print(flight);            //40MHz clock, 2 cycles per count
    Serial.print(" us\t");
    
    averageUS += (flight);
    sampleCount += 1;
    
    Serial.print("average us:");    
    Serial.println(averageUS / sampleCount);

}

void exitFlightMode() {
  
  Serial.println("Digital button time of flight disabled");
  flightType = 0;
  userFunc = &startLogging;  
  
}

void checkCommand() {

	char op0 = Serial.read();
	char op1 = Serial.read();

  if (op0 == '/') {       //Special debugging commands (more than 2 characters)

    char op2;
  
    if (op1 == 'F') {     //Special immediate file command?
 
    //  /FLxxxxxxxx.CSV   //Load this file into RAM
    //  /FSxxxxxxxx.CSV   //Save RAM to this file
		op2 = Serial.read(); 	//Get next character
		
      switch(op2) {
        
        case 'L':                       //Load file from SD card?
          if (getFilenameUART()) {
            if (SD.exists(fileName)) {  //Is file real?
              loadRAM();
            }
            else {
              Serial.print(fileName);
              Serial.println(" file not found");
            }
          }
        break;
        
        case 'S':
          if (getFilename()) {          //Set a filename and make sure it's valid
            if (SD.exists(fileName)) {  //Does file exist? Mention that we'll overwrite it
              Serial.print(fileName);
              Serial.println(" will be overwritten on SD card.");
            }
            else {
              Serial.print(fileName);
              Serial.println(" will be created on next RAM export.");
            }
          } 
        break;
  
      }
      
    }
    
    if (op1 == 'U') {     //UART transfer command?
 
		op2 = Serial.read(); 	//Get next character
 
      switch(op2) {
        
          case 'R':       //Load raw data from UART to RAM
            loadRAWfromUART();          
          break;
          
          case 'C':       //Send CSV via UART and have MCU parse it into RAM (slower)
            parseCSVfromUART();  
          break;

      }
 
    }

	if (op1 == 'P') {		//Manual potentiometer command?
	
		manualPot();
	
	}
	
	if (op1 == 'C') {		//Cooling speed for fan?
	
		op2 = Serial.read(); 	//Get next character
	
		if (op2 > '1' and op2 < '8') {	//Meeds at least 2000 to spin up, but don't exceed 8000
		
			op2 -= 48;
			EEPROM.write(99, op2);				//Store to EEPROM
			fanSpeed = (op2 * 1000) - 100;		//Sub 100us here so it triggers before the subEdges do
			Serial.print("Fan duty cycle = ");
			Serial.print(fanSpeed + 100);		//Lie about the speed
			Serial.println(" / 8000");
		
		}
		else {
			Serial.println("Fan speeds must be 2-7");
		}
	}	
	
	if (op1 == 'A') {		//Analog replay ranging command?
	
		rangingPot();
		
	}

    while(Serial.available() > 0) {     //Flush the buffer (get rid of New Line Carriage Return stuff)
      Serial.read();
    }    
    
    return;
	
  }
  
	if (op0 > 96) {						//Convert to upper-case
		op0 -= 32;
	}
	if (op1 > 96) {
		op1 -= 32;
	}	

  while(Serial.available() > 0) {     //Flush the buffer (get rid of New Line Carriage Return stuff)
    Serial.read();
  }    

	if (op0 == 'H' and op1 == 'P') {	//Show help commands
		showCommands();
	}   
  
	if (op0 == 'T' and op1 == 'T') {	//Test analog lights?
		Serial.print("Shift register TEST...");
		
		for (int x = 0 ; x < 4 ; x++) {
			commAll(0xAA);
			delay(1000);
			commAll(0x55);
			delay(1000);      
		}
		
		commAll(0x00);
		delay(500);
		commAll(0xFF);
		delay(500); 
		Serial.println(" done!");
	}	

  if (op0 == 'D') {     //LED display timer settings

    timer = 0;          //Any command here resets the timer

    switch(op1) {

      case 'S':
        displayFrames = 0;
        Serial.println("LED display set to show seconds elasped or remaining");
      break;
      
      case 'F':
        displayFrames = 1;
        Serial.println("LED display set to show sample frames elapsed or remaining");
      break;
      
      case 'H':
        Serial.println("Frame display set to HEX");
        hexDecimal = 0;
      break;
      
      case 'D':
        Serial.println("Frame display set to DECIMAL");
        hexDecimal = 1;
      break;  
    }

  }

  if (op0 == 'C') {     //Camera control
    
    switch(op1) {
      
      case 'E':
        Serial.println("Pulse sent");
        digitalWrite(camControl, 1);     //Invert control for rising edge
        delay(1);
        digitalWrite(camControl, 0);      //Invert control for rising edge
      break;        
      
    }

  }

  if (op0 == 'E') {     //Set what edge triggers sampling?
      if (op1 == '0') {
        setEdgeMode(0);
      }
      if (op1 == '1') {
        setEdgeMode(1);
      }      
  }
 
  if (op1 == 'X') {   //Change sampling rate?
      if (op0 == '1' or op1 == '0') {
          set1X();
          Serial.println("Sampling/logging frequency set to 125Hz");
          Serial.print("Availble logging RAM: ");
          Serial.print(getTimeLeft());
          Serial.println(" seconds");
      }
      if (op0 == '2') {
          set2X();
          Serial.println("Sampling/logging frequency set to 250Hz");
          Serial.print("Availble logging RAM: ");
          Serial.print(getTimeLeft());
          Serial.println(" seconds");
      }
      if (op0 == '4') {
          set4X();
          Serial.println("Sampling/logging frequency set to 500Hz");
          Serial.print("Availble logging RAM: ");
          Serial.print(getTimeLeft());
          Serial.println(" seconds");
      }      
  }

  if (op0 == 'L') {         //Logging stuff?

    switch(op1) {
      
      case 'G':
        if (loggingState == 0) {
          startLogging();
        }
        else {
          stopLogging();
        }        
      break;

      case 'A':
        Serial.println("Automatic RAM save after capture = ON");
        writeAfterLogging = 1;      
      break;
      
      case 'M':
        Serial.println("Automatic RAM save after capture = OFF");
        writeAfterLogging = 0;      
      break;

    }

  }

	if (op0 == 'P') {         //Playback stuff?
    
    switch(op1) {
      
      case 'B':
        if (loggingState == 0) {
          startPlayback();
        }
        else {
          stopPlayback();
        }      
      break;
      
      case 'X':         //Cancel automatic playback repeat?
        playBackRepeat = 0;
        Serial.println("Automatic playback repeating stopped");
        Serial.println("Issue a P0...P9 command to re-enable");
      break;
	  
	  case 'S':			//Enable sequential file loading for playback loops.
		seqFile = 1;
		Serial.print("Looped playback will load files sequentially starting with ");
		Serial.println(fileName);	  
	  break;
	  
 	  case 'R':			//Disable sequential file loading for playback loops.
		seqFile = 0;
		Serial.println("Looped playback will repeat using RAM.");	  
	  break;   
	  
      default:          //0-7 time delay repeats
      
        op1 -= 48;          //ASCII to integer convert
        
        if (op1 < 10) {      //Valid range?
          playBackRepeat = 1;
          playBackDelay = op1;
          Serial.print("Automatic Playback Repeat enabled with ");
          Serial.print(playBackDelay);
		  Serial.println(" seconds gap between loops.");
          Serial.println("Type PB or tap USER button to start looping. Type PX to stop");
        }
        else {
          Serial.println("Invalid Playback Timer Delay setting");
        }
            
      break;
      

    }
  }

  if (op0 == 'A') {       //Analog stick config?
    
    switch(op1) {
      
      case 'P':
        showRanges();
      break;
      
      case 'V':
        menuWhich = 1;    			//Trigger ranging menu
        menuState = 0;    			//Start of menu
		userLights = lightConfig;     //Turn on Playback LED indicator        
      break;

    }
   
  }  

  if (op0 == 'T') {           //Time of flight stuff?
  
    switch(op1) {
      
      case 'B':
        userFunc = &exitFlightMode;
        Serial.print("Ensure light sensor #");
        Serial.print(lightMaskNumber);
        Serial.println(" is over a square that will transistion from WHITE to BLACK");
        Serial.println("");
        Serial.println("Press any digital controller button to start");
        flightType = 1;       //Trigger on any low button edge (digital controller buttons)  
      break;
 
      case 'E':
        userFunc = &exitFlightMode;
        Serial.print("Ensure light sensor #");
        Serial.print(lightMaskNumber);
        Serial.println(" is over a square that will transistion from WHITE to BLACK");
        Serial.println("");
        Serial.println("Type EB to begin event playback. Time of flight will be displayed after each event.");
        flightType = 2;       //Trigger on any low button edge (digital controller buttons)  
      break;
 
      case 'X':
        exitFlightMode();
      break;
      
      case 'R':
        averageUS = 0;
        sampleCount = 0;
        Serial.println("Time of Flight sample average reset");    
      break;
      
      default:              //Use this for 0-7 entries
        op1 -= 48;          //ASCII to integer convert
        if (op1 < 8) {      //Valid range?
          lightMaskNumber = op1;
          lightBitMask = (1 << op1);
          Serial.print("Time of Flight detection set to Sensor #");
          Serial.println(lightMaskNumber);
        }
        else {
          Serial.println("Invalid Light Sensor #");
        }
      
      break;
    
    }  
  }
  
  if (op0 == 'F') {           //File stuff?
    
      switch(op1) {
        
        case 'D': //Print directory
          listFiles();
        break;
        
        case 'L': //Select file to load from
          Serial.println("Please enter the 8.3 filename to load into RAM");
          if (getFilename()) {    //Get a filename and make sure it's valid
            if (SD.exists(fileName)) {  //Is file real?
              loadRAM();
            }
            else {
              Serial.print(fileName);
              Serial.println(" file does not exist. To create this file, type LG to log data");
            }
          }
        break;
        
        case 'S': //Set filename to use as save target
          Serial.println("Please enter the 8.3 filename for saving to SD card");
          Serial.println("If using auto-numbering the last 3 digits will be overwritten.");
          if (getFilename()) {    //Get a filename and make sure it's valid
            if (SD.exists(fileName)) {  //Is file real? Mention that we'll overwrite it
              Serial.print(fileName);
              Serial.println(" will be overwritten on SD card.");
            }
            else {
              Serial.print(fileName);
              Serial.println(" will be created on next RAM export.");
            }
          }
        break;        

        case 'I':             //Auto file numbering controls
          autoNumber = 1;
          Serial.println("Auto file numbering = ON");            
        break;
       
        case 'N':
          autoNumber = 0;
          Serial.println("Auto file numbering = OFF");                 
        break;

        case '0':
          autoValue = 0;
          Serial.println("Automatic file number reset to 0");                 
        break;        
    
        case 'R':             //Export as RAW data?
          sendType = RAWfile;
          Serial.println("RAM export format = RAW");
        break;    
        
        case 'C':             //Export as CSV?
          sendType = CSVfile;
          Serial.println("RAM export format = CSV");
        break;

        case 'U':             //Send data to serial port?
          sendWhere = sendUART;
          Serial.println("RAM export target = UART");
        break;
        
        case 'F':             //Send data to file on SD card?
          sendWhere = sendSD;
          Serial.println("RAM export target = SD card file");
        break;   

        case 'M':             //Manually convert RAM to CSV file
          saveRAM();
        break;        
        
      }

  }

	if (op0 == 'S') {		//Status report stuff?

		switch(op1) {
			
			case 'T':
				showStatus();			
			break;			
			case 'S':					//Stream the analog stick values over UART?
				if (streamSticks) {
					streamSticks = 0;
					Serial.println("Analog stick stream stopped.");
				}
				else {
					streamSticks = 1;
				}			
			break;

		}
	
	}   

	if (op0 == 'U') {				//User code commands?
	
		switch(op1) {
			
			case 'B':				//Begin user code?
				userCode = 1;
				userBegin();
			break;
			
			case 'E':				//End user code?
				userCode = 0;
				userEnd();
			break;

		}
	
	}

	if (op0 == 'E') {				//Event programming commands?
	
		switch(op1) {
  
			case '0':				//Standard edge trigger
				setEdgeMode(0);
			break;
			
			case '1':				//Light sensor edge trigger (future use)
				setEdgeMode(1);
			break;
			
			case 'B':				//Begin playback
				beginEvent();
			break;
			
			case 'P':				//Enter programming mode
				showEventCodes();
				eventProgramming();
			break;

			case 'E':				//End playback
				lastEvent();
			break;

			
		}
		
	}

}

void showCommands() {

	Serial.println(" ");
	Serial.println("-------------------------------------------------------");  
	Serial.println("Xbox One Controller Monitor Version 2 Terminal Commands");
	Serial.println("  www.benheck.com/xbox1monitor2 for more information");
	Serial.println("-------------------------------------------------------");
	Serial.println(" ");

	Serial.println("---------CONFIGURATION----------------------");   
	Serial.println("1X - Set sampling/logging frequency to 125Hz");
	Serial.println("2X - Set sampling/Logging frequency to 250Hz");
	Serial.println("4X - Set sampling/Logging frequency to 500Hz");
	//Serial.println("E0 - Use controller sync (default 125Hz)");
	//Serial.println("E1 - Use Light Sensor 7 monitor sync (triggers on transistion to black)");  
	Serial.println("AV - Sample and store Analog Stick/Trigger Values");
	Serial.println("AP - Display current Analog Stick/Trigger Values");
	Serial.println("DS - Show seconds (elapsed or remaining) on LED display");
	Serial.println("DF - Show # of sample frames on LED display");
	Serial.println("DH - Show frames in HEX");
	Serial.println("DD - Show frames in DECIMAL"); 
	Serial.println("HP - Help, show commands list");
	Serial.println("ST - Show current system settings");    
	Serial.println("SS - Stream the current analog stick data over serial port"); 
	
	Serial.println("---------CAMERA CONTROL---------------------");  
	Serial.println("CE - Send pulse to camera");
	
	Serial.println("---------TIME OF FLIGHT DETECTION-----------");  
	Serial.println("T0...T7 Select which Light Sensor will be used for Time of Flight detection");
	Serial.println("TB - Display time for any digital button (A B X Y L3 R3 LB RB D-pad)");
	Serial.println("TE - Display time after each Programmed Event");
	Serial.println("TX - Disable Time-Of-Flight Timers");
	Serial.println("TR - Reset sample time average value");

	Serial.println("---------FILE & FORMAT HANDLING-------------");
	Serial.println("FD - List files on SD card");
	Serial.println("FL - Select file and load into Playback RAM");
	Serial.println("FS - Set filename when storing RAM to SD card");
	Serial.println("FI - Enable auto-incrementing of selected filename XXXXX000.csv, XXXXX001.csv etc");
	Serial.println("FN - Disable auto-incrementing of filenames. Files will be overwritten unless new name specified.");
	Serial.println("F0 - (F zero) Reset automatic file number to 000");

	Serial.println("FR - (Format) Playback RAM will be exported as raw data file");
	Serial.println("FC - (Format) Playback RAM will be exported as CSV file");
	Serial.println("FU - (Target) Playback RAM will be sent to this UART port");
	Serial.println("FF - (Target) Playback RAM will be sent to SD card");
	Serial.println("FM - (Saving) Export current Playback RAM to target (SD or serial)");  

	Serial.println("---------INPUT CAPTURE CONTROLS-------------");
	Serial.println("LG - Start/stop data capture to RAM");
	Serial.println("LA - (Log Saving) Automatically save captured RAM when complete");
	Serial.println("LM - (Log Saving) Manually save captured RAM. Use command FM to save");
	Serial.println("PB - Start/stop data playback from RAM");  
	Serial.println("P0...P9 - Enable automatic playback loops with a delay of 0-9 seconds between loops");  
	Serial.println("PX - Stop automatic playback loops");
	Serial.println("PS - Playback loops will use sequential file loading (start point specified with FL command)");
	Serial.println("PR - Playback loops will repeat what's in playback RAM");
	
	Serial.println("---------EVENT PROGRAMMING CONTROLS---------");
	Serial.println("EP - Enter Programming mode for up to 64 timed events");  
	Serial.println("EB - Begin playback of programmed timed events. Can be auto-looped using above P0..P9 commands");
	Serial.println("EE - End playback of programmed timed events.");
	
	Serial.println("---------USER CODE--------------------------");	
	Serial.println("UB - Jumps to the User Code Begin function and enables polled User Code");  
	Serial.println("UE - Jumps to the User Code End function and disables polled User Code");

}

void showEventCodes() {

	Serial.println(" ");
	Serial.println("---------------EVENT CODES---------------------");  
	Serial.println("Analog sticks: LAU\tLAD\tLAL\tLAR\tLAC\tRAU\tRAD\tRAL\tRAR\tRAC");
	Serial.println("Analog triggers: LTP\tLTR\tRTP\tRTR");
	Serial.println("Face buttons: BAP\tBAR\tBBP\tBBR\tBXP\tBXR\tBYP\tBYR");
	Serial.println("D-pad: DUP\tDUR\tDDP\tDDR\tDLP\tDLR\tDRP\tDRR");	
	Serial.println("Shoulder/L3/R3: LBP\tLBR\tRBP\tRBR\tL3P\tL3R\tR3P\tR3R");
	Serial.println("Set final line: END");
	Serial.println("Keywords:");
	Serial.println("NEW - clears memory");
	Serial.println("LIST - lists all events LIST XX - lists single event");
	Serial.println("CODE - displays this help");
	Serial.println("EXIT - when done programming");
	Serial.println(" ");
	
	Serial.println("Event Programming Syntax:");
	Serial.println("(event number)(space)(event code)(space)(number of ticks to hold event)");
	Serial.println("Example: 1 BAP 125(press enter) = Press A button and wait 1 second");
	Serial.println("Example: 2 BAR 250(press enter) = Release A button and wait 2 seconds");
	Serial.println("Example: 3 END 100(press enter) = Ends event playback");	
	
}

void flush() {			//Flush whatever remains in serial buffer (usually to eat NL CR's)
	
  while(Serial.available()) {     //Flush whatever's left
    Serial.read();
  }	
	
}

void showStatus() {
  
    Serial.println("STATUS REPORT:");
    
    Serial.print("Capture Freqeuncy = ");
    Serial.print(secondsDivider);
    Serial.println("Hz");
    
    if (autoNumber) {
      Serial.println("Autonumbering = ON current # = ");
      Serial.println(autoValue);
    }
    else {
      Serial.println("Autonumbering = OFF");
    }
  
    Serial.print("Automatic RAM dump after capture = ");
    if (writeAfterLogging) {
      Serial.println("OFF");
    }
    else {
      Serial.println("ON");
    }      
    showRAMtargets();
    Serial.print("Current CSV filename target = ");
    if (autoNumber) {
      fileNameNumber();           //This only gets changed when we save to CSV, so do it here so we'll know what next one is
    }
    Serial.println(fileName); 

    Serial.print("Automatic Playback Looping = ");
    if (playBackRepeat) {
      Serial.print(playBackDelay);
      Serial.println(" seconds");
    }
    else {
      Serial.println("OFF");
    }
	
    Serial.print("Sequential file loading = ");
    if (seqFile) {
      Serial.println("ON");
    }
    else {
      Serial.println("OFF");
    }	

    Serial.print("Time of Flight detection = SENSOR_");
    Serial.println(lightMaskNumber);

}

void doDisplay() {    //Decide what should go on the 7-segment display (called from main loop every interrupt)
 
  switch(displayState) {
    
    case 0 :  //Default timer   
      if (displayFrames) {
          if (hexDecimal) {
            if (++timer > 9999) {
              timer = 0;
            }
            drawDecimal(timer);
          }
          else {
              drawHex(timer++);
          }     
      }
      else {
        timer++;
        drawDecimal(timer/secondsDivider);  //Set   
      }     
    break;
    case 1 :  //Show how much time is left for RAM logging
      timer++;
      drawDecimal(getTimeLeft());
    break;
    case 2 :  //Show how much time/frames has elapsed for playback
      if (displayFrames) {
          if (hexDecimal) {
            if (timer > 9999) {
              timer = 0;
            }
            drawDecimal(timer++);
          }
          else {
            drawHex(logSamples);
          } 
      }
      else {
        drawDecimal(logSamples / secondsDivider);  
      }   
    break;
	case 3:
		drawDecimal(eventCounter + 1);			//"E xx"
		digits[0] = 14;		//Draw an E
		digits[1] = 16;		//Draw a blank
	break;
	case 4:										//"PROG" for event programming mode
		digits[0] = 17;
		digits[1] = 18;
		digits[2] = 19;
		digits[3] = 20;			
	break;
  }

}

void drawDecimal(uint16_t theValue) {        //Draws a 4 digit decimal number on the 7 segment display
 
  int numerals = 0;           //Always going to be at least 1 numeral
  int zPad = 1;             //Flag for zero padding 
  unsigned long divider = 1000;     //Divider starts at 100k
    
  for (int xx = 0 ; xx < 4 ; xx++) {    //6 digit number    
    if (theValue >= divider) {
      digits[numerals] = (theValue / divider);
      theValue %= divider;
      numerals += 1;
    }   
    else if (zPad or divider == 1) {
      digits[numerals] = 0;
      numerals += 1;
    } 
    divider /= 10;            
  } 
 
}

void drawHex(uint16_t theValue) {

  digits[0] = (theValue >> 12) & 0x0F;
  digits[1] = (theValue >> 8) & 0x0F;
  digits[2] = (theValue >> 4) & 0x0F;
  digits[3] = theValue & 0x0F;  
  
}

void eventProgramming() {			//Manually enter commands into Event Programming

	userLights = lightConfig;
	displayState = 4;				//Display a wonky looking "Prog"
	
	drawLights();
	
	while(1) {

		for (int x = 0 ; x < 12 ; x++) {	//Clear the buffer so we have a zero terminator in case of no CR LF
			input[x] = 0;
		}
	
		while(Serial.available() == 0) {}	//Wait for something to show up
		delay(100);						//Give buffer time to fill	
		
		switch(parseLine()) {

			case 0:			//Line parsed OK?	
				printSingleEvent(eventCounter);		//Print that line back to user
				
				Serial.println(" OK");
				
				if (eventCounter == 62) {			//Did user enter command #63? Remind them that 64 must be THE END				
					Serial.println("Event memory full, setting Event #64 to END");
					eventEdge[63] = 0;			//A default gap of 1 second
					eventASCII[63] = 0x00454E44;	//Make sure final event is END
					event[63] = &lastEvent;		//Make sure final event is END			
				}			
			break;
	
			case 1:
				Serial.println("ERROR - Syntax");	
			break;
			
			case 2:
				Serial.println("ERROR - Time value must be more than 0 and less than 65536");	
			break;

			case 3:
				Serial.println("ERROR - Invalid opcode. Type CODES to see a list of them");	
			break;
			
			case 4:
				Serial.println("ERROR - Invalid event #. Must be 1-63. Event 64 is always END");	
			break;			
			
			case 255:
				Serial.println("Exiting programming mode. Type EB to begin event playback.");
				eventASCII[63] = 0x00454E44;	//Make sure final event is END
				event[63] = &lastEvent;		//Make sure final event is END
				userLights = lightOff;
				displayState = 0;
				return;
			break;
			
			default:
			
			break;
	
		}
		
	}
	
}

int parseLine() {					//Parse one line from buffer

	int pointer = 0;
	uint8_t eventLine = 0;			//Stores what line number user just entered
	uint32_t eventTime = 0;		//Stores how much time this event should take

	while(pointer < 12) {            //Get up to 12 characters (don't overflow input buffer)

		if (Serial.available()) {		//Something there?
		  input[pointer] = Serial.read();	//Copy into buffer

		  if (input[pointer] > 95) {	//Convert lower to uppercase
			  input[pointer] -= 32;
		  }
		  
		  if (input[pointer] == 10 or input[pointer] == 13) {	//Carriage return or line feed?
			  break;
		  }
		  else {
			  pointer++;			//Advance value
		  }
		  
		}
		else {
			break;
		}

	}

	flush();

	if (input[0] > 57) {		//First character NOT a number? Check for reserved word
	
		switch(input[0]) {
		
			case 'N':			//New file (erase all contents)
				clearEvents();
				Serial.println("Event memory cleared.");
			break;
			
			case 'L':			//List event lines
			
				if (input[5]) {	//More than just the word LIST? List a specific line
					sPointer = 5;	//Start looking for a number here
					printSingleEvent(getNumber() - 1);
					Serial.println("");
				}
				else {
					listEvents(0);
				}
				
			break;

			case 'C':			//List event opcodes
				showEventCodes();
			break;
			
			case 'E':			//Exit programming
				return 255;
			break;
			
			default:			//If it doesn't meet these conditions return error state
				return 1;
			break;

		}
		
		return 254;				//Line OK but don't re-print
	
	}
	
	sPointer = 0;
	
	eventLine = getNumber();			//Parse what number is at start of line
	
	if (eventLine < 1 or eventLine > 63) {		//Must be a valid range else error
		return 4;
	}

	sPointer++;								//Jump pass the space
	
	//OK now parse the 3 character opcode

	whatCode = input[sPointer] << 16;		//Cram the ASCII codes into a 32 bit number
	int endCheck = input[sPointer++];		//Flag if event == END
	whatCode |= input[sPointer++] << 8;
	whatCode |= input[sPointer++];		
	
	if (endCheck != 'E') {					//An END command? Then don't parse the last number. Else parse the final number as TIME

		if (input[sPointer++] != ' ') { 		//If the opcode isn't followed by a space return error
			return 0;
		}	

		eventTime = getNumber();			//Parse the final number in the line (event time)

		if (eventTime < 1 or eventTime > 65534) {	//Can't be 0 seconds or more than a 16 bit number
			return 2;
		}
	
	}

	eventCounter = eventLine - 1;				//Set the zero offset and update this so scanCode can use it to set pointer
	
	scanCode();									//Check if that opcode is valid

	if (whatCode == 0) { 					//If the opcode was invalid return error
		return 3;
	}
	
	//OK line is good! Let's put it in memory (scanCode will have done part of this if opcode OK)
	
	eventEdge[eventCounter] = eventTime;		//Set time gap
	eventASCII[eventCounter] = whatCode;		//Array that holds ASCII opcodes. For LISTing purposes

	return 0;			//Parse success
	
}

uint16_t getNumber() {				//Parses number from serial string at current sPointer memory location

	uint8_t startPoint = sPointer;				//Store where we started at

	while(input[sPointer] > 47) {				//Keep advancing the sPointer until we find a space, CR LF or zero terminator (anything lower than ASCII 0)
		if (sPointer++ > 12) {					//If a space isn't found return error state
			return 65535;
		}	
	}

	int16_t value = 0;
	int16_t multiplier = 1;
	uint8_t localPointer = sPointer - 1;		//Put new pointer at last character digit
	
	while(1) {					//Turns the input into a number

		//Serial.println(input[localPointer]);
	
		if (input[localPointer] < 48 or input[localPointer] > 57) {	//Must be 0-9 or return error
			return 65535;
		}

		value += ((input[localPointer] - 48) * multiplier);				//Convert digit
		
		multiplier *= 10;				//Next power of 10
		
		if (localPointer-- == startPoint) {			//Done?
			return value;
		}

	}
	
}

void clearEvents() {			//Clear events (set them all to Event End

	for (int x = 0 ; x < 64 ; x++) {	//Set all events as END
		eventASCII[x] = 0x00454E44;
		event[x] = &lastEvent;	
	}
	
	eventCounter = 255;			//Flag that there is no data
	
}

void listEvents(uint8_t fromWhere) {

	Serial.println("Program Listing:");

	while(fromWhere < 64) {
		if (printSingleEvent(fromWhere) == 0x00454E44) {	//If we printed an END line then we're done
			Serial.println("");
			return;
		}
		Serial.println("");
		fromWhere++;		//Advance
	}
	
}

uint32_t printSingleEvent(uint8_t whichEvent) {				//Prints a single event line from memory

	if (whichEvent > 63) {
		Serial.println("ERROR - Invalid line number");
	}

	Serial.print(whichEvent + 1);		//Show line number
	Serial.write(32);					//Space
	Serial.write((eventASCII[whichEvent] >> 16) & 0xFF);	//Draw the ASCII characters of the opcode
	Serial.write((eventASCII[whichEvent] >> 8) & 0xFF);	//Draw the ASCII characters of the opcode		
	Serial.write(eventASCII[whichEvent] & 0xFF);			//Draw the ASCII characters of the opcode		
	Serial.write(32);					//Space
	Serial.print(eventEdge[whichEvent]);
	
	return eventASCII[whichEvent];

}

void scanCode() {

	switch(whatCode) {			//We stuff the 3 byte command into the bottom of a long and switch-case it using HEX
		
		case 0x004C4155:		//LAU Left Analog Up
			event[eventCounter] = &eventLup;
		break;
		case 0x004C4144:		//LAD Left Analog Down
			event[eventCounter] = &eventLdown;
		break;		
		case 0x004C414C:		//LAL Left Analog Left
			event[eventCounter] = &eventLleft;
		break;
		case 0x004C4152:		//LAR Left Analog Right
			event[eventCounter] = &eventLright;
		break;		
		case 0x004C4143:		//LAR Left Analog Centered
			event[eventCounter] = &eventLcenter;
		break;			
		
		case 0x00524155:		//RAU Right Analog Up
			event[eventCounter] = &eventRup;
		break;
		case 0x00524144:		//RAD Right Analog Down
			event[eventCounter] = &eventRdown;
		break;		
		case 0x0052414C:		//RAL Right Analog Left
			event[eventCounter] = &eventRleft;
		break;
		case 0x00524152:		//RAR Right Analog Right
			event[eventCounter] = &eventRright;
		break;	
		case 0x00524143:		//RAR Right Analog Centered
			event[eventCounter] = &eventRcenter;
		break;	
		
		//Analog triggers

		case 0x004C5450:		//Left trigger pulled
			event[eventCounter] = &eventLTp;
		break;
		case 0x004C5452:		//Left trigger released
			event[eventCounter] = &eventLTr;
		break;		
		case 0x00525450:		//Right trigger pulled
			event[eventCounter] = &eventRTp;
		break;
		case 0x00525452:		//Right trigger released
			event[eventCounter] = &eventRTr;
		break;	
		
		//D-pad
		
		case 0x00445550:		//DU pressed
			event[eventCounter] = &eventDUon;
		break;
		case 0x00444450:		//DD pressed
			event[eventCounter] = &eventDDon;
		break;		
		case 0x00444C50:		//DL pressed
			event[eventCounter] = &eventDLon;
		break;
		case 0x00445250:		//DR pressed
			event[eventCounter] = &eventDRon;
		break;			
		
		case 0x00445552:		//DU released
			event[eventCounter] = &eventDUoff;
		break;
		case 0x00444452:		//DD released
			event[eventCounter] = &eventDDoff;
		break;		
		case 0x00444C52:		//DL released
			event[eventCounter] = &eventDLoff;
		break;
		case 0x00445252:		//DR released
			event[eventCounter] = &eventDRoff;
		break;	

		//Face buttons
		case 0x00424150:		//A pressed
			event[eventCounter] = &eventAon;
		break;
		case 0x00424250:		//B pressed
			event[eventCounter] = &eventBon;
		break;		
		case 0x00425850:		//X pressed
			event[eventCounter] = &eventXon;
		break;
		case 0x00425950:		//Y pressed
			event[eventCounter] = &eventYon;
		break;			
		
		case 0x00424152:		//A released
			event[eventCounter] = &eventAoff;
		break;
		case 0x00424252:		//B released
			event[eventCounter] = &eventBoff;
		break;		
		case 0x00425852:		//X released
			event[eventCounter] = &eventXoff;
		break;
		case 0x00425952:		//Y released
			event[eventCounter] = &eventYoff;
		break;	

		//L3 R3 shoulders
		case 0x004C3350:		//L3 pressed
			event[eventCounter] = &eventL3on;
		break;
		case 0x00523350:		//R3 pressed
			event[eventCounter] = &eventR3on;
		break;		
		case 0x004C4250:		//LB pressed
			event[eventCounter] = &eventLBon;
		break;
		case 0x00524250:		//RB pressed
			event[eventCounter] = &eventRBon;
		break;			
		
		case 0x004C3352:		//L3 released
			event[eventCounter] = &eventL3off;
		break;
		case 0x00523352:		//R3 released
			event[eventCounter] = &eventR3off;
		break;		
		case 0x004C4252:		//LB released
			event[eventCounter] = &eventLBoff;
		break;
		case 0x00524252:		//RB released
			event[eventCounter] = &eventRBoff;
		break;	

		case 0x00454E44:		//END?
			event[eventCounter] = &lastEvent;
			//whatCode = 0xFFFFFFFE;	//Flag for exit programming
		break;

		case 0x00434F44:		//COD? (to see list)
			showEventCodes();
			//whatCode = 0xFFFFFFFF;	//Flag for null operation
		break;
		
		default:
			whatCode = 0;		//Error flag
		break;
	
		
	}	
	
}	

void eventLTp() {		//Event function pointer targets

	LT = LTp;			//Set to Left Trigger Pulled
	
}

void eventLTr() {
	
	LT = LTr;			//Release Left Trigger
	
}

void eventRTp() {

	RT = RTp;			//Set to Left Trigger Pulled
	
}

void eventRTr() {
	
	RT = RTr;			//Release Left Trigger
	
}

void eventLup() {

	LY = LYu;
	LX = LXc;
	
}

void eventLdown() {

	LY = LYd;
	LX = LXc;
	
}

void eventLleft() {
	
	LX = LXl;
	LY = LYc;

}

void eventLright() {
	
	LX = LXr;
	LY = LYc;
	
}

void eventLcenter() {

	LX = LXc;
	LY = LYc;
	
}

void eventRup() {

	RY = RYu;
	RX = RXc;
	
}

void eventRdown() {

	RY = RYd;
	RX = RXc;
	
}

void eventRleft() {
	
	RX = RXl;
	RY = RYc;

}

void eventRright() {
	
	RX = RXr;
	RY = RYc;
	
}

void eventRcenter() {

	RX = RXc;
	RY = RYc;
	
}

void eventAon() {

	buttons &= ~(1 << 1);
	
}	

void eventAoff() {

	buttons |= (1 << 1);
	
}

void eventBon() {

	buttons &= ~(1 << 2);
	
}	

void eventBoff() {

	buttons |= (1 << 2);
	
}

void eventXon() {

	buttons &= ~(1 << 3);
	
}	

void eventXoff() {

	buttons |= (1 << 3);
	
}

void eventYon() {

	buttons &= ~(1 << 4);
	
}	

void eventYoff() {

	buttons |= (1 << 4);
	
}

void eventDUon() {

	buttons &= ~(1 << 5);
	
}	

void eventDUoff() {

	buttons |= (1 << 5);
	
}

void eventDDon() {

	buttons &= ~(1 << 6);
	
}	

void eventDDoff() {

	buttons |= (1 << 6);
	
}

void eventDLon() {

	buttons &= ~(1 << 7);
	
}	

void eventDLoff() {

	buttons |= (1 << 7);
	
}

void eventDRon() {

	buttons &= ~(1 << 8);
	
}	

void eventDRoff() {

	buttons |= (1 << 8);
	
}

void eventL3on() {

	buttons &= ~(1 << 10);
	
}	

void eventL3off() {

	buttons |= (1 << 10);
	
}

void eventR3on() {

	buttons &= ~(1 << 9);
	
}	

void eventR3off() {

	buttons |= (1 << 9);
	
}

void eventLBon() {

	buttons &= ~(1 << 11);
	
}	

void eventLBoff() {

	buttons |= (1 << 11);
	
}

void eventRBon() {

	buttons &= ~(1);
	
}	

void eventRBoff() {

	buttons |= 1;
	
}

void beginEvent() {			//Start playing back events stored in memory

	userLights = lightPlayback;     //Turn on Playback LED indicator

	set1X();			//Always play back at 125Hz
	Serial.print("Beginning Event Playback");
	if (flightType == 2) {
		Serial.println(" with Time of Flight reporting per event.");
	}
	else {
		Serial.println(".");
	}
	
	buttons = 0x0FFF;				//Set all buttons to OFF
	timer = 1;						//Set timer to 1 so first event will execute on next controller sync edge
	displayState = 3;				//Show event number on LED
	setControl(2);                  //Enter CONTROL mode, type = event
	eventCounter = 0;				//Start with event 0
				
}

void lastEvent() {			//This is always the last event called in a sequence

	userLights = lightOff;

	Serial.println("DONE");
	controlSample = 0;
	timer = 0;            //Use this to label event #'s
	displayState = 0;
	setSample();			//Go back to Sampling mode
	
  if (playBackRepeat) {     //Flag set to repeat the loop?  
    masterTimer = micros() + (playBackDelay * 1000000);   //Set future point where we will repeat
    Serial.print("Auto playback in ");
    Serial.print(playBackDelay);
    Serial.println(" seconds. Type PX to stop auto play back");
    playBackRepeat = 3;     //Set flag for restart
  }

}

void readAnalogs() {		//Streams the analog stick data over serial port

  Serial.print(analogRead(LXsense), DEC);
  Serial.write(9);
  Serial.print(analogRead(LYsense), DEC);
  Serial.write(9);
  Serial.print(analogRead(RXsense), DEC);  
  Serial.write(9);
  Serial.println(analogRead(RYsense), DEC);
  
}

void fileNameNumber() {     //Converts integer auto number into a filename
  
  uint16_t theValue = autoValue;    //Load the value into a temp slot
  
  int numerals = 5;         //Start on 5th character of filename
  int zPad = 1;             //Flag for zero padding 
  unsigned long divider = 100;     //Divider starts at 100
    
  for (int xx = 0 ; xx < 3 ; xx++) {    //6 digit number    
    if (theValue >= divider) {
      fileName[numerals] = (theValue / divider) + 48;   //Convert to ASCII
      theValue %= divider;
      numerals += 1;
    }   
    else if (zPad or divider == 1) {
      fileName[numerals] = 48;        //ASCII zero
      numerals += 1;
    } 
    divider /= 10;            
  }   
 
}

void sendPots() {			//Sends raw data to pots (no ranging)

  //Analog sticks basic 10 bit ADC range:
  //0 = Left 1023 = Right
  //0 = Up 1023 = Down
  
  //Analog triggers:
  //115 = Released
  //7 - Pulled
  
  //Here we take the actual ADC readings and scale them to fit our digital potentiometers
  //This creates ranges that will "overpower" the actual sticks and sensors on the controller, allowing us to spoof them
  
  Wire.beginTransmission(0x2C);     //Left and right analog stick control potentiometer
  
  Wire.write(B00000000);            //Write to RX 
  Wire.write(map(RX, RXl, RXr, stickRangeRXl, stickRangeRXr));  
  Wire.write(B00010000);            //Write to RY
  Wire.write(map(RY, RYu, RYd, stickRangeRYu, stickRangeRYd)); 
  Wire.write(B01100000);            //Write to LX
  Wire.write(map(LX, LXl, LXr, stickRangeLXl, stickRangeLXr));
  Wire.write(B01110000);            //Write to LY
  Wire.write(map(LY, LYu, LYd, stickRangeLYu, stickRangeLYd));  
  
  Wire.endTransmission();           // stop transmitting

  Wire.beginTransmission(0x2E);     //Analog triggers potentiometers
  Wire.write(B00000000);            //Write to LT 
  Wire.write(map(LT, LTp, LTr, triggerRangeL, triggerRangeH)); //The scaling to override the Hall Effect Sensors is far different than the analog pots!
  Wire.write(B00010000);            //Write to RT
  Wire.write(map(RT, RTp, RTr, triggerRangeL, triggerRangeH));      
  Wire.endTransmission();     // stop transmitting  

}

void setPortC(int directionWhich, int pullupsYes) {

  for (int x = 0 ; x < 12 ; x++) {
    pinMode(mapC[x], directionWhich);
    if (directionWhich == 0) {            //If inputs, then enable/disable pullups
      digitalWrite(mapC[x], pullupsYes);
    }
  }

}

void setLight(int directionWhich, int pullupsYes) {

  for (int x = 0 ; x < 8 ; x++) {
    pinMode(mapLight[x], directionWhich);
    if (directionWhich == 0) {            //If inputs, then enable/disable pullups
      digitalWrite(mapLight[x], pullupsYes);
    }
  }

}

void setMultiplier(int whatMultipler) {
  
    switch(whatMultipler) {
      case 0:		//0 or 1 both set no multiplier
        set1X();
        break;
      case 1:		//0 or 1 both set no multiplier
        set1X();
        break;				
      case 2:
        set2X();
        break;
      case 4:
        set4X();
        break;       
    }

}

void set1X() {
  
  subMultiplier = 0;       //The frequency multiplier to apply to input trigger
  subInterval = 8000;     //The interval between sub divisions. 8000us/interval 
  secondsDivider = 125;   //How many edges make 1 second
  Serial.println("Edge multiplier set to 1X"); 
  
}

void set2X() {
  subMultiplier = 2;       //The frequency multiplier to apply to input trigger. 480Hz default
  subInterval = 4000;     //The interval between sub divisions. 4000us/interval (480Hz default)   
  secondsDivider = 250;   //How many edges make 1 second
  Serial.println("Edge multiplier set to 2X"); 
}

void set4X() {
  subMultiplier = 4;       //The frequency multiplier to apply to input trigger. 480Hz default
  subInterval = 2000;     //The interval between sub divisions. 4000us/interval (480Hz default)  
  secondsDivider = 500;   //How many edges make 1 second 
  Serial.println("Edge multiplier set to 4X"); 
}

void setEdgeMode(int whichMode) {
  
  if (whichMode == 0) {
    edgeSource = 0;
    attachInterrupt(digitalPinToInterrupt(logFreq), edge, RISING); //Use external controller signal for normal purposes
    detachInterrupt(digitalPinToInterrupt(LIGHT7comp));
    Serial.println("Set to Controller Sync Mode. Default rate 125Hz");
    set1X();
  }
  if (whichMode == 1) {
    edgeSource = 1;
    detachInterrupt(digitalPinToInterrupt(logFreq)); //Use external controller signal for normal purposes
    attachInterrupt(digitalPinToInterrupt(LIGHT7comp), edge, RISING);
    Serial.println("Set to Light Sensor #7 Sync Mode white-to-black transistion");
    set1X();
  }
  
}

uint16_t getTimeLeft() {  //Based off pointer position in buffer and sampling speed, returns how many seconds can be recorded

  //120k shorts / 5 shorts 

  return ((240000 - logP) / 10) / secondsDivider;
  
}

void listFiles() {
  
  Serial.println("------------SD CARD DIRECTORY------------");
  myFile = SD.open("/");
  printDirectory(myFile, 0);      

}

void printDirectory(File dir, int numTabs) {
  
  int fileCount = 0;
  
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    fileCount++;
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
  
  if (fileCount == 0) {
    Serial.println("No files");
  }
  
}

void startLogging() {

  userLights = lightRecord;

  Serial.println("Logging BEGIN - type LG again or tap USER button to stop");
  timer = 0;            //Use this to label event #'s
  logP = 0;
  loggingState = 1;
  displayState = 1;     //Show time left
  masterTimer = 0;
  
  userFunc = &stopLogging;
  
}

void stopLogging() {

  timer = 0;

  userLights = lightOff;
  
  //Send ending data
  dataLog[logP++] = 255;            //Set timer flag that says we've reached the end (flag for playback)
  dataLog[logP++] = 255;            //Low byte of flag
  
  if (subMultiplier) {				//Is this more than zero? (2 or 4)
	dataLog[logP++] = subMultiplier;  //Store the rate at which this data was collected  
  }
  else {
	dataLog[logP++] = 1;			//It's really zero but put a one here since it makes more sense to humans 
  }

  dataLog[logP++] = edgeSource;     //What trigger edge was used (not implemented yet)
  dataLog[logP++] = 0;
  dataLog[logP++] = 0;
  dataLog[logP++] = 0x0F;           //Set buttons high so they won't appear as 1's on the EOF row
  dataLog[logP++] = 0xFF;
  dataLog[logP++] = 0;
  dataLog[logP++] = 0;              //Fill rest of line with null data  

  Serial.println("Logging STOP.");
  logSamples = logP / 10;             //Store how many samples we took
  Serial.print(logSamples);
  Serial.println(" samples collected.");
  Serial.println("Type PB to playback, type LG or tap USER button to record another.");

  logP = 0;                           //Reset pointer for re-use when converting to CSV
  loggingState = 0;                   //No longer logging
  displayState = 0;                   //Show time left

  if (writeAfterLogging) {
    saveRAM();
  }

  userFunc = &startLogging;           //Change the USER button function
  
}

void startPlayback() {
  
  if (loggingState == 100) {      //Already logging? Don't restart!
    return;
  }
  
  if (!isDataThere()) {			//See if anything's in RAM and abort if not
	return;
  }

  setControl(1);                   //Enter CONTROL mode type = RAM playback
  
  userLights = lightPlayback;     //Turn on Playback LED indicator
  
  Serial.println("Playback BEGIN - type PB again or tap USER button to stop before end of file");
  timer = 0;            //Use this to label event #'s
  logP = 0;             //Reset the log array pointer
  logSamples = 0;       //Reset the log sample counter
  loggingState = 100;   //Log state set to PLAYBACK
  displayState = 2;     //Show playback time elapsed

  samplingOffset = 5;   //How long after the controller sync before we set our buttons (in us)

  userFunc = &stopPlayback; //Set user button to STOP PLAYBACK
  
}

void stopPlayback() {

  setSample();
 
  userLights = lightOff;
 
  Serial.println("Playback STOP. Pointer reset to beginning of memory"); 
  timer = 0;            //Use this to label event #'s
  logP = 0;             //Reset the log pointer
  loggingState = 0;   //Log state set to PLAYBACK
  displayState = 0;     //Show time left
  samplingOffset = samplingOffsetDefault;
  
  userFunc = &startPlayback;
  
  if (playBackRepeat) {     //Flag set to repeat the loop?  
  
	if (seqFile) {			//Are we doing sequential file loads instead of just repeating what's in RAM?

		autoValue++;		//Increase the filename number
		fileNameNumber();	//Put that number in filename string
		
		if (!SD.exists(fileName)) {  //If that file doesn't exist...
		  Serial.print("End of file sequence reached, starting over with ");
		  autoValue = autoStart;	//Reset this value to what we loaded originally
		  fileNameNumber();			//Change the filename to this new value		  
		  Serial.println(fileName); //Print filename
		}
		
		loadRAM();			//Load filename into RAM

	}
	
	masterTimer = micros() + (playBackDelay * 1000000);   //Set future point where we will repeat
	Serial.print("Auto playback in ");
	Serial.print(playBackDelay);
	Serial.println(" seconds. Type PX to stop auto play back");
	playBackRepeat = 2;     //Set flag for restart	

  }
  else {
    Serial.println("Type PB or tap USER button to replay");  
  }
  
}

int isDataThere() {

  if (dataLog[0] == 0) {		//There should always be some sort of value in LX. If there isn't, no data is present
	  Serial.println("No playback data present. Type LG to log controller data.");
	  return 0;	  
  }
  
  return 1;
	
}

void setControl(int whatMode) {
  
  setPortC(1, 0);                 //Set digital pins to output
  digitalWrite(muxDir, 0);        //Set as POT/BUTTON CONTROL  
  controlSample = whatMode;          //Set CONTROL MODE
  
}

void setSample() {
  
  setPortC(0, 0);                 //Set digital pins back to input
  digitalWrite(muxDir, 1);        //Set as POT/BUTTON SENSE (1)
  controlSample = 0;          //Revert back to sampling mode
  
}

void showRAMtargets() {
  
  Serial.print("RAM export target = ");

  if (sendWhere == sendSD) {
    Serial.println("SD card");
  }
  else {
    Serial.println("UART");
  }

  Serial.print("RAM export format = ");

  if (sendType == CSVfile) {
    Serial.println("CSV file");
  }
  else {
    Serial.println("RAW file");
  }  
  
  
}

void saveRAM() {
 
  if (!isDataThere()) {			//See if anything's in RAM and abort if not
	return;
  }
 
  if (sendType == RAWfile) {         //Export raw data?
    
    if (sendWhere == sendUART) {      //Sending to UART
        saveToRAW();                  //Just sends data no headers or anything
    }
    else {                            //Saving to SD so open the file

      if (autoNumber) {
        fileNameNumber();     //Update the filename with a 3 digit number
        if (autoValue < 999) {
          autoValue++;
        }
        else {
          autoValue = 0;
          Serial.println("Auto numbering has rolled over to 0.");
        }
      }    
    
      fileName[9] = 'R';
      fileName[10] = 'A';
      fileName[11] = 'W';
      Serial.print("Creating RAW file ");
      Serial.print(fileName);
      Serial.print(" on SD card...");
      myFile = SD.open(fileName, FILE_WRITE | O_TRUNC);      
      saveToRAW();
      myFile.close(); 
      Serial.println(" done!"); 
      
    }
    
  }
  else {                            //OK exporting CSV. But to SD card file or UART dump?
    
    if (sendWhere == sendUART) {      //Sending to UART
        saveToCSV();                  //Just sends data no headers or anything
    }
    else {                            //Saving to SD so open the file

      if (autoNumber) {
        fileNameNumber();     //Update the filename with a 3 digit number
        if (autoValue < 999) {
          autoValue++;
        }
        else {
          autoValue = 0;
          Serial.println("Auto numbering has rolled over to 0.");
        }
      }
      
      fileName[9] = 'C';
      fileName[10] = 'S';
      fileName[11] = 'V';           
      Serial.print("Creating CSV file ");
      Serial.print(fileName);
      Serial.print(" on SD card...");
      myFile = SD.open(fileName, FILE_WRITE | O_TRUNC);
      myFile.println("Frame #,Left Analog X,Left Analog Y,L3,Right Analog X,Right Analog Y,R3,Left Trigger,LB,Right Trigger,RB,A,B,X,Y,UP,DOWN,LEFT,RIGHT,Camera,Sensors,Flags");
      saveToCSV();
      myFile.close(); 
      Serial.println(" done!");    
    
    }   
  }
  
}

void loadRAM() {    //Load file into RAM from SD file CSV or RAW

  if (fileName[9] == 'C' and fileName[10] == 'S' and fileName[11] == 'V') { //File is a CSV?
    parseFromCSV();  
  }
  else {        //Not CSV? Load as RAW
    loadRAW();    
  }

}

void saveToCSV() {   //Converts the logging data in RAM to a human-readable CSV and sends to target
  
  logP = 0;                         //Reset RAM pointer
  timer = 0;                        //Use as counter to draw the event number
  logCounter = logSamples;      //Copy the # of samples so we can count down without destroying original value
  
  while(logCounter--) {
     
    buttons = (dataLog[logP + 6] << 8) | dataLog[logP + 7]; //Load and shift offset 6 high low and OR in low byte

	
	//HAVE A VERSION FOR HEX!
  
    if (hexDecimal) {
      sendInt(timer++, 1);             //Event #  
    }
    else {
      sendHex(timer++, 1);             //Event #
    }
  
    sendInt(dataLog[logP], 1);       //LX
    sendInt(dataLog[logP + 1], 1);   //LY
    printBoolean(10, 1);             //Draw L3

    sendInt(dataLog[logP + 2], 1);   //RX or Muliplier in EOF row
    sendInt(dataLog[logP + 3], 1);   //RY or Edge Source in EOF row
    printBoolean(9, 1);                  //Show R3

    sendInt(dataLog[logP + 4], 1);   //LT  
    printBoolean(11, 1);                 //Left bumper
   
    sendInt(dataLog[logP + 5], 1);   //RT  
    printBoolean(0, 1);                  //Right bumper
    
    printBoolean(1, 1);                  //A  
    printBoolean(2, 1);                  //B
    printBoolean(3, 1);                  //X
    printBoolean(4, 1);                  //Y

    printBoolean(5, 1);                  //Up
    printBoolean(6, 1);                  //Down
    printBoolean(7, 1);                  //Left   
    printBoolean(8, 1);                 //Right
    
    sendInt(dataLog[logP + 8], 1);      //Camera flags
    sendInt(dataLog[logP + 9], 0);      //Light sensor with no following comma
    
    //PUT FLAGS HERE!
    
    if (printFlags and logCounter) {    //Don't print flags on final line as they'll be invalid

      buttons = ~buttons;               //Inverse this for easier logic
      g = 1;                            //Comma flag

      if (dataLog[logP + 4] < LTedges[0]) {
        if (g) {                          //If this is first item add comma else don't
          g = 0;
          sendByte(',');
        }        
        sendString("LeftTrigger ");
      }     
      if (dataLog[logP + 5] < RTedges[0]) {
        if (g) {                          //If this is first item add comma else don't
          g = 0;
          sendByte(',');
        }            
        sendString("RightTrigger ");
      }         
      if (buttons & 0x001E) {
        if (g) {                          //If this is first item add comma else don't
          g = 0;
          sendByte(',');
        }            
        sendString("FaceButtons ");
      }       
      if (buttons & 0x01E0) {
        if (g) {                          //If this is first item add comma else don't
          g = 0;
          sendByte(',');
        }            
        sendString("Dpad ");
      }       
      if (buttons & 0x0801) {
        if (g) {                          //If this is first item add comma else don't
          g = 0;
          sendByte(',');
        }            
        sendString("Shoulders ");
      }  
      if (buttons & 0x0600) {
        if (g) {                          //If this is first item add comma else don't
          g = 0;
          sendByte(',');
        }            
        sendString("L3/R3 ");
      }   
      
    }
 
    sendByte(0x0D);       //Carriage return
    sendByte(0x0A);       //New line
    
    logP += 10;                //Advance to next sample    

  }
  
  sendString("TOTAL^,EOF^,EOF^, ,FREQ X^");
  sendByte(0x0D);       //Carriage return
  sendByte(0x0A);       //New line
  
  timer = 0;
    
}

void saveToRAW() {   //Saves the logging data in RAM as raw data
  
  logP = 0;                         //Reset RAM pointer
  logCounter = logSamples;          //Copy the # of samples so we can count down without destroying original value
  
  //
  
  while(logCounter--) {
    
    sendByte(dataLog[logP]);       //LX
    sendByte(dataLog[logP + 1]);   //LY
    sendByte(dataLog[logP + 2]);   //RX
    sendByte(dataLog[logP + 3]);   //RY
    sendByte(dataLog[logP + 4]);   //LT  
    sendByte(dataLog[logP + 5]);   //RT  
    sendByte(dataLog[logP + 6]);   //Button high byte 
    sendByte(dataLog[logP + 7]);   //Button low byte    
    sendByte(dataLog[logP + 8]);   //Camera flags
    sendByte(dataLog[logP + 9]);   //Light sensor readings
    
    logP += 10;                //Advance to next sample

  }

}

void parseFromCSV() { //Turns an ASCII CSV file into data we can store to RAM and playback for controller zombie
  
  Serial.print("Parsing file ");
  Serial.print(fileName);
  Serial.print(" into RAM...");
  myFile = SD.open(fileName, FILE_READ);

  f = 0;                    //Set value so while will execute at least once 

  while(f != 0x0A) {        //First line is just the text header so skip it (read until we find first NEWLINE 0x0A)
    f = myFile.read();   
  }
  
  logP = 0;                 //Reset pointer  
  logSamples = 0;           //Reset number of samples
     
  while(1) {            //Keep going until we hit the end of file indicator (which is our final sample)

    f = 0;                    //Set value so while will execute at least once 
	  
    while (f != ',') {                //Eat the first value (the frame counter) Use this loop (not parse) because hex values confuse parser
      f = myFile.read();
    }
    
    dataLog[logP] = parseOne();			//OK now we're on to real data, parse it into array variables
    dataLog[logP + 1] = parseOne();
    parseBoolean(10, parseOne());		//ParseBoolean uses the CSV values and stuffs bits in the Buttons variable
    dataLog[logP + 2] = parseOne();
    dataLog[logP + 3] = parseOne();
    parseBoolean(9, parseOne());  

    dataLog[logP + 4] = parseOne();
    parseBoolean(11, parseOne());
    dataLog[logP + 5] = parseOne();
    parseBoolean(0, parseOne());

    parseBoolean(1, parseOne());  
    parseBoolean(2, parseOne());  
    parseBoolean(3, parseOne());  
    parseBoolean(4, parseOne());     
    parseBoolean(5, parseOne());
    parseBoolean(6, parseOne());  
    parseBoolean(7, parseOne());  
    parseBoolean(8, parseOne());
    
    dataLog[logP + 8] = parseOne();     //Get camera control
    dataLog[logP + 9] = parseOne();     //and light sensor data (should be 0 on load)

    f = 0;                    //Set value so while will execute at least once 
    
    while(f != 0x0A) {        //Eat bytes until we find a NEWLINE. This will ignore any FLAGS at the end
      f = myFile.read();   
    }     
    
    dataLog[logP + 6] = buttons >> 8;       //Load button variable (that we built with ParseBoolean) into RAM
    dataLog[logP + 7] = buttons & 0xFF;

    logSamples++;
    
    if (dataLog[logP] == 255 and dataLog[logP + 1] == 255) { //End of file?   
      break;
    }
    
    logP += 10;                 //Advance to next frame in array

  }
  
  //Serial.println(logSamples);
  
  myFile.close(); 
  Serial.println("done!");
  setMultiplier(dataLog[logP + 2]);                      //The 3rd value in the last sample is the frequency the sample was taken at
  if (!playBackRepeat) {				//Message is redundant if playback repeat is enabled so don't print it in that case
	Serial.println("Type PB or tap the USER button to begin playback.");	  
  }

  userFunc = &startPlayback;
  
}

short parseOne() {    //Parse one item of data from the CSV file and return it as a short
  
  short theValue = 0;
  g = 0;
  //clearBuffer();
  f = 48;                //Set this so it executes once

  while (f > 47 and f < 58) {            //Keep going until we hit a comma or carriage return (anything non-ASCII number digit)
    f = myFile.read();
	//Serial.print(f);
    if (f > 47 and f < 58) {    //Actual ASCII number?
        fileString[g++] = f - 48;    //Put it in our array as a numeral. Saves time later on
    }
  }
  
  //OK we hit a comma or carriage return. Now to parse the number
  
  int multiplier = 1;     //Reset multipler to one's place
  
  while(--g > -1) {       //Let's count backwards, lobbing the G value first so we get our actual last char position
      theValue += (fileString[g] * multiplier);
      multiplier *= 10;   //Goes up 10x each time because Arabic numerals 
  }

  return theValue;
  
}

short parseOneUART() {    //Parse one item of data from the CSV file and return it as a short
  
  short theValue = 0;
  g = 0;
  f = 48;                //Clear this

  while (f > 47 and f < 58) {            //Keep going until we hit a comma or carriage return (anything non-ASCII number digit)
    f = getByteUART();
    if (f > 47 and f < 58) {    //Actual ASCII number?
        fileString[g++] = f - 48;    //Put it in our array as a numeral. Saves time later on
    }
  }
  
  //OK we hit a comma or carriage return. Now to parse the number
  
  int multiplier = 1;     //Reset multipler to one's place
  
  while(--g > -1) {       //Let's count backwards, lobbing the G value first so we get our actual last char position
      theValue += (fileString[g] * multiplier);
      multiplier *= 10;   //Goes up 10x each time because Arabic numerals 
  }

  return theValue;
  
}

void loadRAW() {
  
  Serial.print("Loading file ");
  Serial.print(fileName);
  Serial.print(" into RAM...");
  myFile = SD.open(fileName, FILE_READ);
 
  logP = myFile.size();            //How many bytes to read
  logSamples = logP / 10;          //How many samples that is
  
  //Serial.println(logP);  
  //Serial.println(logSamples);
  
  myFile.read(dataLog, logP);      //Stream it all in!
  
  myFile.close(); 
  Serial.println("done!");  
  setMultiplier(dataLog[((logSamples - 1) * 10) + 2]); //The 3rd value in the last sample is the frequency the sample was taken at
  if (!playBackRepeat) {				//Message is redundant if playback repeat is enabled so don't print it in that case
	Serial.println("Type PB or tap the USER button to begin playback.");	  
  }
  userFunc = &startPlayback;
  
}

void parseCSVfromUART() {    //Imports a CSV file from UART into RAM one line at a time with acknowledge

  while(Serial.available()) {       //Remove any carriage returns after the /UC command so they won't affect the next step
    Serial.read();
  }

  Serial.println("READY TO RECEIVE CSV");     //Flag to tell PC to send data

  f = 0;                    //Clear flag
  
  while(f != 0x0A) {        //First line is just the text header so skip it (read until we find first NEWLINE)
    f = getByteUART();   
  }
  
  logP = 0;                 //Reset pointer  
  logSamples = 0;           //Reset number of samples
     
  while(1) {            //Keep going until we hit the end of file indicator (which is our final sample)

    //timer = parseOne();               //Get the event #. Don't need it but have to get past it in the file

	f = 0;                    //Clear flag
		
    while (f != ',') {                //Eat the first value (the frame counter) Use this loop (not parse) because hex values confuse parse
      f = getByteUART();
    }
    
    dataLog[logP] = parseOneUART();
    dataLog[logP + 1] = parseOneUART();
    parseBoolean(10, parseOneUART());
    dataLog[logP + 2] = parseOneUART();
    dataLog[logP + 3] = parseOneUART();
    parseBoolean(9, parseOneUART());  

    dataLog[logP + 4] = parseOneUART();
    parseBoolean(11, parseOneUART());
    dataLog[logP + 5] = parseOneUART();
    parseBoolean(0, parseOneUART());

    parseBoolean(1, parseOneUART());  
    parseBoolean(2, parseOneUART());  
    parseBoolean(3, parseOneUART());  
    parseBoolean(4, parseOneUART());     
    parseBoolean(5, parseOneUART());
    parseBoolean(6, parseOneUART());  
    parseBoolean(7, parseOneUART());  
    parseBoolean(8, parseOneUART());
    
    dataLog[logP + 8] = parseOneUART();     //Get camera control
    dataLog[logP + 9] = parseOneUART();     //and light sensor data (should be 0 on load)

    f = 0;                    //Clear this just in case
    
    while(f != 0x0A) {        //Eat bytes until we find a NEWLINE. This will ignore any FLAGS at the end
      f = getByteUART();   
    }     
    
    dataLog[logP + 6] = buttons >> 8;       //Load buttons into RAM
    dataLog[logP + 7] = buttons & 0xFF;

    logSamples++;
    
    Serial.println(logSamples);
    
    if (dataLog[logP] == 255 and dataLog[logP + 1] == 255) { //End of file?   
      break;
    }
    
    logP += 10;                 //Advance to next frame in array

  }

  f = getByteUART();        //Eat into final line...  
  f = getByteUART(); 
  f = getByteUART(); 
  
  while(f != 0x0A) {        //Eat up final line of CSV file (the EOF text indicators)
    f = getByteUART();   
  }

  //Serial.println(logSamples);
  
  logSamples = (logP + 10) / 10;    //Need to add the 10 here since we break'd out before adding it in the loop
  Serial.print("OK ");
  Serial.print(logSamples);
  Serial.println(" samples loaded.");
  setMultiplier(dataLog[logP + 2]); //The 3rd value in the last sample is the frequency the sample was taken at
  
}

char getByteUART() {

  while(Serial.available() == 0) {} //Wait until we get a byte...

  return Serial.read();

}

void loadRAWfromUART() {    //Imports RAW data from UART into RAM

  while(Serial.available() > 0) {     //Flush the buffer (get rid of New Line Carriage Return stuff)
    Serial.read();
  }    

  logP = 0;                         //Reset pointer
  f = 0;                            //Use this to count elements per frame
  
  Serial.println("READY TO RECEIVE RAW");
  
  while(1) {                        //Stay in this loop until we get the last frame
    
    while(f < 10) {                 //Collect 10 bytes
      if (Serial.available()) {
        dataLog[logP + f] = Serial.read();
        f++;
      }
    }
 
    f = 0;
    
    if (dataLog[logP] == 255 and dataLog[logP + 1] == 255) {  //DId we get an End of File indicator? Break out.
        break;
    }
    
    logP += 10;                     //Jump to next frame in memory

  }

  logSamples = (logP + 10) / 10;    //Need to add the 10 here since we break'd out before adding it in the loop
  Serial.print("OK ");
  Serial.print(logSamples);
  Serial.println(" samples loaded.");
  setMultiplier(dataLog[logP + 2]); //The 3rd value in the last sample is the frequency the sample was taken at
 
}

void sendString(const char *str) {

  if (sendWhere) {      //Send to UART?
      Serial.print(str);
  }
  else {
      myFile.print(str);          //No? Must be SD then
  }

}

void sendStringln(const char *str) {

  if (sendWhere) {      //Send to UART?
      Serial.println(str);
  }
  else {
      myFile.println(str);          //No? Must be SD then
  }

}

void sendByte(uint8_t theByte) {

  if (sendWhere) {      //Send to UART?
      Serial.write(theByte);
  }
  else {
      myFile.write(theByte);          //No? Must be SD then
  }

}

void sendInt(unsigned long theValue, int doComma) { //0x12 Puts up to a 6 digit value on screen at X Y block positions (0-15 0-3)

	int zPad = 0;							          //Flag for zero padding	
	unsigned long divider = 10000;			//Divider starts at 10k
		
	for (int xx = 0 ; xx < 5 ; xx++) {		//5 digit number		
		if (theValue >= divider) {
			sendByte('0' + (theValue / divider));
			theValue %= divider;
			zPad = 1;
		}		
		else if (zPad or divider == 1) {
			sendByte('0');
		}	
		divider /= 10;						
	}	
  
  if (doComma) {
    sendByte(aComma);
  }

}

void sendHex(uint16_t theValue, int doComma) {      //Sends a 4 digit HEX value

  sendByte('0');                //Preface with this so Excel will think it's just text, not a number
  sendByte('x');
  
  char nibble = theValue >> 12;  
    
  sendByte((nibble < 10) ? '0' + nibble : 'A' + nibble - 10);
  
  nibble = (theValue >> 8) & 0x0F;
  
  sendByte((nibble < 10) ? '0' + nibble : 'A' + nibble - 10);
  
  nibble = (theValue >> 4) & 0x0F;  
  
  sendByte((nibble < 10) ? '0' + nibble : 'A' + nibble - 10);
  
  nibble = theValue & 0x0F;  

  sendByte((nibble < 10) ? '0' + nibble : 'A' + nibble - 10);  

  if (doComma == 1) {
    sendByte(aComma);
  }
  
  
}

void clearBuffer() {  //Clears the file input buffer

  for (int x = 0 ; x < 32 ; x++) {
    fileString[x] = 0;  
  }

}

void parseBoolean(int bitToWrite, short whatState) {    //Writes a bit into the button register from a CSV read

  if (whatState) {                    //CSV displays 1 for pressed and 0 for off, but electrically it's the opposite
    buttons &= ~(1 << bitToWrite);
  }
  else {
    buttons |= (1 << bitToWrite);
  }
  
}

void printBoolean(int bitToCheck, int doComma) {        //Check a specific bit of the buttons register and draw a 1 or 0 in ASCII

    if (doComma == 2) {
      sendByte(aComma);
    }
    if ((1 << bitToCheck) & buttons) {       //Button NOT pressed = 1, status negative show nothing
        sendByte('0');                  //For the CSV, 1 = pressed (inverse of electrical logic)
    }
    else {
        sendByte('1');      
    }
    if (doComma == 1) {
      sendByte(aComma);
    }

}

void commAll(unsigned char whatState) {
  
    digitalWrite(shiftLoad, LOW);     //Do this early so we can time the conversion(s) using a scope

    for (int x = 0 ; x < 10 ; x++) {
      SPI.transfer(whatState);  
    }

    digitalWrite(shiftLoad, 1);     //Done sending

}

void manualPot() {		//Get a manual value from UART and puts it on digital pot (sets manual control mode)

	for (int x = 0 ; x < 5 ; x++) {

		fileString[x] = Serial.read();

	}
	
	flush();
	
	uint8_t value = (fileString[2] - 48) * 100;
	value += (fileString[3] - 48) * 10;
	value += (fileString[4] - 48);
	
	switch(fileString[0]) {
		
		case 'R':
		
			switch(fileString[1]) {
			
				case 'X':
					RX = value;
				break;
				case 'Y':
					RY = value;
				break;			
				case 'T':
					RT = value;
				break;		
				
			}

		break;

		case 'L':
		
			switch(fileString[1]) {
			
				case 'X':
					LX = value;
				break;
				case 'Y':
					LY = value;
				break;			
				case 'T':
					LT = value;
				break;		
				
			}
			
		break;
		
		default:
			Serial.println("Exiting manual pot control");
			setSample();			 
			userLights = lightOff;
			timer = 0;            //Use this to label event #'s
			logP = 0;             //Reset the log pointer
			loggingState = 0;   //Log state set to PLAYBACK
			displayState = 0;     //Show time left
			samplingOffset = samplingOffsetDefault;			  
			userFunc = &startPlayback;
			return;
	
		break;
	
	}
	Serial.println("Manual pot control");
	setControl(1);
	logP = 0xFFFFFFFF;

}

void rangingPot() {		//Get a value from UART and stores it in the analog pot ranging variables / EEPROM


	for (int x = 0 ; x < 6 ; x++) {			//Get the next 6 character such as RXr230 - change RX left range edge to 230

		fileString[x] = Serial.read();

	}
	
	flush();
	
	uint8_t value = (fileString[3] - 48) * 100;
	value += (fileString[4] - 48) * 10;
	value += (fileString[5] - 48);

	switch(fileString[0]) {
		
		case 'R':
		
			if (fileString[1] == 'X') {
				if (fileString[2] == 'L') {
					stickRangeRXl = value;
					Serial.print("RXL=");
					Serial.println(value);
				}
				else {
					stickRangeRXr = value;
					Serial.print("RXL=");
					Serial.println(value);					
				}
			}
			else {		//Must be Y
				if (fileString[2] == 'U') {		//Is it the UP range?
					stickRangeRYu = value;
					Serial.print("RYU=");
					Serial.println(value);					
				}
				else {	//Must be down
					stickRangeRYd = value;
					Serial.print("RYD=");
					Serial.println(value);					
				}				
			}

		break;

		case 'L':
		
			if (fileString[1] == 'X') {
				if (fileString[2] == 'L') {
					stickRangeLXl = value;
					Serial.print("LXL=");
					Serial.println(value);					
				}
				else {
					stickRangeLXr = value;
					Serial.print("LXR=");
					Serial.println(value);					
				}
			}
			else {		//Must be Y
				if (fileString[2] == 'U') {		//Is it the UP range?
					stickRangeLYu = value;
					Serial.print("LYU=");
					Serial.println(value);					
				}
				else {	//Must be down
					stickRangeLYd = value;	
					Serial.print("LYD=");
					Serial.println(value);					
				}				
			}
			
		break;
		
		case 'S':		//Save to EEPROM?
			Serial.print("Storing values to EEPROM... ");             
			storeConfig();      //Save new values to EEPROM        
			Serial.println("done!");				
		break;
		
		default:
			Serial.println("Current digital potentiometer stick ranges:");
			Serial.print("LXL=");
			Serial.print(stickRangeLXl);
			Serial.print("\tLXR=");
			Serial.println(stickRangeLXr);
			Serial.print("LYU=");
			Serial.print(stickRangeLYu);
			Serial.print("\tLYD=");
			Serial.println(stickRangeLYd);
			Serial.print("RXL=");
			Serial.print(stickRangeRXl);
			Serial.print("\tRXR=");
			Serial.println(stickRangeRXr);
			Serial.print("RYU=");
			Serial.print(stickRangeRYu);
			Serial.print("\tRYD=");
			Serial.println(stickRangeRYd);			
		break;
	
	}

}

void rangeAnalogs() {    //Get the upper and lower ranges of the Analog triggers and joysticks
  
    switch(menuState) {

      case 0:
        Serial.println("Analog Input Configuration");
        Serial.println("---------------------------------------------------");
        Serial.println(" ");
        Serial.println("Leave all analog controls in default positions and tap A...");
        aState = 1;
        menuState = 5;
      break;
      case 5:
        if (aState == 0xFF) {         
          LTr = LT;
          RTr = RT;
          LXc = LX;
          LYc = LY;
          RXc = RX;
          RYc = RY;        
          Serial.println("Default analog values CAPTURED!");
          Serial.println("");
          Serial.println("Pull LEFT TRIGGER fully and tap A...");
          menuState = 10;
          aState = 1;
        }      
      break; 
      case 10:
        if (aState == 0xFF) {
          //STORE LEFT TRIGGER FULL PULL HERE
          LTp = LT;     //Set left trigger pulled value
          Serial.println("Left trigger analog range CAPTURED!");
          Serial.println("");
          Serial.println("You can now release the left trigger.");
          Serial.println("Next, pull RIGHT TRIGGER fully and tap A...");
          menuState = 15;
          aState = 1;
        }
      break;      
      case 15:
        if (aState == 0xFF) {
          //STORE RIGHT TRIGGER FULL PULL HERE
          RTp = RT;     //Set left trigger pulled value
          Serial.println("Right trigger analog range CAPTURED!");
          Serial.println("");
          Serial.println("You can now release the right trigger.");
          Serial.println("Next, push LEFT ANALOG STICK UP and tap A...");
          //WHAT'S NEXT???
          menuState = 20;
          aState = 1;
        }
      break;
      case 20:
        if (aState == 0xFF) {
          //STORE LEFT ANALOG LEFT HERE
          LYu = LY;
          Serial.println("Next, push LEFT ANALOG STICK DOWN and tap A...");
          //WHAT'S NEXT???
          menuState = 25;
          aState = 1;
        }
      break;      
      case 25:
        if (aState == 0xFF) {
          //STORE LEFT ANALOG LEFT HERE
          LYd = LY;
          Serial.println("Next, push LEFT ANALOG STICK LEFT and tap A...");
          //WHAT'S NEXT???
          menuState = 30;
          aState = 1;
        }
      break;        
      case 30:
        if (aState == 0xFF) {
          //STORE LEFT ANALOG LEFT HERE
          LXl = LX;
          Serial.println("Next, push LEFT ANALOG STICK RIGHT and tap A...");
          //WHAT'S NEXT???
          menuState = 35;
          aState = 1;
        }
      break;       
      case 35:
        if (aState == 0xFF) {
          //STORE LEFT ANALOG LEFT HERE
          LXr = LX;
          Serial.println("Left analog stick range CAPTURED!");
          Serial.println("");        
          Serial.println("Next, push RIGHT ANALOG STICK UP and tap A...");
          //WHAT'S NEXT???
          menuState = 40;
          aState = 1;
        }
      break;        
      case 40:
        if (aState == 0xFF) {
          RYu = RY;
          Serial.println("Next, push RIGHT ANALOG STICK DOWN and tap A...");
          //WHAT'S NEXT???
          menuState = 45;
          aState = 1;
        }
      break;      
      case 45:
        if (aState == 0xFF) {
          RYd = RY;
          Serial.println("Next, push RIGHT ANALOG STICK LEFT and tap A...");
          //WHAT'S NEXT???
          menuState = 50;
          aState = 1;
        }
      break;        
      case 50:
        if (aState == 0xFF) {
          RXl = RX;
          Serial.println("Next, push LEFT ANALOG STICK RIGHT and tap A...");
          //WHAT'S NEXT???
          menuState = 55;
          aState = 1;
        }
      break;
      case 55:
        if (aState == 0xFF) {
          RXr = RX;
          Serial.println("Right analog stick range CAPTURED!");
          Serial.println("");
          menuState = 255;
        }
      break;
      
      case 255:               //Wrap things up!
      
        //Do all the math here and store in an array so we don't have to with each conversion
        
        //Calculate left trigger edges
        delta = LTr - LTp;    //Find distance between trigger extremities
        delta >>= 2;         //Divide by 4
        LTedges[2] = LTr - delta;
        LTedges[1] = LTedges[2] - delta;
        LTedges[0] = LTedges[1] - delta;
        
        //Serial.println(LTedges[0]);
        //Serial.println(LTedges[1]);        
        //Serial.println(LTedges[2]);
        
        //Calculate right trigger edges
        delta = RTr - RTp;    //Find distance between trigger extremities
        delta >>= 2;         //Divide by 4
        RTedges[2] = RTr - delta;
        RTedges[1] = RTedges[2] - delta;
        RTedges[0] = RTedges[1] - delta;

        //Serial.println(RTedges[0]);
        //Serial.println(RTedges[1]);        
        //Serial.println(RTedges[2]);        
        
        //Left analog stick  
        //Calculate LX left edges
        delta = LXc - LXl;
        delta >>= 2;        
        LXedges[2] = LXc - delta;
        LXedges[1] = LXedges[2] - delta;
        LXedges[0] = LXedges[1] - delta;

        //Calculate LX right edges
        delta = LXr - LYc;
        delta >>= 2;        
        LXedges[3] = LXc + delta;
        LXedges[4] = LXedges[3] + delta;
        LXedges[5] = LXedges[4] + delta;          
        
        //Calculate LY up edges
        delta = LYc - LYu;
        delta >>= 2;        
        LYedges[2] = LYc - delta;
        LYedges[1] = LYedges[2] - delta;
        LYedges[0] = LYedges[1] - delta;

        //Calculate LY down edges
        delta = LYd - LYc;
        delta >>= 2;        
        LYedges[3] = LYc + delta;
        LYedges[4] = LYedges[3] + delta;
        LYedges[5] = LYedges[4] + delta;
      
        //Right analog stick
        //Calculate RX left edges
        delta = RXc - RXl;
        delta >>= 2;        
        RXedges[2] = RXc - delta;
        RXedges[1] = RXedges[2] - delta;
        RXedges[0] = RXedges[1] - delta;

        //Calculate RX right edges
        delta = RXr - RYc;
        delta >>= 2;        
        RXedges[3] = RXc + delta;
        RXedges[4] = RXedges[3] + delta;
        RXedges[5] = RXedges[4] + delta;
        
        //Calculate RY up edges
        delta = RYc - RYu;
        delta >>= 2;        
        RYedges[2] = RYc - delta;
        RYedges[1] = RYedges[2] - delta;
        RYedges[0] = RYedges[1] - delta;

        //Calculate RY down edges
        delta = RYd - RYc;
        delta >>= 2;        
        RYedges[3] = RYc + delta;
        RYedges[4] = RYedges[3] + delta;
        RYedges[5] = RYedges[4] + delta;

        Serial.print("Storing values to EEPROM... ");             
        storeConfig();      //Save new values to EEPROM        
        Serial.println("done!");
		showRanges();
        Serial.println("Analog Ranging Process complete!");
        menuWhich = 0;
        menuState = 0;
        aState = 0; 
		userLights = lightOff;
      break;

    }
 
}

void storeConfig() {        //Writes settings and other data to EEPROM

  //Left trigger
  EEPROM.write(0, LTr);
  EEPROM.write(1, LTp);
  EEPROM.write(2, LTedges[0]);
  EEPROM.write(3, LTedges[1]);  
  EEPROM.write(4, LTedges[2]);  

  //Right trigger  
  EEPROM.write(10, RTr);
  EEPROM.write(11, RTp);  
  EEPROM.write(12, RTedges[0]);
  EEPROM.write(13, RTedges[1]);
  EEPROM.write(14, RTedges[2]);  

  //Left analog X
  EEPROM.write(20, LXl);
  EEPROM.write(21, LXc);
  EEPROM.write(22, LXr);
  EEPROM.write(23, LXedges[0]);
  EEPROM.write(24, LXedges[1]);
  EEPROM.write(25, LXedges[2]);
  EEPROM.write(26, LXedges[3]);
  EEPROM.write(27, LXedges[4]);
  EEPROM.write(28, LXedges[5]);

  //Left analog Y
  EEPROM.write(30, LYu);
  EEPROM.write(31, LYc);
  EEPROM.write(32, LYd);
  EEPROM.write(33, LYedges[0]);
  EEPROM.write(34, LYedges[1]);
  EEPROM.write(35, LYedges[2]);
  EEPROM.write(36, LYedges[3]);
  EEPROM.write(37, LYedges[4]);
  EEPROM.write(38, LYedges[5]);

  //Right analog X
  EEPROM.write(40, RXl);
  EEPROM.write(41, RXc);
  EEPROM.write(42, RXr);
  EEPROM.write(43, RXedges[0]);
  EEPROM.write(44, RXedges[1]);
  EEPROM.write(45, RXedges[2]);
  EEPROM.write(46, RXedges[3]);
  EEPROM.write(47, RXedges[4]);
  EEPROM.write(48, RXedges[5]);

  //Right analog Y
  EEPROM.write(50, RYu);
  EEPROM.write(51, RYc);
  EEPROM.write(52, RYd);
  EEPROM.write(53, RYedges[0]);
  EEPROM.write(54, RYedges[1]);
  EEPROM.write(55, RYedges[2]);
  EEPROM.write(56, RYedges[3]);
  EEPROM.write(57, RYedges[4]);
  EEPROM.write(58, RYedges[5]);  


	//Ranges for mapping analog stick inputs to digital potentiometer control
  EEPROM.write(60, stickRangeLXr);
  EEPROM.write(61, stickRangeLXl);
  EEPROM.write(62, stickRangeLYd);
  EEPROM.write(63, stickRangeLYu);

  EEPROM.write(64, stickRangeRXr);
  EEPROM.write(65, stickRangeRXl);
  EEPROM.write(66, stickRangeRYd);
  EEPROM.write(67, stickRangeRYu);

  
}

void getConfig() {        //Reads settings and other data to EEPROM

	//The analog values are stored in EEPROM so you only have to pair a controller to monitor once.
	//Each value range has a 10 offset so there's room for future expanion
  
  LTr = EEPROM.read(0);
  LTp = EEPROM.read(1);
  LTedges[0] = EEPROM.read(2);
  LTedges[1] = EEPROM.read(3); 
  LTedges[2] = EEPROM.read(4);  
  
  //5-9 future use?
  
  RTr = EEPROM.read(10);
  RTp = EEPROM.read(11);  
  RTedges[0] = EEPROM.read(12);
  RTedges[1] = EEPROM.read(13);
  RTedges[2] = EEPROM.read(14);

  LXl = EEPROM.read(20);
  LXc = EEPROM.read(21);
  LXr = EEPROM.read(22);
  
  for (g = 0 ; g < 6 ; g++) {
    LXedges[g] = EEPROM.read(23 + g);
  }
  
  LYu = EEPROM.read(30);
  LYc = EEPROM.read(31);
  LYd = EEPROM.read(32);
  
  for (g = 0 ; g < 6 ; g++) {
    LYedges[g] = EEPROM.read(33 + g);
  }

  RXl = EEPROM.read(40);
  RXc = EEPROM.read(41);
  RXr = EEPROM.read(42);
  
  for (g = 0 ; g < 6 ; g++) {
    RXedges[g] = EEPROM.read(43 + g);
  }
  
  RYu = EEPROM.read(50);
  RYc = EEPROM.read(51);
  RYd = EEPROM.read(52);
  
  for (g = 0 ; g < 6 ; g++) {
    RYedges[g] = EEPROM.read(53 + g);
  }  


  if (EEPROM.read(61) == 255) {	//The lower values would never be 255, so a 255 there means blank EEPROM
	
	//Store defaults
	  EEPROM.write(60, upRange);
	  EEPROM.write(61, downRange);
	  EEPROM.write(62, upRange);
	  EEPROM.write(63, downRange);

	  EEPROM.write(64, upRange);
	  EEPROM.write(65, downRange);
	  EEPROM.write(66, upRange);
	  EEPROM.write(67, downRange);
	  
  }  

	stickRangeLXr = EEPROM.read(60);
	stickRangeLXl = EEPROM.read(61);
	stickRangeLYd = EEPROM.read(62);
	stickRangeLYu = EEPROM.read(63);

	stickRangeRXr = EEPROM.read(64);
	stickRangeRXl = EEPROM.read(65);
	stickRangeRYd = EEPROM.read(66);
	stickRangeRYu = EEPROM.read(67);

  if (EEPROM.read(99) == 255) {	//The lower values would never be 255, so a 255 there means blank EEPROM	
	//Store defaults
	  EEPROM.write(99, 3);
  
  } 
  
  fanSpeed = (EEPROM.read(99) * 1000) - 100;	//Get fan defaults
	
  Serial.println("EEPROM defaults loaded");  
  
}

void showRanges() {
  
    Serial.println();
    Serial.println("Current analog ranges:");
    Serial.print("Left trigger released-pulled:\t\t"); 
    Serial.print(LTr);
    Serial.write(9);    
    Serial.println(LTp);    
    Serial.print("Right trigger released-pulled:\t\t");  
    Serial.print(RTr); 
    Serial.write(9);   
    Serial.println(RTp);    
    Serial.print("Left analog X left-center-right:\t");
    Serial.print(LXl);
    Serial.write(9);	
    Serial.print(LXc);
    Serial.write(9);	
    Serial.println(LXr);
    Serial.print("Left analog Y up-center-down: \t\t"); 
    Serial.print(LYu);
    Serial.write(9);	
    Serial.print(LYc);
    Serial.write(9);	
    Serial.println(LYd);
    Serial.print("Right analog X left-center-right:\t");
    Serial.print(RXl);
    Serial.write(9);
    Serial.print(RXc);
    Serial.write(9);
    Serial.println(RXr);
    Serial.print("Right analog Y up-center-down:\t\t");
    Serial.print(RYu);
    Serial.write(9);	
    Serial.print(RYc);
    Serial.write(9);
    Serial.println(RYd);  
  
}

