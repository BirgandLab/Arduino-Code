/***********************************************************************************/ 
/***********************************************************************************/ 
/***********************Pin Assignments*********************************************/ 
/***********************************************************************************/ 
/***********************************************************************************/ 
    const int pump = 9;                //pump forward from source to probe
    const int purge = 10;              //pump reverse from probe to purge (not source)
    const int solenoid = 8;            // Pneumatic valve solenoid
/***********************************************************************************/ 
/********************* SAMPLE PORT VALVE PIN ASSIGNMENTS ***************************/
/***********************************************************************************/ 
    const int inputPort12 = 22;//DECLARE PINS TO READ THE PRESENT LOCATION OF THE  
    const int inputPort11 = 24;//12 PORT VALVE WHEN THE VALVE IS SELECTED, IT WILL 
    const int inputPort10 = 26;
    const int inputPort9 = 28;//IS READ ALL OF THESE VALVES ONE BY ONE TOSEE WHICH  
    const int inputPort8 = 30;//ONE IS PRESENTLY ACTIVE.ADVANCE, IF NECESSARY, 
    const int inputPort7 = 32;//AND READ AGAIN
    const int inputPort6 = 34;
    const int inputPort5 = 36;
    const int inputPort4 = 38;
    const int inputPort3 = 40;
    const int inputPort2 = 42;
    const int inputPort1 = 44;
    const int checkSignal = 46;        // Used to check the position of the valve
    int port[12] = {44,42,40,38,36,34,32,30,28,26,24,22};
/***********************************************************************************/
/***********************INTERNAL ROUTING VALVE PIN ASSIGNMENTS**********************/  
/***********************************************************************************/ 
 
    const int collectOrPurge3Way = 19;//switches flow from SC to purge (with 
									  //reversed flow)
    const int inputOutput3Way = 18;  //switches flow from input port to SC switch--
								     //for  reversing flow
    const int probeOrPurge3Way = 17; // switches flow from Probe to Purge

/***********************************************************************************/
/***********************FRACTION COLLECTOR ADVANCE PIN ASSIGNMENT ******************/
/***********************************************************************************/  

    const int sampleCollectorAdvance = 16;
/***********************************************************************************/
/***************************Analog Input Pin Assignments    ************************/
/***********************************************************************************/
 
    const int probeSignal = A1;                 //This is a 12V signal with a voltage
												//divider on the input do not move it 
                                                //without also addressing the voltage 
    const int reversePurgeSwitch = A8;          //used to be pin A6: Pump runs 
												//reverse flow from probe sample to
    const int forwardPurgeSwitch = A9;          // purge used to be pin A4: Pump  
												//forward from sample port to purge  
    const int purgeToSampleCollectorSwitch = A10;//below probe used to be pin A7: Pump  
												//runs reverse Flow from probe sample 
												//to Fraction Collector
    const int pumpToInstrumentSwitch = A11;     //used to be pin A5: Pump runs forward 
												//from sample port to Probe
    const int valveAdvanceSwitch = A12;         //used to be pin A3: Advance Valve
    const int sampleCollectorAdvanceSwitch = A13;//Advance Fraction Collector 1 step
    const int startSampleCycleSwitch = A14;     //Begin a sample cycle using the 
												//currently active sample port
    const int probeSampleSwitch = A15;          //Begin an automated sample cycle  
												
    int sampleSignal=0;                         //the signal to collect a sample. I 
												//should not put it here, but was too 
												//lazy to scope it properly.


/***********************************************************************************/ 
/**************************INCLUDE STATEMENTS  *************************************/
/***********************************************************************************/

//#include <LiquidCrystal.h> //the other LCD library that doesn't fuction on I2C bus
// include the library code:
#include <Wire.h>                           //I2C bus header
#include <Adafruit_MCP23017.h>              //Headers for RGB LCD shield
#include <Adafruit_RGBLCDShield.h>          //Headers for RGB LCD shield
//DECLARE AN INSTANCE OF THE LCD FOR THE PROGRAM TO USE.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield(); 

//LiquidCrystal lcd(25,23,33,31,29,27); //initialize the library with the numbers of 
										//interface pins necessary when using the
									    //LCD that doesn't use I2C BUS

/***********************************************************************************/ 
/***********************************************************************************/ 
/***********************************************************************************/ 
/****** THESE VARIABLES CAN/MUST BE DEFINED FOR PROPER LVMX FUNCTION            ****/                     
/******  Select the sample port that is #1 in sampling order                    ****/
/******/  const int initialInputPort = 12; /* the valve to start on   */         /***/
/*****   SELECT WHICH INPUT PORTS WILL BE USED FOR THE CURRENT PROGRAM		    ****/
/*****/  int samplePorts[12] = {1,1,1,1,1,1,1,1,1,1,1,1}; //1=USED 0=UN-USED    /***/
/*****/																		    /***/
/*****   SPECIFY DURATION OF PUMPING FOR EACH UNPUT LINE TO SATISFACTORILY      ****/
/*****   FILL THE SAMPLE LINES AND PROBE										/***/
/*****   Enter Duration-Valve#  1--2--3--4--5--6--7--8--9--10-11-12             ****/
/*****/	 //NO PROBLEMS ARE CAUSED BY HAVING VALUES IN PORTS THAT ARE NOT USED 	/***/
/*****/  long int pumpTimeArray[12]   = {30,30,30,30,30,30,30,30,30,30,30,30};  		/***/
/*****/	//pumpTime defines how long the pump needs to run to bring fresh sample /***/
/*****/	//from the source through the system and out the other side of the LVMx /***/
/*****/	//box																	/***/
/*****/																			/***/
/*****/  long int sampleTimeArray[12] = {40,40,40,40,40,40,40,40,40,40,40,40};			/***/
/*****/	//sampleTime specifies how long (in addition to pumpTime) the pump 		/***/
/*****/	//must run to fill the scan probe cuvette or anything else out there	/***/
/*****/  long int totalPurgeTimeArray[12] = {35,35,35,35,35,35,35,35,35,35,35,35};																	/***/
/*****/// pumping to (filling ) the probe/sample coil--45s gets ~8ml currently  /***/
/*****/  long int sampleReturnArray[12] = {15,15,15,15,15,15,15,15,15,15,15,15};
/*****/  //time specifying backpumping of water from fraction collector to keep the volume so
/*****/  //it can be returned to the source.  Trying to recycle as much of the volume draw as possible
/*****/
/*****/	long int coilPurgeArray[12] = {45,45,45,45,45,45,45,45,45,45,45,45};																		/***/
/*****/	 //coilPurge specifies how long after sending the sample to the FC, and a suitable 																		/***/
/*****/	 //volume has been drawn, and water drawn back out of the FC lines, the line is purged																		/***/
/*****/ // back to the source.	
/*****/ long int FCPurgeArray[12] = {25,25,25,25,25,25,25,25,25,25,25,25,};																/***/
/*****/  long int internalLinesTime = 25; 										/***/
/*****/	//an amount of time to tack on that adds time to clear the lines inside /***/
/*****/ //the sampler this timeis used when purging the probe sample to the 	/***/
/*****/	//fraction collector or waste											/***/
/*****/																			/***/
/*****/  int sampleInterval = 2; //collect every nth sample (first sample,   	/***/
/*****/						 //and theneveryother n)							/***/
/*****/																			/***/
/*****/  int signalTime = 2;/* Signal duration required to start sampling cycle */ 
/*****/  long int measurementDelay = 60;/* Delay from cycle START to measurement*/
/*****/ //This specifies how long between the probe cleaning signal measurement
/*****/ //make sure that it is greater than all the pump times, or the world will end.
/***********************************************************************************/ 
/***********************************************************************************/ 
/***********************************************************************************/ 



/***********************************************************************************/ 
/***********************************************************************************/ 
/***********************************************************************************/ 
/*********  VALVE CONTROL ALLOWS CORRECT ROUTING OF SAMPLE THROUGH LVMX    *********/
            int purgeA[3]={LOW,LOW,LOW};                //SHOULD DECLARE AS CONSTANTS
            int pumpToInstrument[3]={LOW,LOW,HIGH};	    //BUT REALLY DOESN'T MATTER
            int purgeB[3]={LOW,HIGH,HIGH};
            int pumpToSC[3]={HIGH,HIGH,HIGH};
             
			//THESE LOGIC LEVELS ARE USED TO TURN ON THE CORRECT VALVES IN THE SAMPLE
			//CYCLES BELOW
/***********************************************************************************/ 
/***********************************************************************************/ 
/***********************************************************************************/ 

void setup()
{
  
  lcd.begin(16, 2);      //INITIALIZE LCD PANEL
  
   
/***********************************************************************************/  
/********************* set up the IO pins for device control **********************/  
/***********************************************************************************/ 

///INPUT PINS ARE FOR READING EXTERNAL SIGNALS WITH THE ARDUINO
///THEY READ 5V LOGIC (2.5 MAY BE ENOUGH TO GIVE A HIGH SIGNAL)

///THESE ARE FOR SENSING THE POSITION OF THE PNEUMATIC 12V SAMPLE VALVE
  pinMode(inputPort1, INPUT);       pinMode(inputPort2, INPUT);
  pinMode(inputPort3, INPUT);       pinMode(inputPort4, INPUT);
  pinMode(inputPort5, INPUT);       pinMode(inputPort6, INPUT);
  pinMode(inputPort7, INPUT);       pinMode(inputPort8, INPUT);
  pinMode(inputPort9, INPUT);       pinMode(inputPort10, INPUT);
  pinMode(inputPort11, INPUT);      pinMode(inputPort12, INPUT);
  
  //BUTTONS ON THE REMOTE AND IN THE BOX ARE USED TO SEND SIGNALS TO
  //THE ARDUINO FOR INITIATING FUNCTION CALLS
 ///THUS SENSING OF SWITCHE BUTTONS AND KNOBS IS AN INPUT FUNCTION
  pinMode(startSampleCycleSwitch, INPUT);  //MANUALL RUN A SAMPLE CYCLE
  pinMode(probeSignal, INPUT);				//CLEANING SIGNAL FROM SCAN
											//INITIATES PUMPING DURING
											//PROBE DRIVEN SAMPLE CYLE
																						
  pinMode(pumpToInstrumentSwitch, INPUT);   //PUMP FROM SOURCE TO PROBE
  pinMode(forwardPurgeSwitch, INPUT);		//PUMP FROM SOURCE TO PURGE
  pinMode(reversePurgeSwitch, INPUT);		//PURGE FROM PROBE
  pinMode(purgeToSampleCollectorSwitch,INPUT); //PURGE FROM PROBE TO FRACTION
												//COLLECTOR
  
  pinMode(valveAdvanceSwitch, INPUT);		//MANUALLY ADVANCE CURRENT INPUT
											//PORT ON 12 PORT SAMPLE VALVE
  pinMode(sampleCollectorAdvanceSwitch, INPUT); //ADVANCE FRACTION COLLECTOR
  pinMode(probeSampleSwitch,INPUT);			//INITIATE PROBE DRIVEN SAMPLE
											//ROUTINE
    
  ///OUTPUT PINS CONTROL EXTERNAL DEVICES 
  ///HERE WE TELL THE ARDUINO THAT THE PINS ASSIGNED TO THESE VARIABLES
  ///WILL BE USED TO OUTPUT 5V LOGIC SIGNALS
  pinMode(solenoid, OUTPUT);    //FOR POSITIONING PNEUMATIC VALVE
  pinMode(checkSignal, OUTPUT); //FOR SENSING PNEUMATIC VALVE POSITION
  

  pinMode(collectOrPurge3Way, OUTPUT);//POSITION 3-WAY SAMPLE ROUTING VALVE
  pinMode(inputOutput3Way, OUTPUT);//POSITION 3-WAY SAMPLE ROUTING VALVE
  pinMode(probeOrPurge3Way,OUTPUT);//POSITION 3-WAY SAMPLE ROUTING VALVE
  pinMode(sampleCollectorAdvance,OUTPUT);  //ADVANCE FRACTION COLLECTOR
  pinMode(pump, OUTPUT);        		//TURN PUMP ON "FORWARD"
  pinMode(purge, OUTPUT);				//TURN PUMP ON "REVERSE"
/***********************************************************************************/ 
/***********************************************************************************/  
/***********************************************************************************/ 
  
  InitializeTwelvePortinputPort(initialInputPort);
 
   }//end setup

void loop (){
  lcd.setCursor(1,0);   lcd.print("READY!");
  lcd.setCursor(1,1);  lcd.print("at valve : ");   lcd.print(CheckValve()); 
  
///IF VALVE ADVANCE IS SIGNALED, CALL THE VALVE ADVANCE ROUTINE 
 if (digitalRead(valveAdvanceSwitch)){ 
      ValveAdvance();
      }
  
///IF SAMPLE COLLECTOR SWITCH IS ACTIVATED, CALL THE SAMPLE COLLECTOR ADVANCE ROUTINE
  if (digitalRead(sampleCollectorAdvanceSwitch)){
       lcd.clear();
          lcd.setCursor(0,1); lcd.print("Purging");
          genericPumpFunction(reversePurgeSwitch, 1, purge,purgeB,
								"Purge B: ","Purging ") ;
      }

///IF FORWARD PURGE IS ACTIVATED TURN THE PUMP ON AND OPEN THE APPROPRIATE VALVES
  if (digitalRead(forwardPurgeSwitch)){
		  //pump water from Probe to Purge Valve B 
          lcd.clear(); lcd.setCursor(0,1); lcd.print("Purging");
          genericPumpFunction(forwardPurgeSwitch,1, pump,purgeA,
				"Purge A: ", "Purging ");
      }

///IF PUMP TO INSTRUMENT IS ACTIVATED, TURN THE PUMP ON AND OPEN APPROPRIATE VALVES	  
     if (digitalRead(pumpToInstrumentSwitch)){
         //pump water from Probe to Purge Valve B 
          lcd.clear(); lcd.setCursor(0,1); lcd.print("PumpToProbe");
          genericPumpFunction(pumpToInstrumentSwitch,1, purge,purgeB,
							   "toProbe: ", "Pumping ");
     }    

///IF REVERSE PURGE SWITCH IS ACTIVATED, TURN PUMP ON AND OPEN APPROPRIATE VALVES
   if (digitalRead(reversePurgeSwitch)){
        //pump water from Probe to Purge Valve B 
          lcd.clear();
          lcd.setCursor(0,1); lcd.print("Purging");
          genericPumpFunction(reversePurgeSwitch, 1, pump,purgeB,
								"Purge B: ","Purging ") ;
   }
///IF PURGE TO SAMPLE COLLECTOR SWITCH IS ACTIVATED, TURN PUMP ON AND ACTIVATE VALVES   
   if (digitalRead(purgeToSampleCollectorSwitch)){
       //pump water from Probe to Purge Valve B 
          lcd.clear();
          lcd.setCursor(0,1); lcd.print("Purging");
          genericPumpFunction(purgeToSampleCollectorSwitch, 1, purge,pumpToSC,"Purge SC: ", "Purging ") ;
   }
 
 ///IF START SAMPLE CYCLE SWITCH IS ACTIVATED, RUN SAMPLE CYCLE ROUTINE  
  if (digitalRead(startSampleCycleSwitch)){
  //start a sample cycle on the present INPUT PORT, after the cycle starts, 
  // collect thesample by flipping the SC switch to the SC position.
          lcd.clear();
          lcd.setCursor(0,1); lcd.print("Sample Cycle");
          int collect=0;
          StartSampleCycle(collect);
  }
  
 ///IF THE PROBE SAMPLE SWITCH IS ACTIVATED ENTER PROBE DRIVE SAMPLE CYCLE
  if(digitalRead(probeSampleSwitch)){
  //automated Sample Cycle utilizing the specified Valves, sending every Nth sample
  // to the collector: relies on probe timing to send signal;
          lcd.clear();
          lcd.setCursor(0,1); lcd.print("Sample Cycle");
		  //send ACTIVE valve numbers and send nth sample to SC
          ProbeDrivenSampleCycle(samplePorts,sampleInterval);
  }
  
  delay(333);
  

}
 
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
///INITIALIZE THE POSITION OF THE VALVE TO THE CONSTANT DEFINED ABOVE
///THIS FUNCTION CHECKS THE POSITION OF THE VALVE, THEN ADVANCES IT INCREMENTALLY
///UNTIL IT IS AT THE INITIAL VALVE POSITION
 
 void InitializeTwelvePortinputPort( int initialInputPort) {
        int currentSamplePort=CheckValve(); //RETURN THE CURRENT POSITION 
        lcd.clear();   						//CLEAR LCD SCREEN
		lcd.setCursor(0,0); 				//SET POSITION OF NEW TEXT
		lcd.print("valve set to ");			//GIVE CONTEXT
        lcd.print(currentSamplePort);		//REPORT CURRENT POSITION
       delay(1000);							//WAIT FOR THE HUMAN MIND TO CATCHUP
	   
   while (currentSamplePort != initialInputPort) {
   //IF THE CURRENT SAMPLE PORT IS NOT THE SAME AS THE ONE SPECIFIED ABOVE (
   //initialInputPort) ADVANCE THE VALVE AND CHECK AGAIN, UNTIL THEY _ARE_ THE SAME
        ValveAdvanceBasic(); 				//advance 12 port sample valve
        currentSamplePort=CheckValve();		//check its position
        lcd.clear();  						//CLEAR THE LCD SCREEEN
		lcd.setCursor(1,0); 				//SET THE POSITION C1 R0
		lcd.print("VALVE INIT");			//GIVE ROUTINE HEADER
        lcd.setCursor(1,1); 				//SET POSITION C1 R1
		lcd.print("valve : ");				//GIVE CONTEXT
        lcd.print (currentSamplePort);		//PRINT CURRENT POSITION
   }//end while(currentSamplePort !=initialInputPort)
   lcd.clear();								//CLEAR THE SCREEN AGAIN FOR THE NEXT
											//PROCESS
 }//end InitializeTwelvePortinputPort
 
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/ 
 /// ADVANCE SOLENOID VALVE
 void ValveAdvanceBasic(){
      digitalWrite(solenoid, HIGH);	//activate the solenoid
      delay(350);        			//Solenoid opened for 3/8  of a second 
									//well. Brad, this isn't technically 3/8,(0.375) 
	  digitalWrite(solenoid, LOW);  //de-activate the solenoid
      delay(500);					//wait a bit
 } //END void ValveAdvanceBasic()
 
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
 ///EVALUATE CURRENT POSITION OF PNEUMATIC VALVE
 int CheckValve() {
   int currentPosition=0;
   digitalWrite(checkSignal, HIGH); // Turns on a 5V signal that goes to 
									//common tab on 12-way valve's rotary switch
									//look for it on each of the inputPorts to 
									//discover the current position
   int valveCount=0;	
   for (int i=0; i<=11; i++){
			//even numbered Arduino channels from 28-50 are assigned to the 
			//12-port valve. So, we move through them by twos and look for the 
			//+5v signal.
		//valveCount++;				//augment current valve--for some reason input
									//ports are counted by 1s but the pins are
									//assigned by 2s. the latter part of this makes
									//perfect sense based on the physical layout of
									//the i/o pins. the former...just makes book-
									//keeping a bit more tricky
		if(digitalRead(port[i])==HIGH){	//IF THE CURRENT VALVE HAS THE HIGH SIGNAL
				currentPosition=i+1;//ITS THE RIGHT ONE, ASSIGN IT TO 
										   //currentPosition AND RETURN IT
			}//end if(digitalRead(i)==HIGH)
			
	  } //END for (int i=inputPort1; i>=inputPort12; i=i-2)
	  
    return currentPosition;			//RETURN THE VARIABLE
	
 }//end CheckValve
 
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
///VALVE ADVANCE BUTTON
 void ValveAdvance() {
   int count=0;							//INITIALIZE DEBOUNCING VARIABLE
     while(digitalRead(valveAdvanceSwitch) && count <4){ 
	 //If the button is depressed (HIGH), debounce for .2 seconds and advance valve
	 //if the button continues to be held down, exit loop and enter again, allowing
	 //multiple advances
			delay(50);						//A BIT TO WAIT FOR DEBOUNCING
            if(count == 3){					//on the 4rth pass enter
                  lcd.clear();   			//CLEAR LCD
				  lcd.setCursor(1,0); 		//SET POSITION C1 R0
				  lcd.print("Valve Advance ");//PRINT FUNCTION HEADER
                  ValveAdvanceBasic();		//EXCECUTE 12 PORT SAMPLE VALVE ADVANCE
                  lcd.setCursor(1,1); 		//POSITION CURSOR C1 R1
				  lcd.print(CheckValve());  //PRINT CURRENT POSITION OF SAMPLE VALVE
            }//END if(count == 3){
		count++;  						//AUGMENT DEBOUNCING VARIABLE  
   }//END while(digitalRead(valveAdvanceSwitch) & count <4)
lcd.clear();								//CLEAR THE SCREEN
 }//END void ValveAdvance() 
 
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
///SAMPLE COLLECTOR ADVANCE SWITCH  TELL THE FRACTION COLLECTOR TO ADVANCE ONE
///TUBE
 void SampleCollectorAdvance() {
   int count=0;								//INITIALIZE A DEBOUNCING VARIABLE
    
     while(digitalRead(sampleCollectorAdvanceSwitch)){                        
			// While the  button is depressed (HIGH)                                                           
			delay(100); //WAIT A BIT TO ENSURE INTENTIONAL SIGNAL
            if(count == 3){//ON THE 4RTH TIME THROUGH THE LOOP
             lcd.clear();					//CLEAR LCD  
			 lcd.setCursor(1,0);  			//SET POSITION
			 lcd.print("SC Advance");		//PRINT ROUTING HEADER
             digitalWrite(sampleCollectorAdvance, HIGH);//TURN SAMPLE COLLECTOR ON
             delay(3000);					//WAIT FOR THE CARRIAGE TO ADVANCE
											//IT IS PRETTY SLOW
             digitalWrite(sampleCollectorAdvance,LOW);//TURN SAMPLE COLLECTOR OFF
			  						//INCREMENT DEBOUNCING VARIABLE
           }count++;//END if(count == 3)
     } //end while(digitalRead(sampleCollectorrAdvanceSwitch)
   lcd.clear();								//CLEAR LCD SCREEN
 }//end SampleCollectorAdvance
 
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
///RUN A SINGLE SAMPLE CYCLE FROM THE CURRENT INPUTPORT
///CALLED BY MANUAL AND PROBE DRIVEN SAMPLE ROUTINES
 
 void StartSampleCycle(int Collect){ 
	//If collect is specified (>0), THE SAMPLE SHOULD BE COLLECTED 
	
     int sampleCycle = 0;		//keep track of how many times we have been through
								//the inputPorts specified for collection
     int countDown=4;			//JUST A NICE LITTE FEEDBACK TO SHOW THE USER A
								//SAMPLE IS TO BE TAKEN SOON
     int inputPort=CheckValve(); //CHECK CURRENT INPUT PORT
     lcd.clear(); 				//CLEAR LCD
	 lcd.setCursor(1,0);  		//SET CURSOR POSITION C1 R0
	 lcd.print("SampleCycle V:"); //PRINT SAMPLE CYCLE V: AND THE INPUT PORT
	 lcd.print(CheckValve());	//(THERE ISN'T MUCH SPACE ON THE SCREEN)
	 
   // IF A SAMPLE SIGNAL IS RECIEVED (HIGH), debounce for .3 seconds and BEGIN
     while(digitalRead(startSampleCycleSwitch) || sampleSignal==1){       
          sampleCycle++;					//AUGMENT SAMPLE CYCLE
          delay(100);						//WAIT A BIT
		  if(sampleCycle%5 == 0){			//GIVE FEEDBACK EVERY 5XDELAY SEC
            countDown--;					//DECREMENT COUNTDOWN VARIABLE
                lcd.setCursor(1,1);    		//SET POSITION C1,R1
				lcd.print(countDown);		//PRINT COUNTDOWN
          }//end if(sampleCycle%5...
         
         
          if(sampleCycle == signalTime*10){ //SIGNAL TIME IS DECLARED ABOVE
											//2 SO AFTER 20X THROUGH THE LOOP
											//2 SEC, START THE CYCLE
           long int startTime=millis();		//STORE START TIME
           sampleSignal=0;					//RESET SAMPLESIGNAL
		   
///step1 pump sample through system to forward purge A
           long int pumpTime=pumpTimeArray[inputPort-1];//how long to PUMP
		   //PUMP TIMES ARE STORED IN THE ARRAY pumpTimeArray
		   //and indexed from 0 to 11 (that is inputPort-1)
		   //because input ports are numbered 1 to 12
		  
           genericSampleFunction(pumpTime,purge,purgeA,"SampleCycle","purge A: ");
           //call the pump function with ([pumpTime],[pump or purge],
		   //[array of sample direction valves to open],[header text],[lower text])
                    //THIS ROUTINE WILL CHECK IF ANY OF THE  INLET PORTS ARE LEAKING
                    //WAITS 30 sec AND THEN ADVANCES TO NEXT PORT, GOES THROUGH ALL 12 PORTS
                    

           }//end if(sampleCycle == signalTime*10)
          
     }//end while(digitalRead(startSampleCycleSwitch...
     lcd.clear();
 }//end startSampleCycle
 
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
 ///A SAMPLE FUNCTION THAT CAN BE CALLED FROM THE AUTOMATED SAMPLE ROUTINE
 ///IT IS MEANT TO BE ABLE TO PUMP OR PURGE AS WELL AS DIRECT THE SAMPLE TO 
 ///ANY OF THE 3 OUTPUT POSITION (FRACTION COLLECTOR, PURGE, PROBE)
 ///FROM EITHER THE SOURCE INPUTPORT (FORWARD PUMP), OR FROM THE THE PROBES 
 ///(REVERSE PUMP)
 
  void genericSampleFunction(long int duration, int pumpOrPurge, int valves[3], 
						char phrase1[16],char phrase2[16]) {//RunThroughSampleCycle
         lcd.clear();   	//CLEAR LCD SCREEN
		 lcd.setCursor(1,0);//SET CURSOR POSITION C1R0
		 lcd.print(phrase1);//PRINT FUNCTION HEADER
		 lcd.setCursor(1,1); //SET CURSOR POSITION C1R1
		 lcd.print(phrase2); //PRINT CURRENT ACTIVITY UPDATE
         long int time=millis(); //GET TIME FOR START OF CYCLE
         int loopCounter=0;		//INITIALIZE LOOPCOUNTER
///WRITE EACH OF THE INTERNAL SAMPLE ROUTING 3-WAY VALVES IN ORDER TO DIRECT
///THE SAMPLE TO THE DESIRED OUTPUT
		//THE VECTOR VALVES CONTAINES THE DESIRED STATE OF EACH VALVE 
		//(HIGH OR LOW)
         digitalWrite(collectOrPurge3Way,valves[0]); //WRITE VALVE 
         digitalWrite(inputOutput3Way,valves[1]);	 //WRITE VALVE
         digitalWrite(probeOrPurge3Way,valves[2]); 	 //WRITE VALVE
		 //PUMP OR PURGE CONTAINS THE INTEGER REFERRING TO FORWARD OR REVERSE
		 //PUMPING BY THE SYSTEM PERISTALTIC PUMP
		 delay(250);								//ENSURE THAT VALVES HAVE 
													//OPENED FULLY BEFORE ACTIVATING
													//PUMP
         digitalWrite(pumpOrPurge,HIGH);			//TURN PUMP ON
         							
        while(millis()-time<=duration*1000){//until time is satisfied
			if (millis()%100==0){
				//EVERY 100MS UPDATE THE DISPLAY TO SHOW TIME REMAINING
                 long int elapsedTime=((millis()-time)/1000);
                 lcd.setCursor(1,1); 		//SET CURSOR C1R1
				 lcd.print(phrase2); 		//PRINT SECOND PHRASE CYCLE STATUS
				 lcd.print(duration-elapsedTime); //PRINT REMAINING TIME
              }//END  if (millis()%100==0)
              
          }//END while(millis()-time<=duration*1000)
       
	   digitalWrite(pumpOrPurge,LOW);		//TURN THE SYSTEM PUMP OFF
       delay(250);							//ALLOW SOME TIME FOR PRESSURE TO
											//EQUALIZE
	  //MAKE SURE THAT EACH INTERNAL 3-WAY VALVE IS TURNED OFF
	   digitalWrite(collectOrPurge3Way,LOW); //TURN VALVE OFF
       digitalWrite(inputOutput3Way,LOW);	 //TURN VALVE OFF
       digitalWrite(probeOrPurge3Way,LOW);	 //TURN VALVE OFF
        }//end genericSampleFunction

/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
///called from main loop by the activation of individual pump buttons, switches
///and dials. It is pretty flexible and probably save lots of annoying
///repetition of code. The function takes the following aruguments:
//singalSwitch  	integer identifier for the switch that called the function
//duration: 		integer time variable--actually not used here
//pump or purge:  	integer identifier linked to aruino i/o port that 
//					activates the motor clockwise or counterclockwise
//valves[3]			integer vector holding state of valves necessary to move
//					sample to desired output port
//phrase1			char array for display of major function
//phrase2			char array for display of minor activity
void genericPumpFunction(int signalSwitch, int duration,int pumpOrPurge, 
						int valves[3], char phrase1[16],char phrase2[16]) { 
   long int counter = 0;		//INITIALIZE A COUNTER VARIABLE
   long int cycleTime=millis();	//INITIALIZE A TIME KEEPING VARIABLE
   
   lcd.clear();					//CLEAR LCD					
   lcd.setCursor(1,0);     		//SET CURSOR POSITION C1R0
   lcd.print(phrase1);			//PRINT MAJOR FUNCTION PHRASE ON UPPER LINE
   lcd.setCursor(1,1);     		//SET CURSOR POSITION C1R1
   lcd.print(phrase2);			//PRINT MINOR FUNCTION PHRASE (ON LOWER LINE)
   
while(digitalRead(signalSwitch)){ 
// If the button is depressed (HIGH), debounce for .2 seconds and purge
      counter++;				//AUGMENT COUNTER
      delay(50);				//DELAY TO DEBOUNCE
      if(counter == 4){			//IF DEBOUNCED
		   //WRITE THE VALVE POSITIONS ACCORDING TO THE VECTOR PASSED FROM THE 
		   //FUNCTION CALL
           digitalWrite(collectOrPurge3Way,valves[0]);
           digitalWrite(inputOutput3Way,valves[1]);
           digitalWrite(probeOrPurge3Way,valves[2]); 
           delay(250); 	//DELAY A BIT TO ALLOW THEM TO FULLY OPEN BEFORE PUMP
						//IS ACTIVATED
           digitalWrite(pumpOrPurge, HIGH); 	//ACTIVATE PUMP (CW OR CCW AS
												//SPECIFIED IN FUNCTION CALL
          
           while(digitalRead(signalSwitch)){
			//AS LONG AS THE SWITCH SPECIFIED IN THE FUNCTION CALL IS HIGH
			//CONTINUE PUMPING
                if(millis()%250==0){       
					//UPDATE SERIAL DISPLAY WITH ELAPSED TIME
					//EVERY 250 MS
					lcd.setCursor(1,1);  		//SET CURSOR POSITION C1R1
					lcd.print(phrase2);  		//PRINT 2ND PHRASE
					lcd.print((millis()-cycleTime)/1000);//PRINT ELAPSED TIME
                  }//END if(millis()%250==0)
				  
          
			  } //end while(digitalRead(signalSwitch)) 
		//TURN OFF PUMP AND VALVES	  
		 digitalWrite(pumpOrPurge, LOW);		//TURN PUMP OFF
		 delay(250);							//WAIT A BIT FOR PRESSURE TO 
												//EQUALIZE
		 digitalWrite(collectOrPurge3Way,LOW);	//ENSURE VALVE IS CLOSED
		 digitalWrite(inputOutput3Way,LOW);		//ENSURE VALVE IS CLOSED
		 digitalWrite(probeOrPurge3Way,LOW);	//ENSURE VALVE IS CLOSED
    }  //end if(counter == 3)
	
  } //end while(digitalRead(signalSwitch))
  
lcd.clear(); 									//CLEAR LCD FOR THE NEXT FUNCTION
 }  //end genericPumpfunction
 
/*********************************************************************************/
/*********************************************************************************/
/*********************************************************************************/
 ///AN AUTOMATED SAMPLE CYCLE THAT LISTENS FOR SIGNALS FROM THE SCAN PROBE (OR SOME 
 ///OTHER SIGNAL GENERATING DEVICE
 //TAKES FOLLOWING ARGUMENTS:
 //samplePorts--valves from which to sample through the cycle--specified above
 //collectInterval--if >0 the first sample is collected, and every nth subsequent
 //					sample 2-every other 3-every third etc
void ProbeDrivenSampleCycle(int samplePorts[12], int collectInterval){
  int sampleLoopCounter=1;		//KEEP TRACK OF HOW MANY TIMES THE SYSTEM HAS
								//CYCLED THROUGH ALL OF THE SPECIFIED VALVES
  int Continue=1;				//FOR BOOTING THE SYSTEM OUT OF THE FUNCTION
								//IF SOMETHING HAS GONE WRONG 0=FAIL 1=OK
  int currentinputPort=1;		//??
  int nextinputPort;			//
  int sum=0;					//	
  long int cycleTime=millis();	//A TIMER FOR THE BEGINNING OF THE FUNCTION CALL
 
 ///A LITTLE ROUTINE TO MAKE SURE THAT VALVES HAVE BEEN SPECIFIED IN THE 
 ///DECLARATIONS ABOVE (of smplePorts vector). IF NOT VALVES SPECIFIED, SUM=0 AND 
 ///THE LOOP IS EXITED
   for (int failSafe=0; failSafe<12; failSafe++){
         sum=sum+samplePorts[failSafe];
     } //END for (int failSafe=0; failSafe<12; failSafe++)
     
   if (sum==0){
       Continue=0; 
	   lcd.print("no valves set"); 
	   delay(3000);
   }//END if (sum==0){
   
   currentinputPort=1;			//??
   
         lcd.clear(); 					//CLEAR LCD SCREEN
		 lcd.setCursor(0,0); 			//SET CURSOR POSITION C0 R0
		 lcd.print("Probe Sample: ");	//PRINT MAIN FUNCTION
		 lcd.print(sampleLoopCounter);	//REPORT HOW MANY CYCLES HAVE PASSED
										//SINCE SYSTEM ENTERED PROBE-SAMPLE
         delay(2000);					//WAIT A BIT...
		 
///WAIT FOR PROBE SIGNAL    
  while(Continue==1){ //while waiting for probe signal 
      int collect=0;  //INITIALIZE COLLECT VARIABLE
///ALWAYS COLLECT SAMPLES THE FIRST TIME THROUGH THE LOOP
     if (sampleLoopCounter==1){ 
			collect=1;
			}//END if (sampleLoopCounter==1)
	//EVERY 0.25 SECONDS UPDATE SYSTEM STATUS ON LCD   
     if(millis()%250==0){ 
          lcd.setCursor(0,0); 				//SET CURSOR POSITION
		  lcd.print("wait "); 				//REPORT STATUS
		  lcd.print((millis()-cycleTime)/1000); //PRINT ELAPSED TIME
          lcd.setCursor(1,1); 				//ON SECOND LINE PRINT
		  lcd.print("Cy#"); 				//
		  lcd.print(sampleLoopCounter); 	//CYCLE#
		  
		  lcd.print(" V# ");				//NEXT SHOW THE VALVE # 
		  lcd.print(CheckValve()); 			//PRINT CURRENT VALVE
		  lcd.print(" SC: ");				//WHETHER OR NOT SAMPLE WILL
											//BE COLLECTED
		  lcd.print(collect);				//0 OR 1
        }//END if(millis()%100==0)
		
///DECIDE WHETHER THE CURRENT CYCLE SHOULD BE SENT TO THE FRACTION COLLECTOR		
     if ((sampleLoopCounter+1)%collectInterval==0){ 
				collect=1;	//or in the nth pass through the collection cycle
		} //END  if ((sampleLoopCounter+1)%collectInterval==0)
		
///CYCLE THROUGH ALL TWELVE VALVES, USE ONLY THE ONES THAT ARE SPECIFIED IN THE 
///samplePorts VECTOR (SPECIFIED ABOVE)  
    for (int i=12; i>=1; i--)  {
        currentinputPort=i;
          if(samplePorts[currentinputPort-1]==1){ //make sure its the right one
           InitializeTwelvePortinputPort(i);	//get the valve to the right position


             while(digitalRead(probeSignal)==0){
				///WAIT FOR THE PROBE TO SIGNAL
				///WHEN probeSignal GOES HIGH (ON PROBE SIGNAL) EXIT LOOP AND PROCEED
				///THROUGH THE SAMPLE ROUTINE
				//in the mean time give feedback to the LCD	to show that everything
				//is proceedingly swimmingly
				   if(millis()%250==0){ //every 0.250 second give some feedback
						  lcd.setCursor(0,0); 			//SET CURSOR
						  lcd.print("wait ");			//PRINT ACTIVITY
						  lcd.print((millis()-cycleTime)/1000);//PRINT ELAPSED TIME
						  lcd.setCursor(1,1); 			//ON SECOND LINE
						  lcd.print("Cy#"); 			//PRINT CYCLE#
						  lcd.print(sampleLoopCounter); 
						  lcd.print(" V#"); 			//PRINT CURRENT INPUT PORT
						  lcd.print(CheckValve()); 
						  lcd.print(" SC: ");			//PRINT WHETHER OR NOT SAMPLE
						  lcd.print(collect);			//WILL BE SENT TO FRACTION 
														//COLLECTOR
					    }//end if(millis()%100==0)
				
               } //end  while(digitalRead(probeSignal)==0)
			 
			 //SEND FEEDBACK TO THE LCD PANEL
                lcd.clear(); 							//CLEAR LCD
				lcd.print("Signal Received ");			//REPORT SIGNAL RECEPTION
                lcd.setCursor(1,1); 					//ON LINE TWO
				lcd.print("Cy#"); 						//PRINT CYCLE
				lcd.print(sampleLoopCounter); 
				lcd.print(" V#"); 						//PRINT VALVE
				lcd.print(CheckValve()); 
				lcd.print(" SC: ");						//REPORT WHETHER OR NOT 
														//TO SAMPLE
				lcd.print(collect);  
               
			   delay(500);		  //WHO KNOWS WHY THIS IS HERE...
               sampleSignal=1;    //this is  cheap way to make the sample collector 
								  //routine work, consider making it less sneaky
               StartSampleCycle(collect); //run the sample cycle
                delay(1500); 		      //WAIT A WHILE AFTERWARDS
				
			}//if(samplePorts[currentinputPort-1]==1)
			
///AT THE END OF A CYLCE THROUGH THE 12 PORTS UPDATE THE SAMPLE LOOP COUNTER
      if (i==12){
		sampleLoopCounter++; //MADE IT THROUGH 12 PROTS; AGUMENT CYCLE COUNTER
		}//END  if (i==12)
		
  }//end for (int i=1; i<=12; i++) 
  
  }//END while(Continue==1)
  
}//end ProbeDrivenSampleCycle

//
