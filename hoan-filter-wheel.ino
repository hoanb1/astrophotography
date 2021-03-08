
//Hoan Rayz_filter_wheel. ver .1 currently compatable with indi-Xagyl

#include <AccelStepper.h>
#include <EEPROM.h>

//Global declarations
long fullWheel = 60000;
//word posOffset[] = { 0, 750, 6050, 11350, 16650, 21950 };
word posOffset[] = { 0, 300, 5600, 10900, 16200,  21500};
bool Error = false;                      // Error flag

byte currPos = 1;                        // Start up with 1
int newPos = 1;                          // Just start somewhere
bool cmdOK = false;                      // Command ok ?
int PWMvalue = 128;                      // Set PWM to half
int CalibrationOffset = 0;               //for indi info
int maxSpeedX = 500;                      // max motor speed
int currentSpeed = maxSpeedX;                      //current step speed (apparently max is used by the library?)
int setAccel = 1000;                     //
int jitter = 5;                      //fake jitter changes to please indi
int threshold = 30;  // fake threshold change to please indi
int filterNumber = 5;
bool oneWayderection = true;
bool findHome = false;
long targetPosition = 0;
bool motorIsrunning = false;
bool manualGoto = false;
bool debug = false;

unsigned long time_a;
long timemer = 5000;
// Hall pin definition
/*
     reedsw. wiring
     a1 = common
     a3 = input --pullup
     hall effect wiring
    A0 = d0 digital(comparator)output
    A1 = vcc
    A2 = gnd
    A3 = a0 analog out
*/

const int SENSOR = 10;       // PIN A3 = Hall effect switch - onboard comparator with predefined value (red)
//const int hallPower = A2; //(a1 orig)    // PIN A1 = Hall effect supply - pull high   (blue)
//const int hallCom  = A1;  //(a2 orig)   // PIN A2 = Hall effect common - pull low     (black)
//const int d0 = A0;           // hall effect analog input - not used 2/18/18 (green)

// Motor definitions
#define STEPS 4                        // 28BYJ-48 steps 4 or 8

// Motor pin definitions
#define motorPin1 2// 9         //6 IN1 on ULn2003
#define motorPin2 3 // 6         //9 IN2 on ULn2003
#define motorPin3 4// 8         //7 IN3 on ULn2003
#define motorPin4 5 // 7         //8 IN4 on ULn2003

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
AccelStepper stepper(STEPS, motorPin1, motorPin3, motorPin2, motorPin4);

void(* resetFunc) (void) = 0;//declare reset function at address 0

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  Serial.println("FW1.0.0");

  // pinMode(hallCom, OUTPUT);         //hall gnd
  // pinMode(hallPower, OUTPUT);       //hall power
  // pinMode(SENSOR, INPUT_PULLUP);           //(A3) hall signal
  pinMode(motorPin1, OUTPUT);               //motor +a
  pinMode(motorPin2, OUTPUT);               //motor -a
  pinMode(motorPin3, OUTPUT);               //motor +b
  pinMode(motorPin4, OUTPUT);               //motor -b

  //digitalWrite(hallCom, LOW);     // set hall gnd to 0v
  //digitalWrite(hallPower, HIGH);    //set hall power to 5v


  //  Serial.setTimeout(1000);
  // Set stepper stuff
  currPos = newPos =  EEPROM.read(0);
  debugMsg("Load Current Pos "+ String(currPos));  
  targetPosition = posOffset[currPos];
  stepper.setCurrentPosition(posOffset[currPos]);
  // stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(maxSpeedX);    //maximum step rate
  stepper.setSpeed(currentSpeed);       //current step rate
  stepper.setAcceleration(setAccel);     //"1000 = 100%" accellerationt to set step rate --check this in accelStepper

   debugMsg("Read Offset from EEROM");
   int i=0;
   int eeAddress = 0;
   long Value = 0;
   for ( i = 1; i <= filterNumber ; i++ ) {
    Value = posOffset[i];
      eeAddress = (i-1)*sizeof(Value)+1;  
      EEPROM.get( eeAddress, Value );
      if(Value != 0 ){
         posOffset[i] = Value;
         debugMsg("Offset "+String(i) +" Value "+ String(Value)) ;       
      }
    }

  //intial calibration.
  //
  // Locate_Home();                          // Rotate to index 0
  //  Locate_Slot_x();                             //go to first filter offset

  Serial.flush();

} // **** END OF SETUP ****

void loop() {

  //Serial.print("Hall sensor ");
  //Serial.println(digitalReadRand(SENSOR));
  //delay(500);
  //return;

  Non_blockRun();

  String inLine;
  char command;
  // Get incoming command.
  if (Serial.available() > 0) {
    inLine = Serial.readStringUntil('\n');
  }

  command = inLine.charAt(0);         // command+newPos=Goto pos newPos
  char param = inLine.charAt(1); // newPos=int -48 if no input..
  int paramNumber = int(inLine.charAt(1) - 48);

  // Go clockwise...
  if ( inLine == ")0" ) {
    cmdOK = true;
    //posOffset[currPos] = posOffset[currPos] + 10;
    //   newPos = currPos;
    // Locate_Home();
    // Locate_Slot_x();
    Serial.print("P");
    Serial.print(getIndexPos());
    Serial.print(" Offset ");
    Serial.println( posOffset[getIndexPos()] );
    return;
  }

  // Go counterclockwise...  decrements offset value/ not actually reverse
  if ( inLine == "(0" ) {
    cmdOK = true;
    // posOffset[currPos] = posOffset[currPos] - 10;
    // newPos = currPos;
    // Locate_Home();
    // Locate_Slot_x();
    Serial.print("P");
    Serial.print(getIndexPos());
    Serial.print(" Offset ");
    Serial.println( posOffset[getIndexPos()] );
  }

  // Hall-sensor data..
  if ( inLine == "T0" || inLine == "T1") {
    cmdOK = true;
    Serial.print("Sensors ");
    Serial.print(digitalReadRand(SENSOR));
    Serial.print(" ");
    Serial.println(digitalReadRand(SENSOR));
  }

  // Hall-sensor data..
  if ( inLine == "T2" ) {
    cmdOK = true;
    Serial.println("MidRange 520");
  }


  // Hall-sensor data..
  if ( inLine == "T3" ) {
    cmdOK = true;
    Serial.print("RightCal ");
    Serial.println(digitalReadRand(SENSOR) - digitalReadRand(SENSOR));
  }

  // Product name
  if ( inLine == "I0" ) {
    cmdOK = true;
    Serial.println("Xagyl FW Arduino");
  }

  // Firmware version
  if ( inLine == "I1" ) {
    cmdOK = true;
    Serial.println("FW1.0.0"); //must begin with FW and have only digits after. (RC= "FW" %d)
  }
  // Current filter pos
  if ( inLine == "I2" ) {
    cmdOK = true;
    Serial.print("P");
    Serial.println(getIndexPos());
  }

  // Serial number
  if ( inLine == "I3" ) {
    cmdOK = true;
    Serial.println("SN.001");
  }

  // Display the maximum rotation speed - "MaxSpeed XXX%"
  if ( inLine == "I4" ) {
    cmdOK = true;
    Serial.println(maxSpeedX);
  }
  // Display the jitter value - "Jitter XX", XX = values 1-10
  if ( inLine == "I5" ) {
    cmdOK = true;
    Serial.println("Jitter 5");
  }
  // Display sensor position offset for current filter position - "PX Offset XX"
  if ( inLine == "I6" ) {
    cmdOK = true;
    Serial.print("P");
    Serial.print(getIndexPos());
    Serial.print(" Offset ");
    Serial.println( posOffset[getIndexPos()] );
  }
  // Display filter position sensor threshold value - "Threshold XX"
  if ( inLine == "I7" ) {
    cmdOK = true;
    Serial.println("Threshold 30");
  }
  // Display the number of available filter slots - "FilterSlots X"
  if ( inLine == "I8" ) {
    cmdOK = true;
    Serial.print("FilterSlots ");
    Serial.println(filterNumber);
  }
  // Display the Pulse Width value - "Pulse Width XXXXXuS" : PWMvalue 0..255
  if ( inLine == "I9" ) {
    cmdOK = true;
    Serial.print("Pulse Width ");  // Default 1500uS, Range 100 - 10000
    Serial.println("4950uS");      // 10000-100/2=4950 Just a value
  }



  // Increase pulse width by 100uS, Displays ?Pulse Width XXXXXuS?
  if ( inLine == "M0" ) {
    cmdOK = true;
    PWMvalue = PWMvalue + 100;
    Serial.print("Pulse Width ");
    Serial.print(PWMvalue);
    Serial.println("uS");
  }


  // Decrease pulse width by 100uS, Displays ?Pulse Width XXXXXuS?
  if ( inLine ==  "N0" ) {
    cmdOK = true;
    PWMvalue = PWMvalue - 100;
    Serial.print("Pulse Width ");
    Serial.print(PWMvalue);
    Serial.println("uS");
  }

  // Decrease filter position threshold value, Displays Displays "Threshold XX"
  if ( inLine ==  "{0" ) {
    cmdOK = true;
    if (threshold > 0) {
      threshold = threshold - 10;
    }
    Serial.println(threshold);
  }

  // Increase filter position threshold value, Displays Displays "Threshold XX"
  if ( inLine ==  "}0" ) {
    cmdOK = true;
    if (threshold < 100) {
      threshold = threshold + 10;
    }
    Serial.println(threshold);
  }

  // Decrease jitter window by 1, Displays Displays ?Jitter X? value 1-10.
  if ( inLine ==  "[0" ) {
    cmdOK = true;
    if (jitter > 0) {
      jitter--;
    }
    Serial.println(jitter);
  }

  // Increase jitter window by 1, Displays Displays ?Jitter X? value 1-10.
  if ( inLine ==  "]0" ) {
    cmdOK = true;
    if (jitter > 10 ) {
      jitter++;
    }
    Serial.println(jitter);
  }

  // Hard reboot --this should be a cpu reset command
  if ( inLine == "R0" ) {
    Serial.println("Reset in 3 seconds");
    delay(3000);         
    resetFunc();  //call reset
  }

  // Initialize, restarts and moves to filter position 1.
  if ( inLine == "R1" ) {
    cmdOK = true;
    Locate_Home();  // currPos will be 1.
    //delay(500);
    Serial.print("P");
    Serial.println(getIndexPos());
  }


  // Reset all calibration values to 0. Handy for first run/calibrating,
  // if you then save zeroes to eprom with G0
  if ( inLine == "R2" ) {
    cmdOK = true;
    oneWayderection = !oneWayderection;
    //currPos = 1;
    // Never do that
    /*
      for ( int i = 1; i < 6 ; i++ ) {
      posOffset[i] = 0;
      if (i == 5) {
        Serial.println("Calibration Removed");
      }
      }
    */
    Serial.println("Calibration Removed");
  } //end of inline commands

  // Reset Jitter value to 1, displays "Jitter 1"
  if ( inLine == "R3" ) {
    cmdOK = true;
    Serial.println("Jitter 5");
  }

  // Reset maximum carousel rotation speed to 100%, displays "MaxSpeed 100%"
  if ( inLine == "R4" ) {
    cmdOK = true;

    Serial.println("MaxSpeed 100%");
  }


  // Reset Threshold value to 30, displays "Threshold 30"
  if ( inLine == "R5" ) {
    cmdOK = true;
    Serial.println("Threshold 30");
  }

  // Calibrate, No return value displayed.
  if ( inLine == "R6" ) {
    cmdOK = true;
    // delay(1000);
  }


  // Send filter offset value  - "PX Offset XXXX"
  if (command == 'O') {
    cmdOK = true;
    Serial.print("P");
    Serial.print(paramNumber);
    Serial.print(" Offset ");
    Serial.println( posOffset[paramNumber] );
  }
  // Set Offset of current filter to array. (save to eprom with G0)
  if ( command == 'F' ) {
    cmdOK = true;
    int postIndex = (inLine.substring(1, 2).toInt());
    posOffset[postIndex] = (inLine.substring(3).toInt());   //get the int value from the serial string
    //  Locate_Home();
    // Locate_Slot_x();
    Serial.print("P");
    Serial.print(postIndex);
    Serial.print(" Offset ");
    Serial.println( posOffset[postIndex] );
  }
  if (command == 'S' ) {
    String input = inLine.substring(1);
    if (input == "A") {
      input = "10";
    }
    int setSepped = (input.toInt());   //get the int value from the serial string
    if (setSepped > 0) {
      currentSpeed = setSepped * maxSpeedX * 0.1;

      //stepper.setMaxSpeed(maxSpeedX);    //maximum step rate
      stepper.setSpeed(currentSpeed);       //current step rate
      Serial.print("Speed=");
      Serial.print(setSepped * 10);
      Serial.println("%");
    } else {
      Serial.print("Speed=");
      Serial.print(currentSpeed);
      Serial.println("rpm");
    }
  }

  if (command == 'D' ) {
    debug = !debug;
    if(debug){
      debugMsg("Debug on");
    }
    
  }
  if (command == 'P' ) {
    manualGoto = true;
    targetPosition = inLine.substring(1).toInt(); 
    Serial.print("Manual motor: ");
    Serial.print(stepper.currentPosition());
    Serial.print(" -> ");                            
    Serial.println(targetPosition);
      Non_blockRun();
  }
  // Store offsets to eprom..
  if ( inLine == "G0" ) {
    cmdOK = true;
    epromSave(); //store offsets to eprom and display contents
    return;
  }

  if ( paramNumber == -48 ) {
    return;
  }
  int nextPos =0;
  if ( command == 'G') {
    cmdOK = true;
    nextPos = paramNumber; // newPos=int -48 if no input..
    if (  nextPos >= 1 || nextPos <= filterNumber  ) {
      if(newPos==currPos){ // Motor is not running.
        newPos = nextPos;
        if ( newPos != currPos ) {
          Locate_Slot_x();
        }
      }else{
        debugMsg("Motor is running. Skeep command");
      }
    }
    Serial.print("P");
    Serial.println(getIndexPos());
    Serial.println(stepper.currentPosition());
    // delay(100);
    }


  // If command not recognized, flush buffer and wait for next command..
  if ( cmdOK ) {

    //  Serial.println("Error: Command not found");
    Serial.flush();
    cmdOK = false;
    // delay(100);
    return;
  }
}   ////////// End of main loop.. //////////


int digitalReadRand(int sensor) {
  int HallVl = digitalRead(SENSOR);
 // debugMsg("Hall value: "+String(HallVl));
  return HallVl;
}

int getIndexPos() {
  if(currPos == 0){
    return 1;
  }
  return currPos;
  //if(debug){
   // int currentPosition = stepper.currentPosition();
    //  for (int i = filterNumber; i > 0; i--) {
     //   if ( posOffset[i] <= currentPosition) {
      //    return i;
       // }
   // }
   // return 1;
 //}else{
  // return currPos;
 //}
}
void Non_blockRun() {
  int currentPosition = stepper.currentPosition();
  
  if ( currPos == newPos && !findHome && !manualGoto) {
    motor_Off();
    return;
  }
  int HallValue = digitalReadRand(SENSOR);
  if (findHome) {     

    if (!motorIsrunning) {
      stepper.moveTo(fullWheel);
      debugMsg("Finding Home"); 
     // debugMsg(String(fullWheel));
      motorIsrunning = true;
    }
    
    //    if (findHome && HallValue == LOW && stepper.distanceToGo() <= 0 ) {
    //      Serial.println("Error:ET Can't find home");              //if it runs too long stop and error
    //      motor_Off();
    //      Error = true;
    //      findHome = false;
    //    }

  } else {    
    if (currentPosition == targetPosition) {
      if(manualGoto){
          manualGoto = false;
          motorIsrunning = false;
          motor_Off();
          debugMsg("Motor manual went to  "+String(currentPosition)); 
          return;
      }else{
          currPos = newPos;
          motorIsrunning = false;// Tell caller this is the new requested position
          debugMsg("Motor went to  "+String(currentPosition)); 
          motor_Off();          
          EEPROM.write(0, currPos);          
          debugMsg("Save Current Pos "+String(currPos));        
          return;
      }
    } else {
      if (!motorIsrunning) {
        stepper.moveTo(targetPosition);
        debugMsg("Go to "+String(targetPosition));      
        motorIsrunning = true;
      }

    }
  }
  time_a = millis();
  while (millis() - time_a < timemer) {
  //  debugMsg(String(millis() - time_a));
    stepper.run();
    if (findHome) {
      HallValue = digitalReadRand(SENSOR);
       if (HallValue == HIGH 
       && digitalReadRand(SENSOR) == HIGH 
       && digitalReadRand(SENSOR) == HIGH
       && digitalReadRand(SENSOR) == HIGH
       && digitalReadRand(SENSOR) == HIGH
       ) {
        motorIsrunning = false;        
        
        int homePos = stepper.currentPosition();
        stepper.stop();
        stepper.setCurrentPosition(0);
        currPos = 0;
        findHome = false;
        EEPROM.write(0, currPos);
        debugMsg("Finded home at "+String(homePos));
        motor_Off();
        break;
    }
    }
  }

}
//            *********Find Home********
void Locate_Home() {
 // debugMsg("Find Home");
  int HallValue = digitalReadRand(SENSOR);                      // read the hall sensor value
  if (HallValue == LOW) {
    //  Serial.println("Move motor 30000 to find Home position"); // recheck the hall sensor value
    findHome = true;
    Non_blockRun();
  }
}

void debugMsg(String msg) {
  if (debug) {
    Serial.println(msg);
  }

}
// Move to slots 1..5 ***************************************************
// Always run wheel the same direction to avoid backlash issues.

void Locate_Slot_x() {
  debugMsg("Goto "+String(currPos)+"->"+String(newPos));
  if (oneWayderection && posOffset[newPos] < posOffset[currPos] ) { //prevent backwards movement to avoid backlash errors
    Locate_Home();
  }
  targetPosition = posOffset[newPos];                          // Do not need Offset anymore
  Non_blockRun();
}

void motor_Off() {                                            //power down the stepper to save battery
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
}

// Show some values and reset error flag.
void epromSave() {
  debugMsg("Wite data to EEROM");
  int i = 1; 
  int eeAddress = 0; 
  long Value =0;
  long f = 0;
    for ( i = 1; i <= filterNumber ; i++ ) {
      Value = posOffset[i];
      eeAddress = (i-1)*sizeof(Value)+1 ;  
      
      EEPROM.put(eeAddress,Value);
      debugMsg("Adress "+String(eeAddress)+" Value: "+String(Value));
      
    }
    Serial.println("Current values written to EEPROM!!");
   
    for ( i = 1; i <= filterNumber ; i++ ) {
      Value = posOffset[i];
      eeAddress = (i-1)*sizeof(Value)+1 ; 
      EEPROM.get( eeAddress,Value);
      Serial.print("Offset: ");
      Serial.print( eeAddress );
      Serial.print(" = ");
      Serial.println( Value );
    }
  
    if ( !Error ) {
      Serial.println("Normal operation resumed");
    }
  
    Error = false;
  return;

}
