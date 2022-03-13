
  //////////////////////////////////////////////////
  // Project:  Lunar Explortation Mechanical Arm  //
  // Author:   Capstone Team 18                   //
  // Date:     08/03/22                           //
  // Version:  4.2                                //
  //////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////////////////////
  // This program is used for controlling the mechanical arm. It perfroms    //
  // the following functions:                                                //
  // - Turning motors on and off                                             //
  //    - Motors can be set to turn continuously or turn in small increments //
  // - Adjusting power of motors via PWM                                     //
  // - Reading position from rotary encoder                                  //
  // - Converting rotary encoder values to degrees of rotation               //
  /////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////
  // CONTROLS:                                          //
  //                                                    //
  // 1 - control motor 1                                //
  // 2 - control motor 2                                //
  // 3 - control motor 3                                //
  // 4 - control motor 4                                //
  // 5 - control motor 5                                //
  //                                                    //
  // 0 - Zero encoder                                   //
  //                                                    //
  // q - turn of motor                                  //
  // a - rotate clockwise                               //
  // z - rotate counter clockwise                       //
  // d - pulse clockwise                                //
  // c - pulse counter clockwise                        //
  //                                                    //
  // h - rotate motors 3 and 4 clockwise                //
  // n - rotate motors 3 and 4 counter clockwise        //
  // y - turn off motors 3 and 4                        //
  //                                                    //
  // j - rotate motors 3 and 4 in opposite directions   //
  // m - rotate motors 3 and 4 in reversed directions   //
  // u - turn off motors 3 and 4                        //
  //                                                    //
  // w - universal kill switch                          //
  // p - give motor 1 full revolution                   //
  ////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////// PROGRAM BEGIN ////////////////////////////////////////////////////////////////////////


// Global variable being declared. They are used later for reading keyboard input
int incomingByte = 0;   // for incoming serial data
char input = 'v';       // set to 'v' by dedault because that key isn't used for any of the controls. could have been anything else tho

//this class creates motor objects. You should have one Object per motor. Constructor takes in the 5 pins used for the motor

class Motor {
  public:

  int ID;     //ID number that identifies motor
  
  //  Variables to store pin numbers
  int enA;    // PWM
  int in1;    // Positive motor terminal    
  int in2;    // Negative motor termial
  int ENCA;   // Encoder channel A
  int ENCB;   // Econder Channel B


  // Variables used for movement 
  int     dir;        // Can take 3 values. dir=1 -> clockwise, dir=-1 -> counter clockwise, dir=0 -> no movement
  int     pwnVal;     // Pulse width modulation value [0-255]
  double  pos;        // Encoder position
  double  p1;         // Temporary value used for calculating range of rotation
  double  p2;         // Temporary value used for calculating range of rotation

  // Variables used for converting position to angle of rotation
  double ppr;         // Pulse per revolution of encoder
  double convert;     // gear ratio
  double increment;   // angle to rotate motor
  double error;       // compensates for error in encoders
 

  // Constructor method: Assigns pin values to variables. Sets motor direction to off. 
  Motor(int x, int y, int z, int a, int c, int d){
    
    pwnVal  = 240;   // Can take values between [0,255], however values below 100 sometimes cause performance issues
    enA     = x;
    in1     = y;
    in2     = z;
    ENCA    = a;
    ENCB    = c;
    ID      = d;
    dir     = 0;      //motor off by default
  }


// Method that sets the pins to the appropiate modes
  void initializePins(){
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(ENCA, INPUT); 
    pinMode(ENCB, INPUT); 
  }

// Method that turns motors on/off and sets PWM value
  void setMotor(int dir, int pwmVal){

    // Set PWM value
    analogWrite(enA,pwmVal);
    
    // Clockwise
    if(dir == 1){
      digitalWrite(in1,HIGH);
      digitalWrite(in2,LOW);
    }
    
    //Counter clockwise
    else if(dir == -1){
      digitalWrite(in1,LOW);
      digitalWrite(in2,HIGH);
    }
    
    // Motors off
    else{
      digitalWrite(in1,LOW);
      digitalWrite(in2,LOW);
    }  
  }
  
};

// Creating motor objects
// Input values: pwm, m1, m2, enca, encb, id
  Motor motor1(11,35,34,18,38,1);
  Motor motor2(4,37,36,19,39,2);
  Motor motor3(13,23,22,3,44,3);
  Motor motor4(12,24,25,2,21,4);
  Motor motor5(5,31,30,20,41,5);

// Global current motor variable. Points to the motor that is currently being used
  Motor* currentMotor = &motor1;

// Function that reads the encoder
// The voltage read from the encoder will be positive when moving in the forward direction
// and is 0 when moving in the opposite direction
void readEncoder(){ 
  
  int b = digitalRead(currentMotor->ENCB);    //Reads signal from the encoder 

  // If signal is greater than 0, increment the position of the encoder
  if  (b>0){ 
    currentMotor->pos++; 
  } 

  // If the signal <=0, decrement the position
  else { 
    currentMotor->pos--; 
  } 
} 


void setup() {
  
  // Begin serial communication
  Serial.begin(115200);

  // Call the function that initializes the pin modes
  motor1.initializePins();
  motor2.initializePins();
  motor3.initializePins();
  motor4.initializePins();
  motor5.initializePins();

  // Set channel A of encoder to listen for interupt signals
  attachInterrupt(digitalPinToInterrupt(motor1.ENCA),readEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(motor2.ENCA),readEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(motor3.ENCA),readEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(motor4.ENCA),readEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(motor5.ENCA),readEncoder, RISING);

  // Specify the resolution of the encoders for every motor
  motor1.ppr = 1320.0;
  motor2.ppr = 1320.0;
  motor3.ppr = 1850.0;
  motor4.ppr = 1850.0;
  motor5.ppr = 1850.0;

  // Specifiy the gear ratios for every motor
  motor1.convert = 13.33;
  motor2.convert = 10.66;
  motor3.convert = 13.33;
  motor4.convert = 13.33;
  motor5.convert = 90.00;

  // Specifiy the error for every motor
  // Values are in terms of degrees of joint rotation
  motor1.error = 0.0;
  motor2.error = 0.0;
  motor3.error = 0.0;
  motor4.error = 0.0;
  motor5.error = 0.0;

  // Specify the amount each joint should rotate when the pulse function is called
  // Values are in terms of degrees of joint rotation
  motor1.increment = (2.5 + motor1.error)/motor1.convert;
  motor2.increment = (2.5 + motor2.error)/motor2.convert;
  motor3.increment = (2.5 + motor3.error)/motor3.convert;
  motor4.increment = (2.5 + motor4.error)/motor4.convert;
  motor5.increment = (10.0 + motor5.error)/motor5.convert;
}

// Function that halts rotation of all motors
void killMotors(){
      currentMotor = &motor1;
      currentMotor->dir = 0;
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
      currentMotor = &motor2;
      currentMotor->dir = 0;
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
      currentMotor = &motor3;
      currentMotor->dir = 0;
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
      currentMotor = &motor4;
      currentMotor->dir = 0;
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
}

void loop() {

  // Calls the function that sets the speed and direction of the current motor
  currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
  
  // Print the encoder position value, angle of the motor, and angle of the joint rotation
  Serial.print("Motor "); Serial.print(currentMotor->ID); Serial.print(" position: ");
  Serial.print(currentMotor->pos); Serial.print("      motor angle: "); 
  Serial.print(currentMotor->pos /currentMotor->ppr * 360);                               // Converts encoder position to degrees
  Serial.print("      joint angle: "); 
  Serial.print(currentMotor->pos /currentMotor->ppr * currentMotor->convert);             // Applies gear ratio to rotation
  Serial.println("");
  
  //start reading keyboard input
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // prints the received data
    Serial.print("I received: ");  
    input = (char)incomingByte;
    Serial.println(input);


   // Controls for switching between motors
  
  if (input == '1'){
    currentMotor = &motor1;
  }

  if (input == '2'){
    currentMotor = &motor2;      
  }
  
  if (input == '3'){
    currentMotor = &motor3;
  }

  if (input == '4'){
    currentMotor = &motor4;
  }
    
  if (input == '5'){
   currentMotor = &motor5;
  }

   // Command to give current motor one full rotation 
   if (input == 'p'){
      currentMotor->p1 = currentMotor->pos;                       // Store current position
      currentMotor->p2 = currentMotor->p1+(currentMotor->ppr);    // Add resolution (pulses per revolution) of motor to current position 
      currentMotor->dir = 1;                                      // Set motor direction

      // While current position is less the position it would be in given one full rotation
      while (currentMotor->pos < currentMotor->p2){ 
        
        // Print the encoder position value, angle of the motor, and angle of the joint rotation
        Serial.print("Motor "); Serial.print(currentMotor->ID); Serial.print(" position: ");
        Serial.print(currentMotor->pos); Serial.print("      motor angle: "); Serial.print(currentMotor->pos /currentMotor->ppr * 360);
        Serial.print("      joint angle: "); Serial.print(currentMotor->pos /currentMotor->ppr * currentMotor->convert);
        Serial.println("");

        // Motor is on while this condition is ttue
        currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);     
      }
          // Motor is off is condition is false
          currentMotor->dir = 0;
          currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
      }

    // Keyboard input to turn off all motors
    if (input == 'w'){
      killMotors();
    }

    //Zeros encoder of current motor
    if (input == '0'){
    currentMotor->pos = 0;
    }
    
     // Continuous rotation controlls

     // Command to halt rotation
     if (input == 'q'){
      currentMotor->dir = 0;
    }

    // Command to rotate clockwise
    if (input == 'a'){
      currentMotor->dir = 1;
    }

    // Command to rotate counter clockwise
    if (input == 'z'){
      currentMotor->dir = -1;
    }

    // Pulse rotation controls

    // Command to pulse clockwise
    if (input == 'd'){
      
      currentMotor->p1 = currentMotor->pos;                                                      // Store current position
      currentMotor->p2 = currentMotor->p1 + ((currentMotor->increment) * currentMotor->ppr);     // Desired position after 1 pulse
      currentMotor->dir = 1;                                                                      

      // While current position is less than desired position
      while (currentMotor->pos < currentMotor->p2){
        
        // Print the encoder position value, angle of the motor, and angle of the joint rotation
        Serial.print("Motor "); Serial.print(currentMotor->ID); Serial.print(" position: ");
        Serial.print(currentMotor->pos); Serial.print("      motor angle: "); 
        Serial.print(currentMotor->pos /currentMotor->ppr * 360);
        Serial.print("      joint angle: "); 
        Serial.print(currentMotor->pos /currentMotor->ppr * currentMotor->convert);
        Serial.println("");

        // Motor is on while this condition is true
        currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
      }
        // Motor is off while this condition is false
        currentMotor->dir = 0;
        currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
    }

   // Command to pulse counter clockwise
   if (input == 'c'){
    
      currentMotor->p1 = currentMotor->pos;                                                       // Stores current position
      currentMotor->p2 = currentMotor->p1 - ((currentMotor->increment) * currentMotor->ppr);      // Desired position after 1 pulse
      currentMotor->dir = -1;

      // While current position is greater than desired position
      while (currentMotor->pos > currentMotor->p2){
               
        // Print the encoder position value, angle of the motor, and angle of the joint rotation
        Serial.print("Motor "); Serial.print(currentMotor->ID); Serial.print(" position: ");
        Serial.print(currentMotor->pos); Serial.print("      motor angle: "); 
        Serial.print(currentMotor->pos /currentMotor->ppr * 360);
        Serial.print("      joint angle: "); 
        Serial.print(currentMotor->pos /currentMotor->ppr * currentMotor->convert);
        Serial.println("");

        // Motor is on while this condition is true
        currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
      }
        // Motor is off while this condition is false
        currentMotor->dir = 0;
        currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
    }
    
    // Commands to turn motor 3 and motor 4 in the same direction

    // Stops rotation of both motors
    if (input == 'y'){
      currentMotor = &motor3;
      currentMotor->dir = 0;
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
      currentMotor = &motor4;
      currentMotor->dir = 0;
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
    }

    // Both motors turn clockwise
    if (input == 'h'){
      currentMotor = &motor3;
      currentMotor->dir = -1;
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
      currentMotor = &motor4;
      currentMotor->dir = 1;   
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal); 
      }

    // Both motors turn counter clockwise
    if (input == 'n'){
      currentMotor = &motor3;
      currentMotor->dir = 1;
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
      currentMotor = &motor4;
      currentMotor->dir = -1;
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
    }
    
    // Command to motor 3 and motor 4 opposite direction

    // Stops roation of both motors
    if (input == 'u'){
      currentMotor = &motor3;
      currentMotor->dir = 0;
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
      currentMotor = &motor4;
      currentMotor->dir = 0;
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
    }

    // Spin one way
    if (input == 'j'){
      currentMotor = &motor3;
      currentMotor->dir = 1;
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
      currentMotor = &motor4;
      currentMotor->dir = 1;   
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal); 
      }

    // Spin the other way
    if (input == 'm'){
      currentMotor = &motor3;
      currentMotor->dir = -1;
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
      currentMotor = &motor4;
      currentMotor->dir = -1;
      currentMotor->setMotor(currentMotor->dir, currentMotor->pwnVal);
    }
  }
}
