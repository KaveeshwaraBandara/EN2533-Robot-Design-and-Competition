//libries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


//Tasks
#define STATE_TASK1 1
#define STATE_TASK2 2
#define STATE_TASK3 3
#define STATE_TASK4 4
#define STATE_TASK5 5
#define STATE_TASK6 6
#define STATE_TASK7 7
#define STATE_TASK8 8

int currentState = STATE_TASK1;

//Pin definitions for the L298N Motor Driver
#define AIN1 8
#define BIN1 7
#define AIN2 9
#define BIN2 12
#define PWMA 10
#define PWMB 11

bool isBlackLine = 0;             //keep 1 in case of black line. In case of white line change this to 0
unsigned int lineThickness = 15;  //Enter line thickness in mm. Works best for thickness between 10 & 35
unsigned int numSensors = 7;      // Enter number of sensors as 5 or 7

//PID Values
int P, D, I, previousError, PIDvalue;
double error = 0.00;
int lsp, rsp;
int lfSpeed = 70;
int currentSpeed = 30;
int sensorWeight[10] = {5, 4, 2, 1, 0, 0, -1, -2, -4, -5};
int activeSensors;
float Kp = 0.007;
float Kd = 0.005;
float Ki = 0.00;

int onLine = 1;

//Define arrays for store IR Array readings
int minValues[10], maxValues[10], threshold[10], sensorValue[10], sensorArray[10];

// Define color sensor pins
#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define sensorOut 2

int redMin = 17; // Red minimum value
int redMax = 145; // Red maximum value
int greenMin = 18; // Green minimum value
int greenMax = 155; // Green maximum value
int blueMin = 17; // Blue minimum value
int blueMax = 140; // Blue maximum value
//-----------------------------------------

int redPW = 0;
int greenPW = 0;
int bluePW = 0;

int redValue;
int greenValue;
int blueValue;


//Define Array for store the barcode values(i dont know the exact numbers to add here so i allocate 10 elements check it before competion)
int barCode[20];
int currentSize = 0;
int vBoxPosition; 

//Box placed in task 2
int placed = 0;
const int pickBoxLed = 6;  //LED for task 2 virtual box pickup indication

// Constants for encoders
const int EN_LEFT_A_PIN = 2; 
const int EN_LEFT_B_PIN = 3; 
const int EN_RIGHT_A_PIN = 18; 
const int EN_RIGHT_B_PIN = 19; 
#define CPR 225          // Encoder Pulses Per Revolution
#define WHEEL_DIAMETER 0.064 // Wheel diameter in meters (example: 6.5 cm)

// Time tracking
unsigned long prevTime = 0;
unsigned long prevPIDTime = 0;

const float CIRCUMFERENCE = PI * WHEEL_DIAMETER; // Wheel circumference in meters
volatile int position_left = 0;                       // Encoder position (counts)
volatile int position_right = 0;  

// PID control parameters
float KpEncoder = 8.0;  // Proportional gain
float KiEncoder = 0.5;  // Integral gain
float KdEncoder = 4.0;  // Derivative gain

// Speed control parameters
float basePWM = 70;             // Base PWM value
float integral = 0.0;
float prevError = 0.0;
int baseleftPWM = basePWM;

// Variables for speed calculation
volatile int prev_position_left = 0;
volatile int prev_position_right = 0;
float speed_left = 0.0;
float speed_right = 0.0;

// Time tracking
//unsigned long prevTime = 0;

int linecolor = 0;
float length = 0.00;
int distance_right = 0;
int distance_left = 0;
int distance1 = 0;
int distance2 = 0;
int distance3 = 0;

//ultrasonic
const int trig_pin = 4;
const int echo_pin = 5;
float timing = 0.0;
float distance = 0.0;

// Interrupt Service Routine for Channel A
void ISR_LEFT_A() {
    bool current_left_A = digitalRead(EN_LEFT_A_PIN);
    bool current_left_B = digitalRead(EN_LEFT_B_PIN);
    if (current_left_A == current_left_B)
        position_left++;
    else
        position_left--;
}

void ISR_RIGHT_A() {
    bool current_RIGHT_A = digitalRead(EN_RIGHT_A_PIN);
    bool current_RIGHT_B = digitalRead(EN_RIGHT_B_PIN);
    if (current_RIGHT_A == current_RIGHT_B)
        position_right--;
    else
        position_right++;
}

//Display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


void setup() {
  Serial.begin(9600);

  pinMode(EN_LEFT_A_PIN, INPUT_PULLUP);
  pinMode(EN_LEFT_B_PIN, INPUT_PULLUP);
  pinMode(EN_RIGHT_A_PIN, INPUT_PULLUP);
  pinMode(EN_RIGHT_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EN_LEFT_A_PIN), ISR_LEFT_A, RISING);
  attachInterrupt(digitalPinToInterrupt(EN_RIGHT_A_PIN), ISR_RIGHT_A, RISING);


  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // Set S0 - S3 as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
  // Set Sensor output as input
  pinMode(sensorOut, INPUT);
  
  // Set Frequency scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);


  //LED out
  pinMode(13, OUTPUT);
  pinMode(pickBoxLed, OUTPUT);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.setTextSize(1);
  display.setTextColor(WHITE);

  
  calibrate();
}

void loop() {
  switch (currentState) {
    case STATE_TASK1:
      task1();
      break;

    case STATE_TASK2:
      task2();
      break;

    case STATE_TASK3:
      task3();
      break;
    
    case STATE_TASK4:
      task4();
      break;

    case STATE_TASK5:
      task5();
      break;

    case STATE_TASK6:
      task4();
      break;

    case STATE_TASK7:
      task7();
      break;

    case STATE_TASK8:
      task8();
      break;

    default:
      Serial.println("Unknown state!");
      break;
  }
}



// Task 1: Barcode Reading
void task1() {
  display.clearDisplay();
  display.setCursor(0, 10);
  display.println("Task1");
  display.display(); 
  motor1run(70);
  motor2run(70);
  
  
  readLine();
  int count = 0;
  for (int i = 0; i < 10; i++) {
    if (sensorArray[i]){                     //check this condition for if because i didn't remembere output in ir array high or low for white surface
      count++;
    }
  }
  if (count < 4){
    float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
    float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters

    float length = (distance_left + distance_right)/2;  //average value
    if (length>0.005){
    currentSize++;

    if (length > 0.045){
      barCode[currentSize-1] = 1; 
    }
    else{
      barCode[currentSize-1] = 0;
    }
    }
    position_left = 0;
    position_right = 0;
  }

  
  //Transition to Task 2 after barcode is read
  if (barcodeReadComplete()) {
    currentState = STATE_TASK2; 
  }
}

// Task 2: Maze navigation
void task2() {
  int colr;
  int wall = openWall();                            //first wall = 0 second wall = 1
  if(vBoxPosition == 0){                        //8,9 - right 0,1- left
    uTurn();

    while(true){
      readLine();
      if(sensorArray[0] == 1 && sensorArray[1] == 1){
        break;
      }
      linefollow();
    }

    motor2run(0);
    motor1run(0);

    pickBox();

    if(wall == 0){
      moveBack();
      moveForVB0();      
    
    // while(true){
    //   readLine();
    //   int colr = detectcolour();
    //   if(colr == 1 || colr == 2){
    //     break;
    //   }
    //   linefollow();
    // }
    // motor2run(0);
    // motor1run(0);

    // linecolor = colr;

    // placeBox();
    // placed = 1;
    }
    
    else if(wall == 1){
      moveBack();
      moveBack();
      
      moveForVB0();

    //   do{
    //   readLine();
    //   linefollow(); 
    //   int colr = detectcolour(); 
    // }while(colr == 1 || colr == 2);

    // motor2run(0);
    // motor1run(0);

    // placeBox();
    // placed = 1;
    }
  }

  //when vBox in pos 1 
  else if(vBoxPosition == 1){
    if(wall == 0){
      Turnright();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }
      
      motor2run(0);
      motor1run(0);

      pickBox();

      moveBackColour();

      placeBox();
      placed = 1;
    }
    else{
      uTurn();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnleft();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnleft();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      pickBox();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      placeBox();

      moveBackAbove();

      Turnleft();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      // Turnright();

      // while(true){
      //   readLine();
      //   if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
      //     break;
      //   }
      //   linefollow();
      // }

      // motor2run(0);
      // motor1run(0);

      pickBox();

      moveBackColour();

      placeBox();
      placed = 1;
    }
  }

  else if(vBoxPosition == 2){
    if(wall == 0){
      Turnright();
      
      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnleft();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      pickBox();

      moveBackAbove();

      placeBox();

      moveBackAbove();

      Turnleft();

      while(true){
        readLine();
        if(sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);
      
      pickBox();

      moveBackColour();

      placeBox();
      placed = 1;
    }

    else{
      Turnright();
      
      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnleft();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      pickBox();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      placeBox();
      
      moveBackAbove();
      Turnleft();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright(); 

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      } 

      motor2run(0);
      motor1run(0);

      pickBox();

      moveBackColour();

      placeBox();
      placed = 1;
    }
  }

  else if(vBoxPosition == 3){
    if(wall == 0){
      Turnright();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnleft();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      pickBox();

      moveBackAbove();
      moveBackAbove();
      
      placeBox();

      moveBackAbove();

      Turnleft();

      while(true){
        readLine();
        if(sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);
      
      pickBox();

      moveBackColour();

      placeBox();
      placed = 1;
    }
    else{
      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);
      
      pickBox();

      moveBackColour();

      placeBox();
      placed = 1;
    }

  }

  else if(vBoxPosition == 4){
    if(wall == 0){
      Turnright();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnleft();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      pickBox();

      moveBackAbove();
      moveBackAbove();
      moveBackAbove();

      placeBox();

      moveBackAbove();

      Turnleft();

      while(true){
        readLine();
        if(sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);
      
      pickBox();

      moveBackColour();

      placeBox();
      placed = 1;
    }
    else{
      Turnright();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnleft();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      pickBox();
      moveBackAbove();
      
      placeBox();

      moveBackAbove(); 
      
      Turnleft();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright();

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      }

      motor2run(0);
      motor1run(0);

      Turnright(); 

      while(true){
        readLine();
        if(sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[8] == 1 && sensorArray[9] == 1){
          break;
        }
        linefollow();
      } 

      motor2run(0);
      motor1run(0);

      pickBox();

      moveBackColour();

      placeBox();
      placed = 1;
    }
  }
  if(placed){
    currentState = STATE_TASK3;
  }  
}

// Task 3: colour line following
void task3() {
  while(true){
    int count;
    count = sensorCount();
    if(count<5){
      break;
    }
    synchronizeMotorSpeeds(1);
  }
  while(true){
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine == 1) {  //PID LINE FOLLOW
      linefollow();
      digitalWrite(13, HIGH);
    } 
    else {
      digitalWrite(13, LOW);
      if (error > 0) {
        motor1run(-100);
        motor2run(lfSpeed);
      } 
      else {
        motor1run(lfSpeed);
        motor2run(-100);
      }
    }
  if(colourLineDone()){
    motor2run(0);
    motor1run(0);
    currentState = STATE_TASK4;
    break;
  }
  } 
}

// Task 4: Dotted line following
void task4() {
  int count;
 //if all black we need to move foward until we found white dashed line  
  while(true){
    count = sensorCount();
    if(count==0){
      synchronizeMotorSpeeds(1);
    }
    else{
      linefollow();
    }

  //condition for state transition
  if(count>7){
    motor2run(0);
    motor1run(0);
    currentState = STATE_TASK5;
    break;
  }
  }
}

// Task 5: Portal Navigation
void task5(){
  if(gateDetected()){
    do{
      motor2run(0);
      motor1run(0);
    }while(gateDetected());
    do{
      synchronizeMotorSpeeds(1);
    }while(sensorArray[0] == 1 && sensorArray[1] == 1||sensorArray[8] == 1 && sensorArray[9] == 1);
    currentState = STATE_TASK6;
    motor2run(0);
    motor1run(0);
  }
  else{
    do{
      motor2run(0);
      motor1run(0);
    }while(!gateDetected());
    do{
      motor2run(0);
      motor1run(0);
    }while(gateDetected());

    do{
      synchronizeMotorSpeeds(1);
    }while(sensorArray[0] == 1 && sensorArray[1] == 1||sensorArray[8] == 1 && sensorArray[9] == 1);
    currentState = STATE_TASK6;
    
    motor2run(0);
    motor1run(0);
  }
}

void task6() {
  
  if (linecolor == 1){ //1 is blue
    do{
      inversereadline();
      linefollow();
    }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
    Turnleft();

// 1st box

    do{
      inversereadline();
      linefollow();
      distance1 = Tof1read(); //define
    }while(distance1 <= 5);
    Grabbox();
    distance2 = Tof2read();
    distance3 = Tof3read();
    uTurn();
    do{
      inversereadline();
      linefollow();
    }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
    //largest box
    if(distance2 <= 10 && distance3 <= 10){
      Turnleft();
      distance_left = 0;
      distance_right = 0;
      do{
        inversereadline();
        linefollow();
        float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
        float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters
        float length = (distance_left + distance_right)/2;  //average value
      }while(sensorArray[0] == 0 && sensorArray[9] == 0 && length > 35); //zero is black
      Turnright();
      Placerealbox();
      Turnright();
      do{
      inversereadline();
      linefollow();
      }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
      Turnright();
    }
    //medium box
    if(distance2 <= 10 && distance3 > 10){
      Turnleft();
      do{
      inversereadline();
      linefollow();
      }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
      Turnright();
      Placerealbox();
      uTurn();
    }

    //small box
    if(distance2 > 10 && distance3 > 10){
      Placerealbox();
      Turnleft();
      do{
        inversereadline();
        linefollow();
      }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
      Turnleft();
    }

//2nd box

    do{
      inversereadline();
      linefollow();
      distance1 = Tof1read(); //define
    }while(distance1 <= 5);
    Grabbox();
    distance2 = Tof2read();
    distance3 = Tof3read();
    uTurn();
    do{
      inversereadline();
      linefollow();
    }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black

    //large box
    if(distance2 <= 10 && distance3 <= 10){
      Turnleft();
      do{
      inversereadline();
      linefollow();
      }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
      Turnright();
      Placerealbox();
      uTurn();
    }

    //medium box
    if(distance2 > 10 && distance3 > 10){
        Placerealbox();
        Turnleft();
        do{
          inversereadline();
          linefollow();
        }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
        Turnleft();
      }
    //small box
    if(distance2 > 10 && distance3 > 10){
        Turnright();
        do{
          inversereadline();
          linefollow();
          }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
        Turnleft();
        Placerealbox();
        Turnleft();
        distance_left = 0;
        distance_right = 0;
        do{
          inversereadline();
          linefollow();
          float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
          float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters
          float length = (distance_left + distance_right)/2;  //average value
        }while(sensorArray[0] == 0 && sensorArray[9] == 0 && length > 35); //zero is black
        Turnleft();
    }

// 3rd box

    do{
      inversereadline();
      linefollow();
      distance1 = Tof1read(); //define
    }while(distance1 <= 5);
    Grabbox();
    distance2 = Tof2read();
    distance3 = Tof3read();
    uTurn();
    do{
      inversereadline();
      linefollow();
    }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black

    //large box
    if(distance2 <= 10 && distance3 <= 10){
        Placerealbox();
        Turnleft();
        distance_left = 0;
        distance_right = 0;
      do{
        inversereadline();
        linefollow();
        float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
        float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters
        float length = (distance_left + distance_right)/2;  //average value
      }while(sensorArray[0] == 0 && sensorArray[9] == 0 && length > 15);
    }
    //medium box
    if(distance2 > 10 && distance3 > 10){
        Turnright();
        do{
          inversereadline();
          linefollow();
          }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
        Turnleft();
        Placerealbox();
        Turnleft();
        distance_left = 0;
        distance_right = 0;
        do{
          inversereadline();
          linefollow();
          float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
          float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters
          float length = (distance_left + distance_right)/2;  //average value
        }while(sensorArray[0] == 0 && sensorArray[9] == 0 && length > 35); //zero is black
    }
    //small box
    if(distance2 > 10 && distance3 > 10){
      Turnright();
      distance_left = 0;
      distance_right = 0;
      do{
        inversereadline();
        linefollow();
        float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
        float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters
        float length = (distance_left + distance_right)/2;  //average value
      }while(sensorArray[0] == 0 && sensorArray[9] == 0 && length > 35); //zero is black
      Turnleft();
      Placerealbox();
      Turnleft();
      distance_left = 0;
      distance_right = 0;
      do{
        inversereadline();
        linefollow();
        float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
        float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters
        float length = (distance_left + distance_right)/2;  //average value
      }while(sensorArray[0] == 0 && sensorArray[9] == 0 && length > 65);
    }
  }
  if (linecolor == 2){ //2 is red
    do{
      inversereadline();
      linefollow();
    }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
    Turnleft();

// 1st box

    do{
      inversereadline();
      linefollow();
      distance1 = Tof1read(); //define
    }while(distance1 <= 5);
    Grabbox();
    distance2 = Tof2read();
    distance3 = Tof3read();
    uTurn();
    do{
      inversereadline();
      linefollow();
    }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
    //small box
    if(distance2 > 10 && distance3 > 10){
      Turnleft();
      distance_left = 0;
      distance_right = 0;
      do{
        inversereadline();
        linefollow();
        float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
        float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters
        float length = (distance_left + distance_right)/2;  //average value
      }while(sensorArray[0] == 0 && sensorArray[9] == 0 && length > 35); //zero is black
      Turnright();
      Placerealbox();
      Turnright();
      do{
      inversereadline();
      linefollow();
      }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
      Turnright();
    }
    //medium box
    if(distance2 <= 10 && distance3 > 10){
      Turnleft();
      do{
      inversereadline();
      linefollow();
      }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
      Turnright();
      Placerealbox();
      uTurn();
    }

    //large box
    if(distance2 <= 10 && distance3 <= 10){
      Placerealbox();
      Turnleft();
      do{
        inversereadline();
        linefollow();
      }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
      Turnleft();
    }

//2nd box

    do{
      inversereadline();
      linefollow();
      distance1 = Tof1read(); //define
    }while(distance1 <= 5);
    Grabbox();
    distance2 = Tof2read();
    distance3 = Tof3read();
    uTurn();
    do{
      inversereadline();
      linefollow();
    }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black

    //small box
    if(distance2 > 10 && distance3 > 10){
      Turnleft();
      do{
      inversereadline();
      linefollow();
      }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
      Turnright();
      Placerealbox();
      uTurn();
    }

    //medium box
    if(distance2 > 10 && distance3 > 10){
        Placerealbox();
        Turnleft();
        do{
          inversereadline();
          linefollow();
        }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
        Turnleft();
      }
    //large box
    if(distance2 <= 10 && distance3 <= 10){
        Turnright();
        do{
          inversereadline();
          linefollow();
          }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
        }Turnleft();
        Placerealbox();
        Turnleft();
        distance_left = 0;
        distance_right = 0;
        do{
          inversereadline();
          linefollow();
          float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
          float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters
          float length = (distance_left + distance_right)/2;  //average value
        }while(sensorArray[0] == 0 && sensorArray[9] == 0 && length > 35); //zero is black
        Turnleft();
    }

// 3rd box

    do{
      inversereadline();
      linefollow();
      distance1 = Tof1read(); //define
    }while(distance1 <= 5);
    Grabbox();
    distance2 = Tof2read();
    distance3 = Tof3read();
    uTurn();
    do{
      inversereadline();
      linefollow();
    }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black

    //small box
    if(distance2 > 10 && distance3 > 10){
        Placerealbox();
        Turnleft();
        distance_left = 0;
        distance_right = 0;
      do{
        inversereadline();
        linefollow();
        float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
        float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters
        float length = (distance_left + distance_right)/2;  //average value
      }while(sensorArray[0] == 0 && sensorArray[9] == 0 && length > 15);
    }
    //medium box
    if(distance2 > 10 && distance3 > 10){
        Turnright();
        do{
          inversereadline();
          linefollow();
          }while(sensorArray[0] == 0 && sensorArray[9] == 0); //zero is black
        Turnleft();
        Placerealbox();
        Turnleft();
        distance_left = 0;
        distance_right = 0;
        do{
          inversereadline();
          linefollow();
          float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
          float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters
          float length = (distance_left + distance_right)/2;  //average value
        }while(sensorArray[0] == 0 && sensorArray[9] == 0 && length > 35); //zero is black
    }
    //small box
    if(distance2 <= 10 && distance3 <= 10){
      Turnright();
      distance_left = 0;
      distance_right = 0;
      do{
        inversereadline();
        linefollow();
        float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
        float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters
        float length = (distance_left + distance_right)/2;  //average value
      }while(sensorArray[0] == 0 && sensorArray[9] == 0 && length > 35); //zero is black
      Turnleft();
      Placerealbox();
      Turnleft();
      distance_left = 0;
      distance_right = 0;
      do{
        inversereadline();
        linefollow();
        float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
        float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters
        float length = (distance_left + distance_right)/2;  //average value
      }while(sensorArray[0] == 0 && sensorArray[9] == 0 && length > 65);
    }
}


void task7(){
  Turnleft();
  do{
    inversereadline();
    linefollow();
  }while(sensorArray[0] == 1 && sensorArray[1] == 1);

  motor2run(0);
  motor1run(0);

  Turnleft();
  //move until found the box and pick and move back until met the junction
  //......

  motor2run(0);
  motor1run(0);

  Turnright();

  //Hidden Task....................



}

void task8(){
}


// Helper Functions to Check Task Completion
bool barcodeReadComplete() {
  if (barCode[currentSize-1]==barCode[currentSize-2]&& barCode[currentSize-2] == barCode[currentSize-3]){                 //check the flag for end the Barcode
    vBoxPosition = binaryToDecimal(barCode[currentSize], currentSize-3) % 5;    //calculate the virtual box position for task 2 (check do i need to replace the value in array to 10)
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("Barcode =");
    display.println(vBoxPosition);
    display.println("END");
    display.display(); 
    return true;
  }
  else{
    return false;
  }  
}

//maze completion condition already i wrote within the loop
//task 4 completion condition already i wrote within the loop

bool colourLineDone() {
  int count = 0;
  for (int i = 0; i < 10; i++) {
    if (sensorArray[i]){                     //check this condition for if because i didnt remembere output in ir array high or low for white surface
      count++;
    }
  }
  if(count>6){
    return true;
  }
  else{
    return false;
  } 
}

bool someOtherTaskComplete() {
  
  return true; 
}

//calibrate the IR Array
void calibrate() {
  for (int i = 0; i < 10; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  for (int i = 0; i < 3000; i++) {
    motor1run(100);
    motor2run(-100);

    for (int i = 0; i < 10; i++) {
      if (analogRead(i) < minValues[i]) {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i]) {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for (int i = 0; i < 10; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motor1run(0);
  motor2run(0);
  delay(5000);
}

//Read the IR values in each and every loop
void readLine() {
  onLine = 0;
  if (numSensors == 10) {
    for (int i = 0; i < 10; i++) {
      if (isBlackLine) {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
      } else {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);
      }
      sensorValue[i] = constrain(sensorValue[i], 0, 1000);
      sensorArray[i] = sensorValue[i] > 500;
      if (sensorArray[i]) onLine = 1;

      if (isBlackLine == 1 && sensorArray[i]) onLine = 1;
      if (isBlackLine == 0 && !sensorValue[i]) onLine = 1;
    }
  }
}

//Line follow using PID 
void linefollow() {
  error = 0;
  activeSensors = 0;

  if (numSensors == 10) {
    for (int i = 0; i < 10; i++) {
      error += sensorWeight[i] * sensorArray[i];
      activeSensors += sensorArray[i];
    }
    error = error / activeSensors;
  }

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  motor1run(lsp);
  motor2run(rsp);
}

// Function to synchronize motor speeds with direction control(Chatgpt) 
void synchronizeMotorSpeeds(int forward) {        //foward - 1 backward - 0
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - prevPIDTime;

    if (deltaTime >= 100) { // Update every 100 ms
        prevPIDTime = currentTime;

        // Calculate speeds
        noInterrupts(); // Prevent ISR interference
        //int delta_left = position_left - prev_position_left;
        //int delta_right = position_right - prev_position_right;
        
        
        // Speed (in m/s)
        //speed_left = (delta_left / (float)CPR) * CIRCUMFERENCE / (deltaTime / 1000.0);
        //speed_right = (delta_right / (float)CPR) * CIRCUMFERENCE / (deltaTime / 1000.0);

        // PID calculation
        float error = position_right - position_left;
        integral += error * deltaTime / 1000.0;
        float derivative = (error - prevError) / (deltaTime / 1000.0);
        prevError = error;

        prev_position_left = position_left;
        prev_position_right = position_right;
        interrupts();


        float correction = (KpEncoder * error + KiEncoder * integral + KdEncoder * derivative)/100;
        baseleftPWM += (int)correction;
        // Calculate motor PWM values
        int left_pwm = constrain(baseleftPWM, 0, 255); // Base PWM for left motor
        int right_pwm = constrain(basePWM, 0, 255); // Adjusted PWM for right motor

        // Set motor directions
        if (forward == 1) {
            digitalWrite(AIN1, HIGH);  
            digitalWrite(BIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN2, LOW);
        } else {
            digitalWrite(AIN2, HIGH);
            digitalWrite(BIN2, HIGH);
            digitalWrite(AIN1, LOW);  
            digitalWrite(BIN1, LOW);
        }

        // Set motor speeds
        analogWrite(PWMA, left_pwm);
        analogWrite(PWMB, right_pwm);

        // Debugging output
        Serial.print("Speed Left: ");
        Serial.print(speed_left);
        Serial.print(" m/s, Speed Right: ");
        Serial.print(speed_right);
        Serial.print(" m/s, Error: ");
        Serial.println(error);
    }
}

//Function to run Motor 1
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

//Function to run Motor 2
void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}

//check this (Got from chatgpt)s
int binaryToDecimal(int binary[], int length) {
  int decimal = 0;
  for (int i = 0; i < length; i++) {
    decimal = decimal * 2 + binary[i];  // Left shift and add the current binary bit
  }
  return decimal;
}

//turn right using encoders
void Turnright(){
  position_left = 0;
  position_right = 0;  
  while (position_left < 325 || position_right < 15 ){
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("pos_left =");
    display.println(position_left);
    display.println("pos_right =");
    display.println(position_right);
    display.display(); 
    if (position_left < 325 && position_right < 15){
      motor2run(lfSpeed);
      motor1run(lfSpeed);
    }
    else if (position_left < 325 && position_right > 15){
      motor1run(lfSpeed);
      motor2run(0);      
    }
    else if (position_left > 325 && position_right < 15){
      motor1run(0);
      motor2run(lfSpeed);      
    }
    else{
      motor2run(0);
      motor1run(0);   
    }
  }
  motor2run(0);
  motor1run(0);   
}

//turn left using encoders
void Turnleft(){
  position_left = 0;
  position_right = 0;  
  while (position_left < 15 || position_right < 325 ){
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("pos_left =");
    display.println(position_left);
    display.println("pos_right =");
    display.println(position_right);
    display.display(); 
    if (position_left < 15 && position_right < 325){
      motor2run(lfSpeed);
      motor1run(lfSpeed);
    }
    else if (position_left > 15  && position_right < 325){
      motor1run(0);
      motor2run(lfSpeed);      
    }
    else if (position_left < 15 && position_right > 325){
      motor1run(lfSpeed);
      motor2run(0);      
    }
    else{
      motor2run(0);
      motor1run(0);   
    }
  }
  motor2run(0);
  motor1run(0); 
}

//identify junction left(1) or right(2) or straight(0)
int Junction(){
  readLine();
  if (sensorArray[0] == 0 && sensorArray[1] == 0){          //left
    return 1;
  }
  else if(sensorArray[8] == 0 && sensorArray[9] == 0){      //right
    return 2;                                                 
  }
  else{
    return 0;
  }
}

//detect the open wall firstone(0) or secondone(1)      8,9 - right  0,1 - left
int openWall(){
  //move to the maze from barcode 
  while(true){
    readLine();
    linefollow();
    if(sensorArray[8] == 1 && sensorArray[9] == 1){
      break;
    }
  }
    
  motor2run(0);
  motor1run(0);

  Turnright();

  while(true){
    readLine();
    if(sensorArray[8] == 1 && sensorArray[9] == 1){
      break;
    }
    linefollow();
  }

  motor2run(0);
  motor1run(0);
  
  //checking conditions for box position 
  if(vBoxPosition == 0){
    pickBox();
    while(true){
      readLine();
      if(sensorArray[8] == 1 && sensorArray[9] == 1){
        break;
      }
      linefollow();
    }
    
    motor2run(0);
    motor1run(0); 
    
    placeBox();
    uTurn();
    
    while(true){
      readLine();
      if(sensorArray[0] == 1 && sensorArray[1] == 1){
        break;
      }
      linefollow();
    }
    
    Turnleft();

    while(true){
      readLine();
      if(sensorArray[0] == 1 && sensorArray[1] == 1){
        break;
      }
      linefollow();
    }

    Turnleft();

    while(true){
      readLine();
      if(sensorArray[0] == 1 && sensorArray[1] == 1||sensorArray[8] == 1 && sensorArray[9] == 1){
        break;
      }
      linefollow();
    }

    motor2run(0);
    motor1run(0);

    if(detectWall()){
      return 0;
    }
    else{
      return 1;
    }

  }
  else{
    while(true){
      readLine();
      if(sensorArray[8] == 1 && sensorArray[9] == 1){
        break;
      }
      linefollow();
    }
  

    motor2run(0);
    motor1run(0);
    Turnright();
  //check if there is a need for move little bit above from junction.. here actually what we do is we avoid the first junction and move foward
    while(true){
      readLine();
      if(sensorArray[8] == 1 && sensorArray[9] == 1){
        break;
      }
      linefollow();
    }

    motor2run(0);
    motor1run(0);

    Turnright();

    while(true){
      readLine();
      if(sensorArray[0] == 1 && sensorArray[1] == 1||sensorArray[8] == 1 && sensorArray[9] == 1){
        break;
      }
      linefollow();
    }

    motor2run(0);
    motor1run(0);

    Turnleft();

    if(detectWall()){
      return 0;
    }
    else{
      return 1;
    }
  }
}

//pick Vbox
void pickBox(){
  digitalWrite(pickBoxLed, HIGH);  //turn on LED
}

//place Vbox
void placeBox(){
  digitalWrite(pickBoxLed, LOW); //turn off LED
}

//turn 180 degree
void uTurn(){

}

//detect the obstacle using ultrasonic return 1 when open and return 0 when close
int detectWall(){
  digitalWrite(trig_pin, LOW);
  delay(2);
  
  digitalWrite(trig_pin, HIGH);
  delay(10);
  digitalWrite(trig_pin, LOW);
  
  timing = pulseIn(echo_pin, HIGH);
  distance = (timing * 0.034) / 2;
  
  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.print("cm | ");
  // Serial.print(distance / 2.54);
  // Serial.println("in");
  
    
  if (distance <= 15){
  	return 0;
  } else {
  	return 1;
  }
}

//move backward until met a 4 way junction
void moveBack(){
  // do{
  //   readLine();
  //   synchronizeMotorSpeeds(0);
  // }while(sensorArray[0] == 1 && sensorArray[1] == 1||sensorArray[8] == 1 && sensorArray[9] == 1);

  while(true){
    readLine();
    synchronizeMotorSpeeds(0);
    if(sensorArray[0] == 1 && sensorArray[1] == 1||sensorArray[8] == 1 && sensorArray[9] == 1){
      break;
    }
  }
  
  motor2run(0);
  motor1run(0);  
}

//detect blue(1) or red(2) otherwise return 0 
int detectcolour(){
  // Read Red value
  redPW = getRedPW();
  redValue = map(redPW, redMin,redMax,255,0);

  // Read Blue value
  bluePW = getBluePW();
  blueValue = map(bluePW, blueMin,blueMax,255,0);

  if(1){          //find the range of blue and red values for colour sensor for red and blue seperately and usethem for if condition
    return 1;
  }
  else if(1){
    return 2;
  }
  else if(1){
    return 0;
  }
}

//move back with colour detection(we need to move back until colour sensor detect red or blue)
void moveBackColour(){
  int detected;
  // do{
  //   synchronizeMotorSpeeds(0);
  //   detected = detectcolour();
  // }while(detected==0);

  while(true){
    synchronizeMotorSpeeds(0);
    detected = detectcolour();

    if(detected==0){
      break;
    }
  }
  motor2run(0);
  motor1run(0); 
}

//this is for move back in maze above line(move back until met a junction)
void  moveBackAbove(){
  // do{
  //   readLine();
  //   synchronizeMotorSpeeds(0);
  // }while(sensorArray[0] == 1 && sensorArray[1] == 1);

  while(true){
    readLine();
    synchronizeMotorSpeeds(0);

    if(sensorArray[0] == 1 && sensorArray[1] == 1){
      break;
    }
  }
  
  motor2run(0);
  motor1run(0); 
}

// grabbing the box
void Grabbox(){

}

void Placerealbox(){
  distance_left = 0;
  distance_right = 0;
  do{
    inversereadline();
    linefollow();
    float distance_left = (position_left / CPR) * CIRCUMFERENCE; // Distance in meters
    float distance_right = (position_right / CPR) * CIRCUMFERENCE; // Distance in meters
    float length = (distance_left + distance_right)/2;  //average value
    }while(length > 30);
    Dropbox();
    movebackinverse();
    motor2run(0);
    motor1run(0);
    Turnleft();
}

// //linefollow in black line
// void inverselinefollow(){
// //online = 0
// }

//detect a black line
void inversereadline(){
//readline check the conditions and change to read black line for error correction
  onLine = 1;
  if (numSensors == 10) {
    for (int i = 0; i < 10; i++) {
      if (isBlackLine) {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
      } else {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);
      }
      sensorValue[i] = constrain(sensorValue[i], 0, 1000);
      sensorArray[i] = sensorValue[i] > 500;
      if (sensorArray[i]) onLine = 1;

      if (isBlackLine == 1 && sensorArray[i]) onLine = 1;
      if (isBlackLine == 0 && !sensorValue[i]) onLine = 1;
    }
  }
}

//move back in black line with syncrhonization
void movebackinverse(){
  do{
    inversereadline();
    synchronizeMotorSpeeds(0);
  }while(sensorArray[0] == 1 && sensorArray[1] == 1||sensorArray[8] == 1 && sensorArray[9] == 1);
  
  motor2run(0);
  motor1run(0);
}

void Dropbox(){
//drop the box for place the box using servo
}

int Tof1read(){     //this is for detect the gate  return value is distance

}

int Tof2read(){

}

int Tof3read(){

}

// Function to read Red Pulse Widths
int getRedPW() {
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  int PW;
  PW = pulseIn(sensorOut, LOW);
  return PW;
}

// Function to read blue Pulse Widths
int getBluePW() {
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  int PW;
  PW = pulseIn(sensorOut, LOW);
  return PW;
}

//detect gate closed or open (open retuns false.. close returns true)
bool gateDetected(){
  int distance;
    distance = Tof1read();
    if(distance < 0.25){      //assume that tof output is in meters(condition for distance less than 25cm)
      return true;
    }
    else{
      return false;
    }
}

//count the number of sensors detect white surface
int sensorCount(){
  readLine();
  int count = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorArray[i]){                     //check this condition for if because i didnt remembere output in ir array high or low for white surface
      count++;
    }
  }
  return count;
}

void moveForVB0(){
    int colr;
    placeBox();
    while(true){
      synchronizeMotorSpeeds(0);
      if(sensorArray[0]==1 && sensorArray[1]==1){
        break;
      }
    }

  motor2run(0);
  motor1run(0);

    Turnleft();

    while(true){
      readLine();
      linefollow();
      if(sensorArray[0]==1 && sensorArray[1]==1 && sensorArray[8]==1 && sensorArray[9]==1){
        break;
      }
    }

    motor2run(0);
    motor1run(0);
    
    Turnright();
    
    while(true){
      readLine();
      linefollow();
      if(sensorArray[8]==1 && sensorArray[9]==1){
        break;
      }
    }

    motor2run(0);
    motor1run(0);

    Turnright();

    while(true){
      readLine();
      linefollow();
      if(sensorArray[0]==1 && sensorArray[1]==1 && sensorArray[8]==1 && sensorArray[9]==1){
        break;
      }
    }

    pickBox();
    
    while(true){
      colr = detectcolour();
      readLine();
      linefollow();
      if(colr == 1 || colr == 2){
        break;
      }
    }
    placeBox();
    placed = 1;
}