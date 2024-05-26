#include <Servo.h>
#include <Stepper.h>

const int stepsPerRevolution = 2048;
Stepper stepper5(stepsPerRevolution, 4, 8, 7, 11);

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo6;

int val1;
int val2;
int val3;
int val4;
int val6;


//stepper constants
int val5; //stepper pot
const int minPotValue = 0;    
const int maxPotValue = 1023; 
const int minAngle = 0;
const int maxAngle = 360; 
const int minPosition = 0;
const int maxPosition = stepsPerRevolution; 
int currentPosition = 0;

//cinematica inversa
#define PI 3.1415926535897932384626433832795

// float Xc = -30;
// float Yc = 20;
// float Zc = 10;

float L1 = 25;
float L2 = 20;
float L3 = 10;
float L4 = 15;

float theta1;
float theta2;
float theta3;
float theta4;

float r; 
float s;
float A;
float B;

void move_to_point(float Xc, float Yc, float Zc){


  theta1 = atan2(Yc,Xc);
  r = sqrt(Xc*Xc+Yc*Yc);
  s = Zc - L1;
  A = sqrt(s*s+sq(r-L4));

  theta3 = acos((A*A-L2*L2-L3*L3)/(2*L2*L3));
  theta3 = -theta3;

  B = L3*sin(theta3);

  theta2 = atan2(s, r-L4)-asin(B/A);

  theta4 = -(theta2+theta3);

  servo1.write(theta1*180/PI); 

  val2 = theta2*180/PI;            
  val2 = map(val2, -95, 85, 0, 180);     
  servo2.write(val2); 

  val3 = theta3*180/PI;            
  val3 = map(val3, 15, -165, 0, 180); 
  servo3.write(val3); 

  val4 = theta4*180/PI;         
  val4 = map(val4, -50, 105, 0, 180);   
  servo4.write(val4); 




}





void moveStepperToAngle(int angle) {
  int targetPosition = map(angle, minAngle, maxAngle, minPosition, maxPosition);
  int stepsToMove = targetPosition - currentPosition;
  stepper5.step(stepsToMove);
  currentPosition = targetPosition;
}

void setup() {

  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(9);
  servo6.attach(10);

  stepper5.setSpeed(5); 





  Serial.begin(9600);

}

void loop() {
  move_to_point(25, 25, 25);
  servo6.write(180);
  delay(1000);

//moving to 9,27,5
for (int i = 0; i<2000; i++){
  float t = (float)i/1000;
  float x = 25.0 - 12.0*t*t + 4.0*t*t*t;
  float y = 25.0 + 1.5*t*t - 0.5*t*t*t;
  float z = 25.0 - 15.0*t*t + 5.0*t*t*t;
  move_to_point(x, y, z);
  delay(1);
}


//moving to 13,33,5
for (int i = 0; i<1000; i++){
  float t = (float)i/1000;
  float x = 9.0 + 12.0*t*t - 8.0*t*t*t;
  float y = 27.0 + 18.0*t*t - 12.0*t*t*t;
  float z = 5.0;
  move_to_point(x, y, z);
  delay(1);
  // Serial.print("Y: ");
  // Serial.println(x);
}
  
servo6.write(100); 
delay(1000);

//moving to 13,33,10
for (int i = 0; i<1000; i++){
  float t = (float)i/1000;
  float x = 13.0;
  float y = 33.0;
  float z = 5.0 + 15*t*t - 10*t*t*t;
  move_to_point(x, y, z);
  delay(1);
  // Serial.print("Y: ");
  // Serial.println(x);
}

stepper5.step(1150);

//moving to -30,20,10
for (int i = 0; i<2000; i++){
  float t = (float)i/1000;
  float x = 13.0 - 32.25*t*t + 10.75*t*t*t;
  float y = 33.0 - 9.75*t*t + 3.25*t*t*t;
  float z = 10.0;
  move_to_point(x, y, z);
  delay(1);
  // Serial.print("Y: ");
  // Serial.println(x);
}


servo6.write(180);

while(1);



}



