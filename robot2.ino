#include <Servo.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servo;
Servo servo1;



#define L_S A0 //ir sensor Left
#define R_S A1 //ir sensor Right
int MotorRight_IN1=2;
int MotorRight_IN2=3;
int MotorLeft_IN3=8;
int MotorLeft_IN4=9;
int MotorRight_PWM=5;
int MotorLeft_PWM=6;
int max_speed=240;   // pin number 9 & 10 doesn't work as PWM.
int trigPin=7;
int echoPin=12;
int mode;
int state;
int val;


void setup()
{
  lcd.backlight();
  lcd.init();
  servo.attach(11);
  servo1.attach(10);
  servo.write(90);
  servo1.write(0);
  pinMode(MotorRight_IN1,OUTPUT);
  pinMode(MotorRight_IN2,OUTPUT);
  pinMode(MotorLeft_IN3,OUTPUT);
  pinMode(MotorLeft_IN4,OUTPUT);
  pinMode(MotorRight_PWM,OUTPUT);
  pinMode(MotorLeft_PWM,OUTPUT);
  pinMode(trigPin, OUTPUT);      
  pinMode(echoPin, INPUT);
  pinMode(R_S, INPUT);  
  pinMode(L_S, INPUT);
  Serial.begin(9600);
  
}

void loop() {

  if(Serial.available()!=0)
  {
    state=Serial.read(); 
  }

  if(state==8){mode=0;}
  else if(state==9){mode=1;}
  else if(state==10){mode=2;}
  else if(state==11){mode=3;}

  lcd.setCursor(0,0);
  lcd.print("mode:");
 if(mode==2){
  lcd.print("Obstacle");
  float distance=0.00;
  float RobotSpeed=0.00;
  float RightDistance=0.00;
  float LeftDistance=0.00;
 
  distance = search();
  Serial.println(distance);
  if(distance<=30)
  {
    analogWrite(MotorRight_PWM, 200); 
    analogWrite(MotorLeft_PWM, 200);
    stop();
    delay(100);
    servo.write(5);
    delay(400);
    RightDistance = search();
    delay(200);
    servo.write(180);
    delay(400);
    LeftDistance = search();
    delay(200);
    servo.write(90);
    delay(400);
    if(LeftDistance > RightDistance){
      goBackward();
      delay(200);
      stop();
      delay(200);
      goLeft();
      delay(1000);
      stop();
      delay(200);
    }
    else{
      goBackward();
      delay(200);
      stop();
      delay(200);
      goRight();
      delay(1000);
      stop();
      delay(200);
    }

  }
 else{
    analogWrite(MotorRight_PWM, max_speed); 
    analogWrite(MotorLeft_PWM, max_speed);
    goForward();

  }
 }  

  if(mode==0){

  lcd.print("Manual");
  analogWrite(MotorRight_PWM,max_speed);
  analogWrite(MotorLeft_PWM,max_speed);
  if(state==1)
  {
    goForward();
  }

  else if(state==2)
  {
    goBackward();
  }

  else if(state==3)
  {
    goLeft();
  }

  else if(state==4)
  {
    goRight();
  }

  else if(state==5)
  {
    stop();
  }
 } 

 if(mode==1){
  lcd.print("  Line ");
  lcd.setCursor(0,1);
  lcd.print("    Follower");
  float distance=0.00;
  float RobotSpeed=0.00;
  float RightDistance=0.00;
  float LeftDistance=0.00;
  //distance = search();
  //Serial.println(distance);
  analogWrite(MotorRight_PWM, 110); 
  analogWrite(MotorLeft_PWM, 110);
  if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){

    goForward();
 }
 
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){goRight();}  
else if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){goLeft();} 
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1)){stop();}
 }

 if(mode==3){
  val = digitalRead(A3);
  lcd.print("Smart Bin");
  lcd.setCursor(0,1);
  lcd.print("status:");
  float distance_bin=0.00;
  distance_bin = search();
  Serial.println(distance_bin);

    if(distance_bin<15 && val==1)
  {
  //    if (val==1)
  // {

  //   lcd.print("Not full");
  // }
  // else if (val==0) // always detected black things
  // {
  //   lcd.print("Full");
  //   }
    lcd.print("Not full");
     servo1.write(180);  
    delay(4000); 
  }
  else if(val==0){
          lcd.print("Full");
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Bin doesn't open.");
          lcd.setCursor(0,1);
          lcd.print("Please clean it.");
          delay(500);
         servo1.write(0);
         delay(50); 
  }
    else if(distance_bin>=15 && val==1){
  //     if (val==1)
  // {

  //   lcd.print("Not full");
  // }
  // else if (val==0) // always detected black things
  // {
  //   lcd.print("Full");
  //   } 
        lcd.print("Not full");
         servo1.write(0);
         delay(50);
           }
 }
  
 //delay(10);
   delay(70);
  lcd.clear();
}
  


float search(void)
{
  float duration=0.00;
  float space=0.00;
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);

  //Calculation
  duration = pulseIn(echoPin,HIGH);
  space = duration * 0.034 / 2; 
  return space;
}

void goForward(){
  digitalWrite(MotorRight_IN1,HIGH);
  digitalWrite(MotorRight_IN2,LOW);
  digitalWrite(MotorLeft_IN3,HIGH);
  digitalWrite(MotorLeft_IN4,LOW);
}

void goBackward(){
  digitalWrite(MotorRight_IN1,LOW);
  digitalWrite(MotorRight_IN2,HIGH);
  digitalWrite(MotorLeft_IN3,LOW);
  digitalWrite(MotorLeft_IN4,HIGH);
}

void stop(){
  digitalWrite(MotorRight_IN1,LOW);
  digitalWrite(MotorRight_IN2,LOW);
  digitalWrite(MotorLeft_IN3,LOW);
  digitalWrite(MotorLeft_IN4,LOW);
}

void goLeft(){
  digitalWrite(MotorRight_IN1,HIGH);
  digitalWrite(MotorRight_IN2,LOW);
  digitalWrite(MotorLeft_IN3,LOW);
  digitalWrite(MotorLeft_IN4,LOW);
}

void goRight(){
  digitalWrite(MotorRight_IN1,LOW);
  digitalWrite(MotorRight_IN2,LOW);
  digitalWrite(MotorLeft_IN3,HIGH);
  digitalWrite(MotorLeft_IN4,LOW);
}

    
    
  



