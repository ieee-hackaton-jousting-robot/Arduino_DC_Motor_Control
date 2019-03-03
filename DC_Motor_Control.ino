// Define the pin numbers
#include<PacketSerial.h>


PacketSerial myPacketSerial;
int motorPin1 = 6;
int motorPin2 = 7;
int motor2Pin1 = 3;
int motor2Pin2 = 4;
int pwmMotor1 = 5;
int pwmMotor2 = 9;
int8_t parameter1 = 0;
int8_t parameter2 = 0;
#define sRate 115200

/*
 * 
 * Remapping the dutycycle and stuff: we're taking some input parameter/number
 * this parameter could be from -127 to 128 or somethin: 8bit 10101010
 * 0 means stop
 * negative means make the other lead high - we could take the abs val which is nice
 * I think parameter also dictates which motor for which duty cycle
 * 
 * if parameter = negative
 *   
 * 
 * duty cycle could be from 0-255 but we may not even want to consider 0-75 because the motors don't even engage for those values
 * 
 * 
 */
void leftMotor(int input) {
  int dtyCycl = 0;
  if (input < 0) { //move backwards
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }
  else { //move forwards
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }

  // motor power here
  dtyCycl = abs(input*2);
  analogWrite(pwmMotor1, dtyCycl);
}

void rightMotor(int input) { 
  int dtyCycl = 0;
  if (input < 0) { //move backwards
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  } 
  else { //move forwards
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  }

  // motor power here
  dtyCycl = abs(input*2);
  analogWrite(pwmMotor2, dtyCycl);
}

void onPacketReceived(const uint8_t* buffer, size_t size){ //what do we want to do with packet info - simply just store it in 2 parameters
  //basically read the first and second values of the array
  if (size < 2){ //this is garbage, do nothing
    leftMotor(0);
    rightMotor(0);
    return;
  }  
  parameter1 = buffer[0];
  parameter2 = buffer[1];
  leftMotor(parameter1);
  rightMotor(parameter2);
}

void setup() {
    // Set the pin modes of the above IO pins to OUTPUT
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(pwmMotor1, OUTPUT);
    pinMode(pwmMotor2, OUTPUT);
    myPacketSerial.begin(sRate);
    myPacketSerial.setPacketHandler(&onPacketReceived);

}

void loop() {
//leftMotor(127);
//rightMotor(127); 
  
  //first we need to read the values from serial, take these values as parameters 1 and 2 and use it to control our robot
  myPacketSerial.update(); //regularly needs to do this
  
//    analogWrite(pwmMotor1, dtyCycl1);
//    analogWrite(pwmMotor2, dtyCycl2);
//
//
//  
//    // Turn the motor in one direction
//    digitalWrite(motor2Pin1, LOW); //fwrds for 1
//    digitalWrite(motor2Pin2, HIGH); //bwds for 2
//    delay(1000);
//    digitalWrite(motor2Pin1, HIGH);
//    digitalWrite(motor2Pin2, LOW);
//    delay(1000);
//    //digitalWrite(motor2Pin1, HIGH);
//    //digitalWrite(motor2Pin2, LOW);
//
//    
//    //Go Straight
//    while (parameter ){
//      digitalWrite(motorPin1, HIGH);
//      digitalWrite(motorPin2, LOW);
//      digitalWrite(motor2Pin1, LOW);
//      digitalWrite(motor2Pin2, HIGH);
//    }
//
//    //Go Backwards
//    while (){
//      digitalWrite(motorPin1, LOW);
//      digitalWrite(motorPin2, HIGH);
//      digitalWrite(motor2Pin1, HIGH);
//      digitalWrite(motor2Pin2, LOW);
//    }
//
//    //Turn Left, should be the same
//    while (){
//      digitalWrite(motorPin1, LOW);
//      digitalWrite(motorPin2, HIGH);
//      digitalWrite(motor2Pin1, LOW);
//      digitalWrite(motor2Pin2, HIGH);
//    }
//
//    //Turn Right
//    while (){
//      digitalWrite(motorPin1, HIGH);
//      digitalWrite(motorPin2, LOW);
//      digitalWrite(motor2Pin1, HIGH);
//      digitalWrite(motor2Pin2, LOW);
//    }
    
}
