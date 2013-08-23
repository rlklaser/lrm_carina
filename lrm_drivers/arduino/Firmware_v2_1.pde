/***
 * FIRMWARE V2.0
 * 01/Mar/2011
 **/

#include <Servo.h> 
 
#define SERVO_PIN        9
#define YELLOWLED_PIN   13
#define MAX_ANGLE      110   // Define the MAXIMUM SERVO ANGLE

#define CMD_SETVEL      0xF5
#define CMD_STATUS      0xF6
#define CMD_SERVORESET  0xF7
#define CMD_RESET       0xF8
#define CMD_PING        0xF9
#define CMD_PONG        0xFA

Servo servoMotor;
int inByte = 0;
byte pinState = 0;
int statusLedsPins[] = { 2, 3, 4, 5, 6, 7, 8 };
int toggleCounter = 0;

/*//////////////////////////////////////////////////*/ 
void setup() {
  pinMode(SERVO_PIN,OUTPUT);
  pinMode(YELLOWLED_PIN, OUTPUT);
  for (int thisPin = 0; thisPin < 7; thisPin++)  {
    pinMode( statusLedsPins[thisPin], OUTPUT );
    digitalWrite(statusLedsPins[thisPin], LOW);
  }
  establishContact();
}

/*//////////////////////////////////////////////////*/ 
void establishContact() {
  Serial.begin(9600);
  resetServo();
  digitalWrite(YELLOWLED_PIN, LOW);
  while (Serial.available() <= 0) {
    waitingConnection();    //Guess what!?
  }
}

/*//////////////////////////////////////////////////*/ 
void waitingConnection() {
  //Knight rider
  for (int thisPin = 0; thisPin < 7; thisPin++) { 
    digitalWrite(statusLedsPins[thisPin], HIGH);   
    delay(100);                  
    digitalWrite(statusLedsPins[thisPin], LOW);
  }
  for (int thisPin = 6; thisPin >= 0; thisPin--) { 
    digitalWrite(statusLedsPins[thisPin], HIGH);
    delay(100);
    digitalWrite(statusLedsPins[thisPin], LOW);
  }
}

/*//////////////////////////////////////////////////*/ 
void displayStatus(int value) {
  /*
  Serial.print("See the Christmans lights, Bob: ");
  Serial.println(value,BIN);
  */
  digitalWrite(statusLedsPins[0], value&B00000001);
  digitalWrite(statusLedsPins[1], value&B00000010);
  digitalWrite(statusLedsPins[2], value&B00000100);
  digitalWrite(statusLedsPins[3], value&B00001000);
  digitalWrite(statusLedsPins[4], value&B00010000);
  digitalWrite(statusLedsPins[5], value&B00100000);
  digitalWrite(statusLedsPins[6], value&B01000000);
}

/*//////////////////////////////////////////////////*/ 
void resetServo() {
  if (!servoMotor.attached()) {
    servoMotor.attach(SERVO_PIN);
    Serial.println("Servo attached");
  }
  servoMotor.write(0);
  Serial.print("Angle: ");
  Serial.println(servoMotor.read());
}

/*//////////////////////////////////////////////////*/
//Note: The counter was used for create a non-block rotine
void toggle(int pinNum) {
  if (toggleCounter++ >= 1500) {
    digitalWrite(pinNum, pinState);
    pinState = !pinState;
    toggleCounter = 0;
  }
}

/*//////////////////////////////////////////////////*/ 
void loop() 
{ 
  if (Serial.available() > 0) {

    do {
      inByte = Serial.read();
    } while (inByte == -1);           //Wait for a command CMD_XXX

    Serial.print("#Recv:");
    Serial.println(inByte);
    
    if (inByte == CMD_SETVEL) {       //Serial Input is TOKEN?
      Serial.print("$Ack:");
      Serial.println(inByte);

      do {
        inByte = Serial.read();
      } while (inByte == -1);         //Wait for Angle

      Serial.print("$Cmd:");
      Serial.println(inByte);

      if (inByte <= MAX_ANGLE) {      //Limite Maximo do Servo
        servoMotor.write(inByte);
        displayStatus(inByte);
      }
      else {
        Serial.print("$Err:Out of upper limit (");
        Serial.print(inByte);
        Serial.println(")");
      }
    }
    
    if (inByte == CMD_STATUS) {
      Serial.print("$Ack:");
      Serial.println(inByte);
      Serial.print("$Status:");
      Serial.println(servoMotor.read());
      displayStatus(servoMotor.read());
    }
    
    if (inByte == CMD_SERVORESET) {
      Serial.print("$Ack:");
      Serial.println(inByte);
      Serial.println("$Reset");
      Serial.flush();
      resetServo();
      displayStatus(servoMotor.read());
    }
    
    if (inByte == CMD_RESET) {
      Serial.print("$Ack:");
      Serial.println(inByte);
      Serial.println("$SerialReset");
      Serial.end();
      establishContact();
    }
    
    if (inByte == CMD_PING) {
      Serial.println(CMD_PONG);
    }
    
  }
  toggle(YELLOWLED_PIN);
}

