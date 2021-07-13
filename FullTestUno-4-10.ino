/*

Serial Input Chars
a - Encoder
b - Pump Out
c - Hot Plate
d - Solenoid
e - Raw Boba Stepper
f - Mesh Stepper
g - RTD
h - Flow
i - Ultrasonic

Pinout
0 = TX
1 = RX
2 - Flow 
3 - Encoder 
4 - in2 (DC)
5 - in1 (DC)
6 - en (DC)
7 - en (wp step)
8 - dir (wp step)
9 - step/pul (wp step)
10 - solenoid
11 - hot plate
12 - RTD
13 - Pump out

A0 - en (raw step)
A1 - dir (raw step)
A2 - step (raw step)
A3 - ultrasonic
A4 = SDA
A5 = SCK

*/

#include <OneWire.h> 
#include <DallasTemperature.h>
#include <NewPing.h>

//Relay and Stepper Pin Delcarations
const int solenoid = 10;
const int hotPlate = 11;
const int pumpOut = 13;
const int mEnPin = 7;
const int mDirPin = 8;
const int mStepPin = 9;
const int rEnPin = A0;
const int rDirPin = A1;
const int rStepPin = A2;

//Encoder
#define ENC_COUNT_REV 374
#define encInt 3
#define encIn1 4
#define encIn2 5
#define encEn 6
volatile long encoderValue = 0;
volatile long encoderStore = 0;

//RTD
#define ONE_WIRE_BUS 12 
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
bool startTimer = false;
int boilTimer = 0;
float tempC;

//Ultrasonic
#define trigPin A3 // Trigger and Echo both on pin A3
#define echoPin A3
#define maxDistance 400
NewPing sonar(trigPin, echoPin, maxDistance);
float duration, distance;

//Flow Meter
int sensorInterrupt = 0;  // interrupt 0
int sensorPin = 2; //Digital Pin 2
unsigned int SetPoint = 400; //400 milileter
float calibrationFactor = 840; 
volatile byte pulseCount = 0; 
float flowRate = 0.0;
unsigned int flowMilliLitres = 0;
unsigned long totalMilliLitres = 0;
unsigned long oldTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //Relay pin mode and initial state
  pinMode(solenoid,OUTPUT);
  pinMode(hotPlate,OUTPUT);
  pinMode(pumpOut,OUTPUT);
  digitalWrite(solenoid,LOW);
  digitalWrite(hotPlate,LOW);
  digitalWrite(pumpOut,LOW);

  //Mesh stepper pin mode and initial state
  pinMode(mEnPin,OUTPUT);
  pinMode(mDirPin,OUTPUT);
  pinMode(mStepPin,OUTPUT);
  digitalWrite(mEnPin,HIGH);
  digitalWrite(mDirPin,LOW);
  digitalWrite(mStepPin,LOW);

  //Raw boba stepper pin mode and initial state
  pinMode(rEnPin,OUTPUT);
  pinMode(rDirPin,OUTPUT);
  pinMode(rStepPin,OUTPUT);
  digitalWrite(rEnPin,HIGH);
  digitalWrite(rDirPin,LOW);
  digitalWrite(rStepPin,LOW);

  //Encoder pin mode and initial state
  pinMode(encInt,INPUT_PULLUP);
  pinMode(encIn1, OUTPUT);
  pinMode(encIn2, OUTPUT);
  digitalWrite(encIn1,LOW);
  digitalWrite(encIn2,LOW);
  analogWrite(encEn,50);

  //RTD
  sensors.begin();

  //Flow Meter
  pinMode(sensorPin, INPUT_PULLUP);
    
  
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available()) {
    char token = Serial.read();
    if (token == "a") {
      Serial.println("Encoder_Motor_Test");
      encoderTest();
      Serial.println("Done");

    } else if(token == "b") {
      Serial.println("Pump_Out_Relay_Test");
      relayTest(1);
      Serial.println("Done");
         
    } else if(token == "c") {       
      Serial.println("Hot_Plate_Relay_Test");
      relayTest(2);
      Serial.println("Done");

    } else if(token == "d") {
      Serial.println("Solenoid_Relay_Test");
      relayTest(3);
      Serial.println("Done");

    } else if(token == "e") {
      Serial.println("Raw_Boba_Stepper_Test");
      rawStepper(3); //Input how many rotations you want
      Serial.println("Done");
      
    } else if(token == "f") {
      Serial.println("Mesh_Stepper_Test"); 
      meshStepper(); //Might need to flip DIR pin high and low in the function
      Serial.println("Done");
      
    } else if(token == "g") {
      Serial.println("RTD_Test");
      getTemps(10); //Input how many readings you want (2 sec delay between readings)
      Serial.println("Done");
      
    } else if(token == "h") {
      Serial.println("Flow_Meter_Test");
      flowTest(20); //Input how many readings you want (1 sec delay between readings)
      Serial.println("Done");
      
    } else if(token == "i") {
      Serial.println("Ultrasonic_Test");
      getDistance(20); //Input how many readings you want (0.5 sec delay between readings)
      Serial.println("Done");
      
    }
  }
}

void relayTest(int relayNum) {
  if (relayNum == 1) {
    digitalWrite(pumpOut,HIGH);
    delay(5000);
    digitalWrite(pumpOut,LOW);
    
  } else if (relayNum == 2) {
    digitalWrite(hotPlate,HIGH);
    delay(5000);
    digitalWrite(hotPlate,LOW);
    
  } else if (relayNum == 3) {
    digitalWrite(solenoid,HIGH);
    delay(5000);
    digitalWrite(solenoid,LOW);
    
  }
}

void meshStepper() {
  digitalWrite(mEnPin,LOW);
  digitalWrite(mDirPin,LOW); //Switch this
  // Makes 800 pulses for making one full cycle rotation
  for(int x = 0; x < 200; x++) {
    digitalWrite(mStepPin,HIGH);
    delayMicroseconds(5000);
    digitalWrite(mStepPin,LOW);
    delayMicroseconds(5000);
  }
  delay(2000);
  digitalWrite(mDirPin,HIGH); //Switch this
  // Makes 800 pulses for making one full cycle rotation
  for(int x = 0; x < 200; x++) {
    digitalWrite(mStepPin,HIGH);
    delayMicroseconds(5000);
    digitalWrite(mStepPin,LOW);
    delayMicroseconds(5000);
  }
  digitalWrite(mEnPin,HIGH);
}

void rawStepper(int rotations) {
  digitalWrite(rEnPin,LOW);
  digitalWrite(rDirPin,LOW);
  // Makes 800 pulses for making one full cycle rotation
  for(int x = 0; x < rotations*200; x++) {
    digitalWrite(rStepPin,HIGH);
    delayMicroseconds(5000);
    digitalWrite(rStepPin,LOW);
    delayMicroseconds(5000);
  }
  digitalWrite(rEnPin,HIGH);
}

void encoderTest() {
  attachInterrupt(digitalPinToInterrupt(encInt), updateEncoder, RISING);
  delay(500);
  if (encoderValue < 5) {
    digitalWrite(encIn1,LOW);
    digitalWrite(encIn2,HIGH);
  }
  if (encoderValue >1000) {
    digitalWrite(encIn1,LOW);
    digitalWrite(encIn2,LOW);
  }
  delay(500);
  encoderStore = encoderValue - encoderStore;
  encoderValue = 0;
  Serial.println(encoderStore);
  delay(2000);
  if (encoderValue < 5) {
    digitalWrite(encIn1,HIGH);
    digitalWrite(encIn2,LOW);
  }
  if (encoderValue > encoderStore) {
    digitalWrite(encIn1,LOW);
    digitalWrite(encIn2,LOW);
  }
  encoderStore = encoderValue-encoderStore;
  encoderValue = 0;
  
  detachInterrupt(digitalPinToInterrupt(encInt));
  
}

void updateEncoder()
{
  // Increment value for each pulse from encoder
  encoderValue++;
}

void getTemps(int iteration) {
  for(int y = 0; y < iteration; y++) {
    sensors.requestTemperatures();
    tempC = sensors.getTempCByIndex(0);
    Serial.print("The temperature is: ");
    Serial.print(tempC);
    Serial.println(" C");
    delay(2000);
  } 
}

void getDistance(int iteration) {
  for(int z = 0; z < iteration; z++) {
    distance = sonar.ping_cm();
    Serial.print("Distance = ");
    
    if (distance >= 400 || distance <= 2) 
    {
      Serial.println("Out of range");
    }
    else 
    {
      Serial.print(distance);
      Serial.println(" cm");
    }
    delay(500);
  }
}

void flowTest(int iteration) {
  oldTime = millis();
  attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
  while (millis()-oldTime < iteration*1000) {
    if((millis() - oldTime) > 1000) {
      detachInterrupt(sensorInterrupt);
      flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
      oldTime = millis();
      flowMilliLitres = (flowRate / 60) * 1000;
      totalMilliLitres += flowMilliLitres;
      unsigned int frac;
      
      Serial.print("Flow rate: ");
      Serial.print(flowMilliLitres, DEC);  // Print the integer part of the variable
      Serial.print("mL/Second");
      Serial.print("\t");
      Serial.print("Output Liquid Quantity: ");        
      Serial.print(totalMilliLitres,DEC);
      Serial.println("mL"); 
      Serial.print("\t"); 
  
      pulseCount = 0;   
      attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
    }
  }
}

void pulseCounter(){
  // Increment the pulse counter
  pulseCount++;
}
