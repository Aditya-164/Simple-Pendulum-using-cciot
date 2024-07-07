// For Blynk
#define BLYNK_TEMPLATE_ID "TMPL3faq4VSqD"
#define BLYNK_TEMPLATE_NAME "SimplePendulum Dashboard"
#define BLYNK_AUTH_TOKEN "bL6gBSfiROqkWPF9r5C6jLnSOSR9XvyG"
#define BLYNK_PRINT Serial

// Required Header files
#include <ESP32Servo.h>
#include <ThingSpeak.h>
#include <TimeLib.h>
#include <math.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Wifi Authorization
const char ssid[] = "Lenovo2024";
const char pass[] = "";
const char auth[] = BLYNK_AUTH_TOKEN;

// For Thingspeak
const unsigned long myChannelNumber = 2533338;
const char* myWriteAPIKey = "MBYNHOQE1H8BOFYF";
WiFiClient client;

// Defining the Servo object
Servo Actuator;

// Defining some parameters
const float pi = M_PI; // pi value from math library
const int IR_PIN = 21; // Pin for IR Sensor used to calculate Time period  
const int SERVO_PIN = 15; // Pin for Servo motor used for actuation
const int STOPPING_IR_PIN = 5; // Pin for IR Sensor used to stop the bob at minimum length
const float MinLength = 10; // Minimum Length of Simple Pendulum
const float MaxLength = 40;  // Maximum Length of Simple Pendulum
const int motorpin1 = 27; // Input Pin1 for DC motor
const int motorpin2 = 26; // Input Pin2 for DC motor
const int Enablepin = 14; // Pin for Enabling the motion of DC motor also controlsits speed
const int freq = 30000; // Frequency of PWM wave
const int pwmChannel = 0;
const int resolution = 8;
const int MOVING_AVG_SIZE = 10; // Size of Window for Moving average
//Global Variables to update accordingly
float l = MinLength;
int newLength;
int Start = 0;
int Stop = 0;
int currentState = LOW;
int previousState = LOW;
unsigned long lastHighTime = 0;
unsigned long prevHighTime = 0;
int count = 0;
float currentPosition = MinLength;
int attenuationState = LOW;
bool experimentRunning = false;
int duration = 0;
float movingAvgBuffer[MOVING_AVG_SIZE];
int movingAvgIndex = 0;
float movingAvgSum = 9.8 * MOVING_AVG_SIZE;

// Receiving data formmBlynk and performing necessary actions accordingly
BLYNK_WRITE(V4)
{
  newLength = param.asInt(); // Get the value of Changed Length from BLYNK
}
BLYNK_WRITE(V0)
{
  Start = param.asInt(); // Start the experiment
  // Changes the length to input value and gives actuation to the Pendulum to initiate its motion and gives signal to IR to sense the oscillations of the bob
  if (Start == 1)
  {
    Serial.println(newLength);
    if (10 <= newLength && newLength <= 40)
    {
      Serial.print("Current Position: ");
      Serial.print(currentPosition);
      Serial.println(" cm");
      float distance = newLength - currentPosition;
      if (distance > 0)
      {
        Serial.print("Moving Forward by ");
        Serial.print(distance);
        Serial.println(" cm");
        digitalWrite(motorpin1, LOW);
        digitalWrite(motorpin2, HIGH);
        float forewardSpeed = 6.9; // Speed of motor with which it brings the bob down
        duration = (distance * 1000) / forewardSpeed;

        ledcSetup(pwmChannel, freq, resolution);
        ledcAttachPin(Enablepin, pwmChannel);
        ledcWrite(pwmChannel, 180);
        analogWrite(Enablepin, 180);
        delay(duration);
        currentPosition = newLength;
      }

      digitalWrite(motorpin1, LOW);
      digitalWrite(motorpin2, LOW);
      analogWrite(Enablepin, 0);
      Serial.print("Current Position: ");
      Serial.print(currentPosition);
      Serial.println(" cm");
      delay(duration);
      // initiating the Motion of the Pendulum adaptively for different Lengths so as to account for the ooposinf torque
      Actuator.attach(SERVO_PIN);
      if (MinLength <= l <= 15)
      {
        Actuator.write(90);
        delay(2500);
        Actuator.write(110);
        delay(1000);
        Actuator.write(0);
        delay(300);
      }
      else if (15 < l <= 35)
      {
        Actuator.write(100);
        delay(2500);
        Actuator.write(125);
        delay(1000);
        Actuator.write(0);
        delay(300);
      }
      else if (35 < l <= MaxLength)
      {
        if (MinLength <= l <= 15)
        {
          Actuator.write(100);
          delay(2500);
          Actuator.write(135);
          delay(1000);
          Actuator.write(0);
          delay(300);
        }
      }
      Actuator.detach();
      experimentRunning = true;
    }
    else
    {
      Serial.println("Invalid Input");
      Serial.println(newLength);
    }
  }
}
BLYNK_WRITE(V5)
{ 
  Stop = param.asInt(); // Stops the experiment
  // Bring back the bob to its minimum length and dampen its oscillations and signals IR to stop sensing oscillations of the bob
  Serial.println(Stop);
  if (Stop == 1)
  {
    Serial.print("Stop = ");
    Serial.println(Stop);
    // Bring back to minimum Length
    while (digitalRead(STOPPING_IR_PIN) != LOW) {
      digitalWrite(motorpin1, HIGH);
      digitalWrite(motorpin2, LOW);
      ledcSetup(pwmChannel, freq, resolution);
      ledcAttachPin(Enablepin, pwmChannel);
      ledcWrite(pwmChannel, 150);
      analogWrite(Enablepin, 180);
      delay(10);
    }
    digitalWrite(motorpin1, LOW);
    digitalWrite(motorpin2, LOW);
    analogWrite(Enablepin, 0);
    experimentRunning = false;
    // Damping the oscillations
    Actuator.attach(SERVO_PIN);
    for(int i = 0;i<=100;i+=5){
      Actuator.write(i);
      delay(10);
    }
    delay(5000);
    Actuator.write(0);
    delay(300);
    Actuator.detach();
    //Update the Lengths
    l = MinLength;
    currentPosition = MinLength;
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200); // For serial communication
  setTime(0, 0, 0, 8, 5, 2024); // For initiliazing pproper time
  // Initializing the functions of Sensor Pins
  pinMode(IR_PIN, INPUT); // For Time Period IR
  pinMode(STOPPING_IR_PIN, INPUT); // For Stopping IR
  // For DC motor
  pinMode(motorpin1, OUTPUT);
  pinMode(motorpin2, OUTPUT);
  pinMode(Enablepin, OUTPUT);
   // Check the initial satet of Time Period IR
  previousState = digitalRead(IR_PIN);
  // Connecting to Wifi
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  // Initiliaze communication with Blynk and Thingspeak
  Blynk.begin(auth, ssid, pass);
  ThingSpeak.begin(client);
  // For PWM wave and control of Motor Driver
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(Enablepin, pwmChannel);
}

void loop() {
  // put your main code here, to run repeatedly:
  Blynk.run(); // For communication with Blynk
  // Sensingof oscillations, Finding Time period and calculating the value of Acceleration due to Gravity.
  currentState = digitalRead(IR_PIN);
  if (currentState == LOW && previousState == HIGH && experimentRunning) // Detects after just coming to mean postion and Signals form the microcontrollar to sense the oscillations
  {
    count++;
    if (count % 2 == 0) // Bob passes through the mean position twice so one time period after every two counts
    {
      unsigned long currentTime = millis();
      float timeDifference = currentTime - lastHighTime;
      float t = timeDifference / 1000;
      lastHighTime = currentTime;
      float g_inst = (4 * pi * pi * l) /(100 *(t * t)); // Instantaneous value of Acceleration due to Gravity
      // Calculating and updating the moving average
      movingAvgSum -= movingAvgBuffer[movingAvgIndex];
      movingAvgBuffer[movingAvgIndex] = g_inst;
      movingAvgSum += g_inst;
      float g_avg = movingAvgSum / MOVING_AVG_SIZE; // Average value of Acceleration due to Gravity
      movingAvgIndex = (movingAvgIndex + 1) % MOVING_AVG_SIZE;
      // Sending data to thingspeak for data Analysis
      ThingSpeak.setField(1, t);
      ThingSpeak.setField(2, g_inst);
      ThingSpeak.setField(3, g_avg);
      ThingSpeak.setField(4, l);
      ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      // Sending Data to Blynk to display to the user
      Blynk.virtualWrite(V1, g_inst);
      Blynk.virtualWrite(V2, timeDifference);
      // For Printing on the serial monitor
      Serial.print("Time Difference = ");
      Serial.print(timeDifference);
      Serial.print(" ms;");
      Serial.print(" Instantaneous g = ");
      Serial.print(g_inst);
      Serial.println(" m/s^2;");
      Serial.print(" Moving average g = ");
      Serial.print(g_avg);
      Serial.println(" m/s^2;");
    }
    previousState = currentState;
  }
  previousState = currentState;
}