#include <Arduino.h>
#include <UltrasonicSensor.h>
#include <ESP32Encoder.h>
#include <KnobManager.h>
#include <EEPROM.h>



// Delay definitions
#define BREATH_INTERVAL 8  // Interval in milliseconds between updates
#define MAX_BRIGHTNESS 200  // Maximum brightnesensorSerial value for analogWrite
#define BUTTON_LONG_PRESS 5000 // How long to press the button to reset saved values
#define RELAY_TOGGLE_DELAY 1000 // How long to wait before changing the relay state

#define SENSOR_READ_INTERVAL 500 //ms between reading the ultrasonic sensor

// EEPROM Address definitions
#define LOWER_LIMIT_ADDRESS 0
#define UPPER_LIMIT_ADDRESS 16
#define EEPROM_SIZE 64

// Pin definitions
#define POWER_LED_PIN 14
#define LOWER_LIMIT_LED_PIN 15
#define UPPER_LIMIT_LED_PIN 2
#define RELAY_LED_PIN 16
#define RELAY_PIN 12
#define SENSOR_RX 18
#define SENSOR_TX 19
#define ENCODER_PIN_A 23
#define ENCODER_PIN_B 22
#define ENCODER_BUTTON_PIN 21
    
unsigned long previoussensormillis = 0;
unsigned long previousbreathmillis = 0;
unsigned long previousrelaymillis = 0;
unsigned long previousdebugmillis = 0;


int value = 0;  // Current breathing LED brightness
int direction = 1;  // Direction of the brightnesensorSerial change (1 for increasing, -1 for decreasing)


UltrasonicSensor sensor(SENSOR_RX, SENSOR_TX);
unsigned int lowerLimit = 0; //Measured in distance from the sensor. This should be smaller than upperLimit
unsigned int upperLimit = 0; //Measured in distance from the sensor. This should be larger than lowerLimit
unsigned int distance;

Knob knobAdjuster(22,23,21);

// Define a factor to map the knob to the sensor reading
#define KNOB_TO_SENSOR_READING_FACTOR 20

// Function prototypes
void InitialSetup();
void blinkLEDs();
void breatheLED();
void resetEEPROM();

void setup(){
  Serial.begin(9600);
  //while (!Serial);
  delay(1000);

  //Initialize EEPROM
  if(!EEPROM.begin(EEPROM_SIZE)){
    Serial.println("Failed to initialise EEPROM");
    delay(1000);
    ESP.restart();    
  }

  pinMode(POWER_LED_PIN, OUTPUT);
  pinMode(LOWER_LIMIT_LED_PIN, OUTPUT);
  pinMode(UPPER_LIMIT_LED_PIN, OUTPUT);
  pinMode(RELAY_LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SENSOR_RX, OUTPUT);
  pinMode(SENSOR_TX, INPUT);

	ESP32Encoder::useInternalWeakPullResistors=UP;


  sensor.begin();

  // Check if there are values in the memory
  EEPROM.get(LOWER_LIMIT_ADDRESS, lowerLimit);
  EEPROM.get(UPPER_LIMIT_ADDRESS, upperLimit);
  Serial.print("Lower from EEPROM: ");
  Serial.println(lowerLimit);

  Serial.print("Upper from EEPROM: ");
  Serial.println(upperLimit);
  

  if (lowerLimit == 0 || upperLimit == 0)
  {
   InitialSetup();
  }

}

void loop()
{
  // Breath LED
  if (millis() - previousbreathmillis >= BREATH_INTERVAL)
  {
    breatheLED();
    previousbreathmillis = millis();
  }  

  // Read sensor
  if (millis()-previoussensormillis >= SENSOR_READ_INTERVAL)
  {
      distance = sensor.getDistance();
      if (distance > 0) { // Ensure we have a valid distance
          Serial.print("Distance: ");
          Serial.print(distance);
          Serial.println(" mm");
      }
      previoussensormillis = millis();
  }

  // Check the level and turn on/off the relay
  if (millis() - previousrelaymillis >= RELAY_TOGGLE_DELAY)
  {
    if (distance > lowerLimit)
    {
      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(RELAY_LED_PIN, HIGH);
      digitalWrite(LOWER_LIMIT_LED_PIN, HIGH);
      digitalWrite(UPPER_LIMIT_LED_PIN, LOW);
    }
    else if (distance < upperLimit)
    {
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(RELAY_LED_PIN, LOW);
      digitalWrite(LOWER_LIMIT_LED_PIN, LOW);
      digitalWrite(UPPER_LIMIT_LED_PIN, HIGH);
    }
    else
    {
      digitalWrite(LOWER_LIMIT_LED_PIN, LOW);
      digitalWrite(UPPER_LIMIT_LED_PIN, LOW);
    }

    previousrelaymillis = millis();
  }

  //Check the button status
  if (knobAdjuster.buttonPressed())
  {
    Serial.println("Button pressed");
    unsigned int buttonpressmillis = millis();
    bool resetvalues = false;
    while(knobAdjuster.buttonPressed())
    {
      if (millis() - BUTTON_LONG_PRESS >= buttonpressmillis)
      {
        resetvalues = true;
        blinkLEDs();
        delay(50);
      }
    }
    if (resetvalues){
      resetEEPROM();
      delay(1000);
      InitialSetup();
      resetvalues = false;
    }
  }


  if (millis() - 1000 > previousdebugmillis){
    int myint2;
    EEPROM.get(LOWER_LIMIT_ADDRESS, myint2);
    int myint3;
    EEPROM.get(UPPER_LIMIT_ADDRESS, myint3);
    
    Serial.print("Lower: ");
    Serial.println(myint2);
    Serial.print("Upper: ");
    Serial.println(myint3);

    Serial.println(knobAdjuster.getCount());
    previousdebugmillis = millis();
  }

  delay(5);
}

void InitialSetup()
{
    unsigned long previousloopmillis = millis();
    unsigned int distance;
    int encodervalue;
    Serial.println("Entering initialsetup");
    knobAdjuster.encoder.clearCount();

    //Set lower limit
    while(knobAdjuster.buttonPressed()==false)
    { //Continue as long as the button is unpressed
      if (millis() - 400 >= previousloopmillis){
        distance = sensor.getDistance();
        encodervalue = knobAdjuster.getCount();
         if (distance < encodervalue*KNOB_TO_SENSOR_READING_FACTOR)
        {
          digitalWrite(LOWER_LIMIT_LED_PIN, HIGH);
        }
        else {digitalWrite(LOWER_LIMIT_LED_PIN, LOW);}
        Serial.print("Setting lower value. distance: ");
        Serial.print(distance);
        Serial.print(" scaled encoder value: ");
        Serial.println(encodervalue*KNOB_TO_SENSOR_READING_FACTOR);

        previousloopmillis = millis();
      }
    }
    Serial.println("Lower limit set");

    delay(200);
    lowerLimit = encodervalue * KNOB_TO_SENSOR_READING_FACTOR;
    while(knobAdjuster.buttonPressed()==true){}; //Wait for the button release before continuing
    knobAdjuster.encoder.clearCount();

    //Set upper limit
    while(knobAdjuster.buttonPressed() == false){ //Continue as long as the button is unpressed
      if (millis() - 400 >= previousloopmillis){
        distance = sensor.getDistance();
        encodervalue = knobAdjuster.getCount();
        if (distance < encodervalue*KNOB_TO_SENSOR_READING_FACTOR)
        {
          digitalWrite(UPPER_LIMIT_LED_PIN, HIGH);
        }
        else {digitalWrite(UPPER_LIMIT_LED_PIN, LOW);}
        Serial.print("Setting upper value. distance: ");
        Serial.print(distance);
        Serial.print(" scaled encoder value: ");
        Serial.println(encodervalue*KNOB_TO_SENSOR_READING_FACTOR);

        previousloopmillis = millis();
      }    
    }
    while(knobAdjuster.buttonPressed()==true){}; //Wait for the button release before continuing
        
    upperLimit = encodervalue * KNOB_TO_SENSOR_READING_FACTOR;
    Serial.println("Upper limit set");

    // Save the new values to EEPROM
    EEPROM.put(LOWER_LIMIT_ADDRESS, lowerLimit);
    EEPROM.put(UPPER_LIMIT_ADDRESS, upperLimit);
    EEPROM.commit();

    Serial.println("Values saved to EEPROM");

    digitalWrite(POWER_LED_PIN, LOW);
    digitalWrite(LOWER_LIMIT_LED_PIN, LOW);
    digitalWrite(UPPER_LIMIT_LED_PIN, LOW);
    digitalWrite(RELAY_LED_PIN, LOW);
    Serial.println("LEDS OFF");

}

void blinkLEDs() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(POWER_LED_PIN, HIGH);
    digitalWrite(LOWER_LIMIT_LED_PIN, HIGH);
    digitalWrite(UPPER_LIMIT_LED_PIN, HIGH);
    digitalWrite(RELAY_LED_PIN, HIGH);
    Serial.println("LEDS ON");
    delay(200);
    digitalWrite(POWER_LED_PIN, LOW);
    digitalWrite(LOWER_LIMIT_LED_PIN, LOW);
    digitalWrite(UPPER_LIMIT_LED_PIN, LOW);
    digitalWrite(RELAY_LED_PIN, LOW);
    Serial.println("LEDS OFF");
    delay(200);
  }
}

void resetEEPROM() {
  EEPROM.write(LOWER_LIMIT_ADDRESS, 0);
  EEPROM.write(UPPER_LIMIT_ADDRESS, 0);
  EEPROM.commit();
  lowerLimit = 0;
  upperLimit = 0;
  Serial.println("EEPROM Reset");
}


void breatheLED()
{
        value += direction;
        // If we reach the maximum or minimum brightnesensorSerial, reverse the direction
        if (value >= MAX_BRIGHTNESS || value <= 0)
        {
            direction = -direction;
        }
        // Write the new brightnesensorSerial value to the LED
        analogWrite(POWER_LED_PIN, value);
}

