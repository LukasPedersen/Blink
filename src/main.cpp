/*
  Blink
  Turns an LED on for one second, then off for one second, repeatedly.
*/
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <SPI.h>

BLEService newService("180A"); // creating the service

BLEUnsignedCharCharacteristic randomReading("2A58", BLERead | BLENotify); // creating the Analog Value characteristic
BLEByteCharacteristic switchChar("2A57", BLERead | BLEWrite); // creating the LED characteristic

const int redLEDPin = 2;
const int greenLEDPin = 3;
long previousMillis = 0;

void setup()
{
  Serial.begin(9600);    // initialize serial communication
  while (!Serial);       //starts the program if we open the serial monitor.

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected
  pinMode(redLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  //initialize ArduinoBLE library
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");
    while (1);
  }

  BLE.setLocalName("MKR WiFi 1010-Lukas"); //Setting a name that will appear when scanning for Bluetooth® devices
  BLE.setAdvertisedService(newService);

  newService.addCharacteristic(switchChar); //add characteristics to a service
  newService.addCharacteristic(randomReading);

  BLE.addService(newService);  // adding the service

  switchChar.writeValue(0); //set initial value for characteristics
  randomReading.writeValue(0);

  BLE.advertise(); //start advertising the service
  Serial.println("Bluetooth® device active, waiting for connections...");
}

// the loop function runs over and over again forever
void loop()
{
  BLEDevice central = BLE.central(); // wait for a Bluetooth® Low Energy central
  bool redLED = false;
  bool greenLED = false;


  if (central) {  // if a central is connected to the peripheral
    Serial.print("Connected to central: ");

    Serial.println(central.address()); // print the central's BT address

    digitalWrite(LED_BUILTIN, HIGH); // turn on the LED to indicate the connection



    while (central.connected()) { // while the central is connected:
      long currentMillis = millis();

      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;

        int randomValue = analogRead(A1);
        randomReading.writeValue(randomValue);

        if (switchChar.written()) {
          Serial.println(switchChar.value());
          if (switchChar.value() == 0) // if (value == 0): Switch green LED
          {
            if (redLED)
            {
              redLED = !redLED;
              digitalWrite(redLEDPin, LOW);  
            }
            else
            {
              redLED = !redLED;
              digitalWrite(redLEDPin, HIGH);  
            }
            
          }
          else if (switchChar.value() == 1) // if (value == 1): Switch red LED
          {
            if (greenLED)
            {
              greenLED = !greenLED;
              digitalWrite(greenLEDPin, LOW);  
            }
            else
            {
              greenLED = !greenLED;
              digitalWrite(greenLEDPin, HIGH);  
            }
          }
          else if (switchChar.value() == 34) // if (value == 0): Switch both LED
          {
            if (greenLED)
            {
              greenLED = !greenLED;
              digitalWrite(greenLEDPin, LOW);  
            }
            else
            {
              greenLED = !greenLED;
              digitalWrite(greenLEDPin, HIGH);  
            }

            if (redLED)
            {
              redLED = !redLED;
              digitalWrite(redLEDPin, LOW);  
            }
            else
            {
              redLED = !redLED;
              digitalWrite(redLEDPin, HIGH);  
            }
          }
        }
      }
    }

    digitalWrite(LED_BUILTIN, LOW); // when the central disconnects, turn off the LED
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}