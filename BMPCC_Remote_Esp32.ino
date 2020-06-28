
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <string>

/**
 * This code is based on examples from:
 * - Sparkfun      - https://github.com/sparkfun/ESP32_Thing
 * - espressif     - https://github.com/espressif/arduino-esp32
 *                 - https://github.com/espressif/arduino-esp32/blob/master/libraries/BLE/examples/BLE_client/BLE_client.ino
 * - MagicButton4k - https://www.instructables.com/id/Magic-Button-4k-the-20USD-BMPCC4k-Remote/
 */

const int BAUD_RATE  = 115200;
const int LED_PIN    = 5;
const int BUTTON_PIN = 0;

/**
 * setup the bluetooth client
 */
void setup()
{
  // set up and test serial to the Sparkfun ESP32 Thing
  Serial.begin(BAUD_RATE);
  // initialize the LED pin as an output:
  pinMode(LED_PIN, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(BUTTON_PIN, INPUT);


}

void loop()
{
  // First and simplest test is to flash the LED and respond to the button.
  static int buttonState = LOW;         // variable for reading the pushbutton status
  static int lightState = LOW;
  static bool switchState = true;
  // read the state of the pushbutton value:
  static buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is LOW:
  if( LOW == buttonState )
  {
    if( switchState )
    {
      lightState = (lightState == LOW ? HIGH : LOW);
      digitalWrite(ledPin, lightState);
      Serial.println("  Changed record state: ");
      Serial.println( lightState );
      switchState = false;
    }
  }
  else
  {
    switchState = true;
  }
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  delay( 100 );
}
