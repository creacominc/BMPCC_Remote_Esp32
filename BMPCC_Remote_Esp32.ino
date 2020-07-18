#include "BMPCC_Remote.h"

/**
 * This code is based on examples and data from:
 * - Sparkfun          - https://github.com/sparkfun/ESP32_Thing
 * - Arduino           - https://www.arduino.cc/en/Reference/ArduinoBLE
 * - Neil Kolban       - https://github.com/nkolban/esp32-snippets/blob/master/Documentation/BLE%20C%2B%2B%20Guide.pdf
 * - espressif         - https://github.com/espressif/arduino-esp32
 *                     - https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE/src
 *                     - https://github.com/espressif/arduino-esp32/blob/master/libraries/BLE/examples/BLE_client/BLE_client.ino
 * - Blackmagic Design - https://www.blackmagicdesign.com/developer/product/camera, https://www.bhphotovideo.com/lit_files/452872.pdf
 */

const int BAUD_RATE = 115200;
const int LED_PIN = 5;
const int BUTTON_PIN = 16;

/**
 *
 */
BMPCC_Remote bmpccRemote(BUTTON_PIN, LED_PIN);

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

    bmpccRemote.setup();
}

void loop()
{
    bmpccRemote.control();
    delay(100);
}
