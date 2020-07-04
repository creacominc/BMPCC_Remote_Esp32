#include "BMPCCSecurityCallback.h"

#include <HardwareSerial.h>

BMPCCSecurityCallback::BMPCCSecurityCallback()
{
}


BMPCCSecurityCallback::~BMPCCSecurityCallback()
{
}


uint32_t BMPCCSecurityCallback::onPassKeyRequest()
{
  // this probably only works on the Serial Monitor.
  uint32_t pin = 0;
  Serial.println("Enter pass key from device: ");
  while( ! Serial.available() )
    {
      delay(1);
    }
  if( Serial.available() > 0 )
    {
      pin = Serial.parseInt();
      Serial.print(" pin: ");
      Serial.println( pin );
    }
  Serial.print("Returning pass key: ");
  Serial.println( pin );
  return pin;
}


void BMPCCSecurityCallback::onPassKeyNotify(uint32_t pass_key)
{
  Serial.println("not providing pass key.");
}

bool BMPCCSecurityCallback::onSecurityRequest()
{
  Serial.println("not allowing connection to this device.");
  return false;
}


void BMPCCSecurityCallback::onAuthenticationComplete(esp_ble_auth_cmpl_t status)
{
  Serial.print("Authentication Status: ");
  Serial.println( status.success );
}


bool BMPCCSecurityCallback::onConfirmPIN(uint32_t pin)
{
  Serial.print("PIN: ");
  Serial.println(pin);
  return (0 != pin);
}
