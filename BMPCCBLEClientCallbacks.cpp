#include "BMPCCBLEClientCallbacks.h"
#include <HardwareSerial.h>

BMPCCBLEClientCallbacks::BMPCCBLEClientCallbacks( bool * _pconnected )
  : BLEClientCallbacks()
  , m_pconnected( _pconnected )
{
}

BMPCCBLEClientCallbacks::~BMPCCBLEClientCallbacks()
{
}

void BMPCCBLEClientCallbacks::onConnect(BLEClient *pClient)
  {
    Serial.println("Connected.");
    (*m_pconnected) = true;
  }

void BMPCCBLEClientCallbacks::onDisconnect(BLEClient *pClient)
  {
    (*m_pconnected) = false;
    pClient->disconnect();
    Serial.println("Disconnected.");
  }
