#include "BMPCC_Remote.h"

#include <HardwareSerial.h>
#include <BLEDevice.h>
#include <string>





BMPCC_Remote::BMPCC_Remote( const int _BUTTON_PIN, const int _LED_PIN )
  : /** constants */
    BUTTON_PIN( _BUTTON_PIN )
  , LED_PIN( _LED_PIN )
  , SCAN_TIME( 5 )
{
}


BMPCC_Remote::~BMPCC_Remote()
{
}



void BMPCC_Remote::setup()
{
  m_bmpcc_bluetooth.setup( SCAN_TIME );
}

void BMPCC_Remote::control()
{
  m_bmpcc_bluetooth.control( SCAN_TIME, BUTTON_PIN, LED_PIN );
}
