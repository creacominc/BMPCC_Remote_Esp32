#ifndef BMPCC_REMOTE_H
#define BMPCC_REMOTE_H

#include "BMPCC_BlueTooth.h"
#include "BMPCC_Wifi.h"

/**
  Service: Device Information Service       UUID: 180A
  Characteristics
  Camera Manufacturer                     UUID: 2A29   Read the name of the manufacturer (always “Blackmagic Design”).
  Camera Model                            UUID: 2A24   Read the name of the camera model (eg. “URSA Mini Pro”).

  Service: Blackmagic Camera Service        UUID: 291D567A-6D75-11E6-8B77-86F30CA893D3
  Characteristics
  Outgoing Camera Control (encrypted)       UUID: 5DD3465F-1AEE-4299-8493-D2ECA2F8E1BB
  Incoming Camera Control (encrypted)       UUID: B864E140-76A0-416A-BF30-5876504537D9
  Timecode (encrypted)                      UUID: 6D8F2110-86F1-41BF-9AFB-451D87E976C8
  Camera Status (encrypted)                 UUID: 7FE8691D-95DC-4FC5-8ABD-CA74339B51B9
  Device Name                               UUID: FFAC0C52-C9FB-41A0-B063-CC76282EB89C
  Protocol Version                          UUID: 8F1FD018-B508-456F-8F82-3D392BEE2706
*/

class BMPCC_Remote
{
public:
    BMPCC_Remote(const int _BUTTON_PIN, const int _LED_PIN);
    virtual ~BMPCC_Remote();

    void setup();
    void control();

protected:
    BMPCC_BlueTooth m_bmpcc_bluetooth;
    BMPCC_Wifi m_bmpcc_wifi;

private:
    /** constants */
    const int BUTTON_PIN;
    const int LED_PIN;
    const int SCAN_TIME;
};

#endif
