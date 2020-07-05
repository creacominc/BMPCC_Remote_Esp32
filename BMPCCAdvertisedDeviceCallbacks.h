#ifndef BMPCCADVERTISEDDEVICECALLBACKS_H
#define BMPCCADVERTISEDDEVICECALLBACKS_H

#include <BLEAdvertisedDevice.h>

/**
 * For each advertised device, check to see if it is a BMPCC camera.
 */
class BMPCCAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks
{
  public:
    BMPCCAdvertisedDeviceCallbacks( BLEAdvertisedDevice ** _Camera,
                                    bool * _deviceFound,
                                    BLEUUID & _cameraControlServiceUUID );
    virtual ~BMPCCAdvertisedDeviceCallbacks();

  protected:
    virtual void onResult(BLEAdvertisedDevice advertisedDevice);

  private:
    BLEAdvertisedDevice ** m_Camera;
    bool * m_pdeviceFound;
    BLEUUID & m_cameraControlServiceUUID;

};


#endif
