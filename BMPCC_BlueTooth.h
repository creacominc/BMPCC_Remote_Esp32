#ifndef BMPCC_BLUETOOTH_H
#define BMPCC_BLUETOOTH_H

#include <BLEClient.h>
#include <BLERemoteService.h>
#include <BLERemoteCharacteristic.h>
#include <BLEAdvertisedDevice.h>

class BMPCC_BlueTooth
{
  public:
    BMPCC_BlueTooth();
    virtual ~BMPCC_BlueTooth();

    void setup( const int scan_time );
    void control( const int scan_time, const int button_pin, const int led_pin );


  protected:
    void listServices( BLEClient *pClient );
    BLERemoteService * connectToService( BLEClient* pClient,  BLEUUID &uuid );
    BLERemoteCharacteristic * getCharacteristic( BLERemoteService *pCameraInfoSvc,  BLEUUID &uuid );

    /** callbacks */
    static void cameraStatusNotifyCallback( BLERemoteCharacteristic* pBLERemoteCharacteristic
                                            , uint8_t* pData, size_t length, bool isNotify );
    static void timecodeNotifyCallback( BLERemoteCharacteristic* pBLERemoteCharacteristic
                                        , uint8_t* pData, size_t length, bool isNotify );
    static void cameraInControlNotifyCallback( BLERemoteCharacteristic* pBLERemoteCharacteristic
        , uint8_t* pData, size_t length, bool isNotify );

    bool getConnected();
    void setRecordState(bool state);


  private:
    /** constants */
    const std::string DEVICE_NAME;
    const int SCAN_INTERVAL;
    const int SCAN_WINDOW;

    /** camera service */
    BLEUUID m_BMPCC_cameraControlServiceUUID;
    //static BLEUUID BMPCC_serviceUUID("00001800-0000-1000-8000-00805f9b34fb");

    /** device info service */
    BLEUUID m_BMPCC_deviceInfoServiceUUID;
    /** characteristics */
    BLEUUID m_BMPCC_manufacturerCharacteristicUUID;
    BLEUUID m_BMPCC_cameraModelCharacteristicUUID;
    BLEUUID m_BMPCC_cameraOutControlCharacteristicUUID;
    BLEUUID m_BMPCC_cameraInControlCharacteristicUUID;
    BLEUUID m_BMPCC_timecodeCharacteristicUUID;
    BLEUUID m_BMPCC_cameraStatusCharacteristicUUID;
    BLEUUID m_BMPCC_deviceNameCharacteristicUUID;
    BLEUUID m_BMPCC_protocolVersionCharacteristicUUID;

    /** bluetooth device */
    BLEAdvertisedDevice * m_pBMPCC_Camera;
    bool m_deviceFound;

    /** status */
    bool m_connected;
    uint32_t m_camera_PIN;

    BLERemoteCharacteristic *m_pBMPCC_cameraOutControlCharacteristic;
    BLERemoteCharacteristic *m_pBMPCC_cameraInControlCharacteristic;


};

#endif
