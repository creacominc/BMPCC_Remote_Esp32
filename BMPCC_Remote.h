#ifndef BMPCC_REMOTE_H
#define BMPCC_REMOTE_H

#include <BLEClient.h>
#include <BLERemoteService.h>
#include <BLERemoteCharacteristic.h>
#include <BLEAdvertisedDevice.h>

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
  BMPCC_Remote( const int _BUTTON_PIN, const int _LED_PIN );
  virtual ~BMPCC_Remote();

  void setup();
  void control();


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
  const int BUTTON_PIN;
  const int LED_PIN;
  const std::string DEVICE_NAME;
  const int SCAN_INTERVAL;
  const int SCAN_WINDOW;
  const int SCAN_TIME;

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
