#include "BMPCC_BlueTooth.h"

#include <HardwareSerial.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

#include "BMPCC_BlueTooth.h"
#include "BMPCCAdvertisedDeviceCallbacks.h"
#include "BMPCCSecurityCallback.h"
#include "BMPCCBLEClientCallbacks.h"

namespace
{
uint8_t record[] =   {255, // broadcast
                      9,   // length - only include up to mode ([8])
                      0,   // command id - Command 0 : change configuration
                      0,   // reserved
                      10,  // category 10 - Media
                      1,   // parameter 1 - Transport mode
                      1,   // data type: 1 == signed byte
                      0,   // operation type: 0 == assign value
                      0,   // [8] (mode) 0 == preview, 1 == play, 2 == record
                      0,   // [9] (speed)  0 == pause
                      0,   // [A] (flags)   0 == loop
                      0,   // [B] (storage) 0 == CFast, 1 == SD
                      0, 0, 0, 0
                     }; // 0=OFF, 2=ON, [8]


typedef enum { BROADCAST_POS = 0, LENGTH_POS, COMMAND_ID_POS, RESERVED_POS, CATEGORY_POS, PARAMETER_POS,
               DATA_TYPE_POS, OPERATION_TYPE_POS, DATA_0_POS, DATA_1_POS, DATA_2_POS, DATA_3_POS
             } DATA_t;
typedef enum { PREVIEW = 0, PLAY = 1, RECORD = 2 } TRANSPORT_MODE_t;
};



BMPCC_BlueTooth::BMPCC_BlueTooth()
  : /** constants */
    DEVICE_NAME( "BMPCC_Remote" )
  , SCAN_INTERVAL( 1200 )
  , SCAN_WINDOW( 800 )
    /** camera service */
  , m_BMPCC_cameraControlServiceUUID("291D567A-6D75-11E6-8B77-86F30CA893D3")
    /** device info service */
  , m_BMPCC_deviceInfoServiceUUID("180A")
    /** characteristics */
  , m_BMPCC_manufacturerCharacteristicUUID("2A29")
  , m_BMPCC_cameraModelCharacteristicUUID("2A24")
  , m_BMPCC_cameraOutControlCharacteristicUUID("5DD3465F-1AEE-4299-8493-D2ECA2F8E1BB")
  , m_BMPCC_cameraInControlCharacteristicUUID("B864E140-76A0-416A-BF30-5876504537D9")
  , m_BMPCC_timecodeCharacteristicUUID("6D8F2110-86F1-41BF-9AFB-451D87E976C8")
  , m_BMPCC_cameraStatusCharacteristicUUID("7FE8691D-95DC-4FC5-8ABD-CA74339B51B9")
  , m_BMPCC_deviceNameCharacteristicUUID("FFAC0C52-C9FB-41A0-B063-CC76282EB89C")
  , m_BMPCC_protocolVersionCharacteristicUUID("8F1FD018-B508-456F-8F82-3D392BEE2706")
    /** bluetooth device */
  , m_pBMPCC_Camera( NULL )
  , m_deviceFound( false )
    /** status */
  , m_connected( false )
  , m_camera_PIN( 0 )
  , m_pBMPCC_cameraOutControlCharacteristic( NULL )
  , m_pBMPCC_cameraInControlCharacteristic( NULL )
{
}

BMPCC_BlueTooth::~BMPCC_BlueTooth()
{
}

void BMPCC_BlueTooth::setup( const int scan_time )
{
  // initialize BLE
  BLEDevice::init(DEVICE_NAME);
  // get scan object
  BLEScan* pBLEScan = BLEDevice::getScan();
  // set callback
  pBLEScan->setAdvertisedDeviceCallbacks(new BMPCCAdvertisedDeviceCallbacks( &m_pBMPCC_Camera
                                         , &m_deviceFound
                                         , m_BMPCC_cameraControlServiceUUID ));
  pBLEScan->setInterval(SCAN_INTERVAL);
  pBLEScan->setWindow(SCAN_WINDOW);
  pBLEScan->setActiveScan(true);
  pBLEScan->start( scan_time, false);
}

void BMPCC_BlueTooth::control( const int scan_time, const int button_pin, const int led_pin )
{
  // if not yet connected, try again
  if (! m_deviceFound)
  {
    delay( 1000 );
    Serial.println("  --- scanning again.");
    BLEDevice::getScan()->start( scan_time, false);
    return;
  }

  if (! m_connected)
  {
    if (getConnected())
    {
      Serial.println("We are now connected to the BLE Server.");
      m_connected = true;
    }
    else
    {
      Serial.println("Not connected to the server; will try again.");
      m_deviceFound = false;
      return;
    }
  }

  // First and simplest test is to flash the LED and respond to the button.
  static int buttonState = digitalRead( button_pin );         // variable for reading the pushbutton status
  static int lightState = LOW;
  static bool switchState = true;
  // check if the pushbutton is pressed. If it is, the buttonState is LOW:
  if ( LOW == buttonState )
  {
    if ( switchState )
    {
      lightState = (lightState == LOW ? HIGH : LOW);
      digitalWrite( led_pin, lightState );
      Serial.print( "  Changed record state: " );
      Serial.println( lightState );
      switchState = false;
      setRecordState( lightState );
    }
  }
  else
  {
    switchState = true;
  }
  // read the state of the pushbutton value:
  buttonState = digitalRead( button_pin );
}



/**

*/
void BMPCC_BlueTooth::listServices( BLEClient *pClient )
{
  // list all services for informational purposes
  std::map<std::string, BLERemoteService*> *pServiceMap = pClient->getServices();
  Serial.print("Number of entries in service map: ");
  Serial.println( pServiceMap->size() );
  std::map<std::string, BLERemoteService*>::const_iterator itr = pServiceMap->begin();
  std::map<std::string, BLERemoteService*>::const_iterator ite = pServiceMap->end();
  for (; itr != ite; ++itr )
  {
    std::string msg = "Service [" + itr->first + "]  == " + itr->second->toString();
    Serial.println( msg.c_str() );
    // list characteristics of the service
    std::map<uint16_t, BLERemoteCharacteristic*>* characteristics = itr->second->getCharacteristicsByHandle();
    std::map<uint16_t, BLERemoteCharacteristic*>::const_iterator citr = characteristics->begin();
    std::map<uint16_t, BLERemoteCharacteristic*>::const_iterator cite = characteristics->end();
    for (; citr != cite; ++citr)
    {
      std::string cmsg = "    characteristic == " + citr->second->toString();
      Serial.println( cmsg.c_str() );
    }
  }
}



/**

*/
BLERemoteService * BMPCC_BlueTooth::connectToService( BLEClient* pClient,  BLEUUID &uuid )
{
  BLERemoteService *pService = pClient->getService( uuid );
  if ( nullptr == pService )
  {
    std::string msg = "Failed to find our service UUID: " + uuid.toString();
    Serial.println( msg.c_str() );
    pClient->disconnect();
    return NULL;
  }
  return ( pService );
}



/**

*/
BLERemoteCharacteristic * BMPCC_BlueTooth::getCharacteristic( BLERemoteService *pCameraInfoSvc,  BLEUUID &uuid )
{
  BLERemoteCharacteristic *characteristic = pCameraInfoSvc->getCharacteristic( uuid );
  if ( nullptr == characteristic )
  {
    std::string msg = "Failed to find characteristic using UUID: " + uuid.toString();
    Serial.println( msg.c_str() );
    return NULL;
  }
  return characteristic;
}



/**

*/
void BMPCC_BlueTooth::cameraStatusNotifyCallback( BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify )
{
  Serial.print("camera status notify callback");
  Serial.println( pBLERemoteCharacteristic->getUUID().toString().c_str() );
  Serial.print("    length: ");
  Serial.println( length );
  Serial.print("    isNotify: ");
  Serial.println( isNotify );
  uint8_t * pStatus = (uint8_t *)pData;
  Serial.print("    Status: ");
  Serial.println( *pStatus );
}



/**

*/
void BMPCC_BlueTooth::timecodeNotifyCallback( BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify )
{
  Serial.print("camera timecode notify callback");
  Serial.println( pBLERemoteCharacteristic->getUUID().toString().c_str() );
  Serial.print("    length: ");
  Serial.println( length );
  Serial.print("    isNotify: ");
  Serial.println( isNotify );
}



/**

*/
void BMPCC_BlueTooth::cameraInControlNotifyCallback( BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify )
{
  Serial.print("camera in control notify callback");
  Serial.println( pBLERemoteCharacteristic->getUUID().toString().c_str() );
  Serial.print("    length: ");
  Serial.println( length );
  Serial.print("    isNotify: ");
  Serial.println( isNotify );
}



/**

*/
bool BMPCC_BlueTooth::getConnected()
{
  std::string msg = "Camera address: " + m_pBMPCC_Camera->getAddress().toString();
  Serial.println( msg.c_str() );

  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks( new BMPCCSecurityCallback() );
  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setKeySize();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
  pSecurity->setCapability(ESP_IO_CAP_IN);
  pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  BLEClient*  pBLEClient  = BLEDevice::createClient();

  pBLEClient->setClientCallbacks( new BMPCCBLEClientCallbacks( & m_connected ) );
  pBLEClient->connect( m_pBMPCC_Camera );

  listServices( pBLEClient );

  // get the camera device information service
  BLERemoteService *pCameraInfoSvc = connectToService( pBLEClient, m_BMPCC_deviceInfoServiceUUID );
  if ( pCameraInfoSvc )
  {
    // confirm that this is a BMD camera
    BLERemoteCharacteristic *pBMPCC_ManufacturerCharacteristic = getCharacteristic( pCameraInfoSvc, m_BMPCC_manufacturerCharacteristicUUID );
    if ( NULL == pBMPCC_ManufacturerCharacteristic )
    {
      pBLEClient->disconnect();
      return false;
    }
    else
    {
      std::string manufacturer = pBMPCC_ManufacturerCharacteristic->readValue();
      if ( 0 != manufacturer.compare("Blackmagic Design") )
      {
        std::string msg = "Invalid manufacturer: " + manufacturer;
        pBLEClient->disconnect();
        return false;
      }
      else
      {
        std::string msg = "Manufacturer: " + manufacturer;
        Serial.println( msg.c_str() );
      }
    }

    // confirm that this is a BMPCC4k or BMPCC6k
    BLERemoteCharacteristic *pBMPCC_ModelCharacteristic = getCharacteristic( pCameraInfoSvc, m_BMPCC_cameraModelCharacteristicUUID );
    if ( NULL == pBMPCC_ModelCharacteristic )
    {
      pBLEClient->disconnect();
      return false;
    }
    else
    {
      std::string model = pBMPCC_ModelCharacteristic->readValue();
      if ( 0 != model.substr(0, 21).compare( "Pocket Cinema Camera " ) )
      {
        std::string msg = "Not a BMPCC4k or BMPCC6k.  Model: " + model;
        Serial.println( msg.c_str() );
        pBLEClient->disconnect();
        return false;
      }
      else
      {
        std::string msg = "Model: " + model;
        Serial.println( msg.c_str() );
      }
    }
  }

  // get the camera control service
  BLERemoteService *pCameraControlService = connectToService( pBLEClient, m_BMPCC_cameraControlServiceUUID );
  if ( pCameraControlService )
  {

    // device name
    BLERemoteCharacteristic *pBMPCC_deviceNameCharacteristic = getCharacteristic( pCameraControlService, m_BMPCC_deviceNameCharacteristicUUID );
    if ( NULL == pBMPCC_deviceNameCharacteristic )
    {
      pBLEClient->disconnect();
      return false;
    }
    else
    {
      if ( pBMPCC_deviceNameCharacteristic->canWrite() )
      {
        pBMPCC_deviceNameCharacteristic->writeValue( DEVICE_NAME );
      }
    }

    // protocol version
    BLERemoteCharacteristic *pBMPCC_protocolVersionCharacteristic = getCharacteristic( pCameraControlService, m_BMPCC_protocolVersionCharacteristicUUID );
    if ( NULL == pBMPCC_protocolVersionCharacteristic )
    {
      pBLEClient->disconnect();
      return false;
    }
    else
    {
      std::string value = pBMPCC_protocolVersionCharacteristic->readValue();
      std::string msg = "Protocol version: " + value;
      Serial.println( msg.c_str() );
    }

    // camera status
    BLERemoteCharacteristic *pBMPCC_cameraStatusCharacteristic = getCharacteristic( pCameraControlService, m_BMPCC_cameraStatusCharacteristicUUID );
    if ( NULL == pBMPCC_cameraStatusCharacteristic )
    {
      pBLEClient->disconnect();
      return false;
    }
    else
    {
      if ( pBMPCC_cameraStatusCharacteristic->canNotify() )
      {
        pBMPCC_cameraStatusCharacteristic->registerForNotify( cameraStatusNotifyCallback );
      }
    }

    // timecode
    BLERemoteCharacteristic *pBMPCC_timecodeCharacteristic = getCharacteristic( pCameraControlService, m_BMPCC_timecodeCharacteristicUUID );
    if ( NULL == pBMPCC_timecodeCharacteristic )
    {
      pBLEClient->disconnect();
      return false;
    }
    else
    {
      if ( pBMPCC_cameraStatusCharacteristic->canNotify() )
      {
        pBMPCC_cameraStatusCharacteristic->registerForNotify( timecodeNotifyCallback );
      }
    }

    // cameera in control
    m_pBMPCC_cameraInControlCharacteristic = getCharacteristic( pCameraControlService, m_BMPCC_cameraInControlCharacteristicUUID );
    if ( NULL == m_pBMPCC_cameraInControlCharacteristic )
    {
      pBLEClient->disconnect();
      return false;
    }
    else
    {
      if ( m_pBMPCC_cameraInControlCharacteristic->canNotify() )
      {
        m_pBMPCC_cameraInControlCharacteristic->registerForNotify( cameraInControlNotifyCallback );
      }
    }

    // camera out control
    m_pBMPCC_cameraOutControlCharacteristic = getCharacteristic( pCameraControlService, m_BMPCC_cameraOutControlCharacteristicUUID );
    if ( NULL == m_pBMPCC_cameraOutControlCharacteristic )
    {
      pBLEClient->disconnect();
      return false;
    }
    else
    {
      std::string value = m_pBMPCC_cameraOutControlCharacteristic->readValue();
      std::string msg = "out control: " + value;
      Serial.println( msg.c_str() );
    }
  }

  Serial.println("Done getting connected.");
  return true;
}


void BMPCC_BlueTooth::setRecordState(bool state)
{
  if (!state)
  {
    record[8] = PREVIEW;
  }
  else
  {
    record[8] = RECORD;
  }
  m_pBMPCC_cameraOutControlCharacteristic->writeValue((uint8_t*)record, 16, true);
}
