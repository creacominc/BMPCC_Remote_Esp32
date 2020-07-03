#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <string>


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

const std::string DEVICE_NAME("BMPCC_Remote");
const int BAUD_RATE  = 115200;
const int LED_PIN    = 5;
const int BUTTON_PIN = 0;
const int SCAN_INTERVAL = 1200;
const int SCAN_WINDOW   = 800;
const int SCAN_TIME     = 5;

/** device info service */
static BLEUUID BMPCC_deviceInfoServiceUUID("180A");
static BLEUUID BMPCC_manufacturerCharacteristicUUID("2A29");
static BLEUUID BMPCC_cameraModelCharacteristicUUID("2A24");

/** camera service */
static BLEUUID BMPCC_cameraControlServiceUUID("291D567A-6D75-11E6-8B77-86F30CA893D3");
//static BLEUUID BMPCC_serviceUUID("00001800-0000-1000-8000-00805f9b34fb");
static BLEUUID BMPCC_cameraOutControlCharacteristicUUID("5DD3465F-1AEE-4299-8493-D2ECA2F8E1BB");
static BLEUUID BMPCC_cameraInControlCharacteristicUUID("B864E140-76A0-416A-BF30-5876504537D9");
static BLEUUID BMPCC_timecodeCharacteristicUUID("6D8F2110-86F1-41BF-9AFB-451D87E976C8");
static BLEUUID BMPCC_cameraStatusCharacteristicUUID("7FE8691D-95DC-4FC5-8ABD-CA74339B51B9");
static BLEUUID BMPCC_deviceNameCharacteristicUUID("FFAC0C52-C9FB-41A0-B063-CC76282EB89C");
static BLEUUID BMPCC_protocolVersionCharacteristicUUID("8F1FD018-B508-456F-8F82-3D392BEE2706");

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

static BLEAdvertisedDevice *pBMPCC_Camera = NULL;
static bool deviceFound = false;
static bool connected   = false;

static BLERemoteCharacteristic *pBMPCC_cameraOutControlCharacteristic = NULL;
static BLERemoteCharacteristic *pBMPCC_cameraInControlCharacteristic = NULL;

static uint32_t camera_PIN = 0;

/**
 * For each advertised device, check to see if it is a BMPCC camera.
 */
class BMPCCAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());
    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID()
        && advertisedDevice.isAdvertisingService( BMPCC_cameraControlServiceUUID )) 
    {
      // stop scanning
      BLEDevice::getScan()->stop();
      Serial.println("found camera.");
      pBMPCC_Camera = new BLEAdvertisedDevice( advertisedDevice );
      deviceFound = true;
    }
  }
};


/**
 * Security callback class handles the security requests from the service
 */
class BMPCCSecurityCallback : public BLESecurityCallbacks
{
public:
  /**
   * @brief Its request from peer device to input authentication pin code displayed on peer device.
   * It requires that our device is capable to input 6-digits code by end user
   * @return Return 6-digits integer value from input device
   */
  virtual uint32_t onPassKeyRequest()
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

  /**
   * @brief Provide us 6-digits code to perform authentication.
   * It requires that our device is capable to display this code to end user
   * @param
   */
  virtual void onPassKeyNotify(uint32_t pass_key)
  {
    Serial.println("not providing pass key.");
  }

  /**
   * @brief Here we can make decision if we want to let negotiate authorization with peer device or not
   * return Return true if we accept this peer device request
   */
  virtual bool onSecurityRequest()
  {
    Serial.println("not allowing connection to this device.");
    return false;
  }

  /**
   * Provide us information when authentication process is completed
   */
  virtual void onAuthenticationComplete(esp_ble_auth_cmpl_t status)
  {
    Serial.print("Authentication Status: ");
    Serial.println( status.success );
  }

  virtual bool onConfirmPIN(uint32_t pin)
  {
    Serial.print("PIN: ");
    Serial.println(pin);
    return (0 != pin);
  }
};




/**
 * 
 */
class BMPCCBLEClientCallbacks : public BLEClientCallbacks
{
  virtual void onConnect(BLEClient *pClient)
  {
    Serial.println("Connected.");
    connected = true;
  }

  virtual void onDisconnect(BLEClient *pClient)
  {
    connected = false;
    pClient->disconnect();
    Serial.println("Disconnected.");
  }

};


/**
 * 
 */
void listServices( BLEClient *pClient )
{
  // list all services for informational purposes
  std::map<std::string, BLERemoteService*> *pServiceMap = pClient->getServices();
  Serial.print("Number of entries in service map: ");
  Serial.println( pServiceMap->size() );
  std::map<std::string, BLERemoteService*>::const_iterator itr = pServiceMap->begin();
  std::map<std::string, BLERemoteService*>::const_iterator ite = pServiceMap->end();
  for(; itr != ite; ++itr )
  {
    std::string msg = "Service [" + itr->first + "]  == " + itr->second->toString();
    Serial.println( msg.c_str() );
    // list characteristics of the service
    std::map<uint16_t, BLERemoteCharacteristic*>* characteristics = itr->second->getCharacteristicsByHandle();
    std::map<uint16_t, BLERemoteCharacteristic*>::const_iterator citr = characteristics->begin();
    std::map<uint16_t, BLERemoteCharacteristic*>::const_iterator cite = characteristics->end();
    for(; citr != cite; ++citr)
    {
      std::string cmsg = "    characteristic == " + citr->second->toString();
      Serial.println( cmsg.c_str() );
    }
  }
}

/**
 * 
 */
BLERemoteService * connectToService( BLEClient* pClient,  BLEUUID &uuid )
{
  BLERemoteService *pService = pClient->getService( uuid );
  if ( nullptr == pService ) 
  {
    std::string msg = "Failed to find our service UUID: " + uuid.toString();
    Serial.println( msg.c_str() );
    pClient->disconnect();
    return NULL;
  }
  return( pService );
}


/**
 * 
 */
BLERemoteCharacteristic * getCharacteristic( BLERemoteService *pCameraInfoSvc,  BLEUUID &uuid )
{
    BLERemoteCharacteristic *characteristic = pCameraInfoSvc->getCharacteristic( uuid );
    if( nullptr == characteristic )
    {
      std::string msg = "Failed to find characteristic using UUID: " + uuid.toString();
      Serial.println( msg.c_str() );
      return NULL;
    }
    return characteristic;
}


/**
 * 
 */
void cameraStatusNotifyCallback( BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify )
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
 * 
 */
void timecodeNotifyCallback( BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify )
{
  Serial.print("camera timecode notify callback");
  Serial.println( pBLERemoteCharacteristic->getUUID().toString().c_str() );
  Serial.print("    length: ");
  Serial.println( length );
  Serial.print("    isNotify: ");
  Serial.println( isNotify );
}


/**
 * 
 */
void cameraInControlNotifyCallback( BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify )
{
  Serial.print("camera in control notify callback");
  Serial.println( pBLERemoteCharacteristic->getUUID().toString().c_str() );
  Serial.print("    length: ");
  Serial.println( length );
  Serial.print("    isNotify: ");
  Serial.println( isNotify );
}



/**
 * 
 */
bool getConnected() 
{
  std::string msg = "Camera address: " + pBMPCC_Camera->getAddress().toString();
  Serial.println( msg.c_str() );

  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks( new BMPCCSecurityCallback() );
  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setKeySize();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
  pSecurity->setCapability(ESP_IO_CAP_IN);
  pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  BLEClient*  pBLEClient  = BLEDevice::createClient();

  pBLEClient->setClientCallbacks( new BMPCCBLEClientCallbacks() );
  pBLEClient->connect( pBMPCC_Camera );

  listServices( pBLEClient );

  // get the camera device information service
  BLERemoteService *pCameraInfoSvc = connectToService( pBLEClient, BMPCC_deviceInfoServiceUUID );
  if( pCameraInfoSvc )
  {
    // confirm that this is a BMD camera
    BLERemoteCharacteristic *pBMPCC_ManufacturerCharacteristic = getCharacteristic( pCameraInfoSvc, BMPCC_manufacturerCharacteristicUUID );
    if( NULL == pBMPCC_ManufacturerCharacteristic )
    {
      pBLEClient->disconnect();
      return false;
    }
    else
    {
      std::string manufacturer = pBMPCC_ManufacturerCharacteristic->readValue();
      if( 0 != manufacturer.compare("Blackmagic Design") )
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
    BLERemoteCharacteristic *pBMPCC_ModelCharacteristic = getCharacteristic( pCameraInfoSvc, BMPCC_cameraModelCharacteristicUUID );
    if( NULL == pBMPCC_ModelCharacteristic )
    {
      pBLEClient->disconnect();
      return false;
    }
    else
    {
      std::string model = pBMPCC_ModelCharacteristic->readValue();
      if( 0 != model.substr(0,21).compare( "Pocket Cinema Camera " ) )
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
  BLERemoteService *pCameraControlService = connectToService( pBLEClient, BMPCC_cameraControlServiceUUID );
  if( pCameraControlService )
  {

    // device name 
    BLERemoteCharacteristic *pBMPCC_deviceNameCharacteristic = getCharacteristic( pCameraControlService, BMPCC_deviceNameCharacteristicUUID );
    if( NULL == pBMPCC_deviceNameCharacteristic )
    {
      pBLEClient->disconnect();
      return false;
    }
    else
    {
      if( pBMPCC_deviceNameCharacteristic->canWrite() )
      {
        pBMPCC_deviceNameCharacteristic->writeValue( DEVICE_NAME );
      }
    }

    // protocol version
    BLERemoteCharacteristic *pBMPCC_protocolVersionCharacteristic = getCharacteristic( pCameraControlService, BMPCC_protocolVersionCharacteristicUUID );
    if( NULL == pBMPCC_protocolVersionCharacteristic )
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
    BLERemoteCharacteristic *pBMPCC_cameraStatusCharacteristic = getCharacteristic( pCameraControlService, BMPCC_cameraStatusCharacteristicUUID );
    if( NULL == pBMPCC_cameraStatusCharacteristic )
    {
      pBLEClient->disconnect();
      return false;
    }
    else
    {
      if( pBMPCC_cameraStatusCharacteristic->canNotify() )
      {
        pBMPCC_cameraStatusCharacteristic->registerForNotify( cameraStatusNotifyCallback );
      }
    }

    // timecode
    BLERemoteCharacteristic *pBMPCC_timecodeCharacteristic = getCharacteristic( pCameraControlService, BMPCC_timecodeCharacteristicUUID );
    if( NULL == pBMPCC_timecodeCharacteristic )
    {
      pBLEClient->disconnect();
      return false;
    }
    else
    {
      if( pBMPCC_cameraStatusCharacteristic->canNotify() )
      {
        pBMPCC_cameraStatusCharacteristic->registerForNotify( timecodeNotifyCallback );
      }
    }

    // cameera in control
    pBMPCC_cameraInControlCharacteristic = getCharacteristic( pCameraControlService, BMPCC_cameraInControlCharacteristicUUID );
    if( NULL == pBMPCC_cameraInControlCharacteristic )
    {
      pBLEClient->disconnect();
      return false;
    }
    else
    {
      if( pBMPCC_cameraInControlCharacteristic->canNotify() )
      {
        pBMPCC_cameraInControlCharacteristic->registerForNotify( cameraInControlNotifyCallback );
      }
    }

    // camera out control
    pBMPCC_cameraOutControlCharacteristic = getCharacteristic( pCameraControlService, BMPCC_cameraOutControlCharacteristicUUID );
    if( NULL == pBMPCC_cameraOutControlCharacteristic )
    {
      pBLEClient->disconnect();
      return false;
    }
    else
    {
      std::string value = pBMPCC_cameraOutControlCharacteristic->readValue();
      std::string msg = "out control: " + value;
      Serial.println( msg.c_str() );
    }
  }

  Serial.println("Done getting connected.");
  return true;
}


/**
 * 
 */
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
                      0, 0, 0, 0}; // 0=OFF, 2=ON, [8]
typedef enum { BROADCAST_POS=0, LENGTH_POS, COMMAND_ID_POS, RESERVED_POS, CATEGORY_POS, PARAMETER_POS, 
                DATA_TYPE_POS, OPERATION_TYPE_POS, DATA_0_POS, DATA_1_POS, DATA_2_POS, DATA_3_POS } DATA_t;
typedef enum { PREVIEW=0, PLAY=1, RECORD=2 } TRANSPORT_MODE_t;
void setRecordState(boolean state)
{
  if (!state)
  {
    record[8] = PREVIEW;
  }
  else
  {
    record[8] = RECORD;
  }
  pBMPCC_cameraOutControlCharacteristic->writeValue((uint8_t*)record, 16, true);
}

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

  // initialize BLE
  BLEDevice::init(DEVICE_NAME);
  // get scan object
  BLEScan* pBLEScan = BLEDevice::getScan();
  // set callback
  pBLEScan->setAdvertisedDeviceCallbacks(new BMPCCAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(SCAN_INTERVAL);
  pBLEScan->setWindow(SCAN_WINDOW);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(SCAN_TIME, false);
}

void loop()
{
  // if not yet connected, try again
  if(! deviceFound)
  {
      delay( 1000 );
      Serial.println("  --- scanning again.");
      BLEDevice::getScan()->start(SCAN_TIME, false);
      return;
  }

  if(! connected)
  {
    if (getConnected())
    {
      Serial.println("We are now connected to the BLE Server.");
      connected = true;
    }
    else
    {
      Serial.println("Not connected to the server; will try again.");
      deviceFound = false;
      return;
    }
  }

  // First and simplest test is to flash the LED and respond to the button.
  static int buttonState = digitalRead(BUTTON_PIN);         // variable for reading the pushbutton status
  static int lightState = LOW;
  static bool switchState = true;
  // check if the pushbutton is pressed. If it is, the buttonState is LOW:
  if( LOW == buttonState )
  {
    if( switchState )
    {
      lightState = (lightState == LOW ? HIGH : LOW);
      digitalWrite(LED_PIN, lightState);
      Serial.print("  Changed record state: ");
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
  buttonState = digitalRead(BUTTON_PIN);
  delay( 100 );
}
