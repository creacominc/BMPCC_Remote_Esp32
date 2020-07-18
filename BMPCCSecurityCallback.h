#ifndef BMPCCSECURITYCALLBACK_H
#define BMPCCSECURITYCALLBACK_H

#include <BLESecurity.h>

/**
 * Security callback class handles the security requests from the service
 */
class BMPCCSecurityCallback : public BLESecurityCallbacks
{
public:
    BMPCCSecurityCallback();
    virtual ~BMPCCSecurityCallback();

    /**
   * @brief Its request from peer device to input authentication pin code displayed on peer device.
   * It requires that our device is capable to input 6-digits code by end user
   * @return Return 6-digits integer value from input device
   */
    virtual uint32_t onPassKeyRequest();

    /**
   * @brief Provide us 6-digits code to perform authentication.
   * It requires that our device is capable to display this code to end user
   * @param
   */
    virtual void onPassKeyNotify(uint32_t pass_key);

    /**
   * @brief Here we can make decision if we want to let negotiate authorization with peer device or not
   * return Return true if we accept this peer device request
   */
    virtual bool onSecurityRequest();

    /**
   * Provide us information when authentication process is completed
   */
    virtual void onAuthenticationComplete(esp_ble_auth_cmpl_t status);

    virtual bool onConfirmPIN(uint32_t pin);
};

#endif
