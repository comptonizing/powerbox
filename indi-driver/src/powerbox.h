#pragma once

#include <string.h>
#include <termios.h>

#include <indibase.h>
#include <defaultdevice.h>
#include <connectionplugins/connectionserial.h>

#include "json.hpp"

#define CMDBUFF 1024
#define ERRBUFF 256
#define RSPBUFF 1024


namespace Connection {
  class Serial;
}

class Powerbox : public INDI::DefaultDevice {
  public:
    Powerbox();
    ~Powerbox();
  protected:
    virtual const char *getDefaultName() override;
    virtual bool initProperties() override;
    virtual bool loadConfig(bool silent, const char *property) override;
    /*
    virtual void TimerHit() override;
    virtual bool updateProperties() override;
    */
  private:
    Connection::Serial *serialConnection{nullptr};

    bool sendCommand(const char *cmd, char *rsp);
    virtual bool Handshake();
    uint16_t crc16_update(uint16_t crc, uint8_t a);
    uint16_t crcCalc(const void *data, size_t n);
    uint16_t crcCalc(const char *str);
    void cmdCrc(const char *cmd, char *out);
    bool checkCrc(const char *rsp);
};
