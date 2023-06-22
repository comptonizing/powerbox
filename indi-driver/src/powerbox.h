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

#define TIMEOUT 2

using json = nlohmann::json;

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
    virtual bool updateProperties() override;
    virtual bool ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n) override;
    virtual void TimerHit() override;
  private:
    Connection::Serial *serialConnection{nullptr};
    // Voltage
    INumber VoltageN[1];
    INumberVectorProperty VoltageNP;
    // External environment sensor
    INumber EnvN[4];
    INumberVectorProperty EnvNP;
    // 4x12V Rail
    ISwitch RailS[2];
    ISwitchVectorProperty RailSP;

    enum {
      TEMPERATURE,
      HUMIDITY,
      PRESSURE,
      DEWPOINT
    };

    enum {
      ON,
      OFF
    };

    bool sendCommand(const char *cmd, char *rsp);
    virtual bool Handshake();
    uint16_t crc16_update(uint16_t crc, uint8_t a);
    uint16_t crcCalc(const void *data, size_t n);
    uint16_t crcCalc(const char *str);
    void cmdCrc(const char *cmd, char *out);
    bool checkCrc(const char *rsp);
    const json& getJSON();
    bool updateFromResponse(const char *rsp);
    bool update();
    void setVoltage(const json& data);
    void setEnvironment(const json& data);
    void setRail(const json& data);
};
