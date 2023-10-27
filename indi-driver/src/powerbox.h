/*
 *  This file is part of the Pollux Astro Powerbox software
 *
 *  Created by Philipp Weber
 *  Copyright (c) 2023 Philipp Weber
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

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
    virtual bool ISNewNumber(const char *dev, const char *name, double *values, char *names[], int n) override;
    virtual void TimerHit() override;
  private:
    Connection::Serial *serialConnection{nullptr};
    static constexpr const char *DEW_TAB = "Dewheaters";
    // Voltage
    INumber VoltageN[1];
    INumberVectorProperty VoltageNP;
    // External environment sensor
    INumber EnvN[4];
    INumberVectorProperty EnvNP;
    INumber EnvOffsetN[1];
    INumberVectorProperty EnvOffsetNP;
    // 4x12V Rail
    ISwitch RailS[2];
    ISwitchVectorProperty RailSP;
    // Adj
    ISwitch AdjS[2];
    ISwitchVectorProperty AdjSP;
    INumber AdjN[1];
    INumberVectorProperty AdjNP;

    enum {
      MODE_FIXED = 0,
      MODE_DEWPOINT = 1,
      MODE_AMBIENT = 2,
      MODE_MIDPOINT = 3,
      MODE_SLAVE = 4
    };

    enum {
      DH_DC = 0,
      DH_TEMPERATURE = 1
    };

    // DH1
    ISwitch DH1ModeS[5];
    ISwitchVectorProperty DH1ModeSP;
    INumber DH1StatusN[2];
    INumberVectorProperty DH1StatusNP;
    INumber DH1TemperatureOffsetN[1];
    INumberVectorProperty DH1TemperatureOffsetNP;
    INumber DH1FixedN[1];
    INumberVectorProperty DH1FixedNP;
    INumber DH1DewpointOffsetN[1];
    INumberVectorProperty DH1DewpointOffsetNP;
    INumber DH1AmbientOffsetN[1];
    INumberVectorProperty DH1AmbientOffsetNP;
    INumber DH1MidpointOffsetN[1];
    INumberVectorProperty DH1MidpointOffsetNP;

    // DH2
    ISwitch DH2ModeS[5];
    ISwitchVectorProperty DH2ModeSP;
    INumber DH2StatusN[2];
    INumberVectorProperty DH2StatusNP;
    INumber DH2TemperatureOffsetN[1];
    INumberVectorProperty DH2TemperatureOffsetNP;
    INumber DH2FixedN[1];
    INumberVectorProperty DH2FixedNP;
    INumber DH2DewpointOffsetN[1];
    INumberVectorProperty DH2DewpointOffsetNP;
    INumber DH2AmbientOffsetN[1];
    INumberVectorProperty DH2AmbientOffsetNP;
    INumber DH2MidpointOffsetN[1];
    INumberVectorProperty DH2MidpointOffsetNP;

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
    void setEnvironmentOffset(const json& data);
    void setRail(const json& data);
    void setAdjState(const json& data);
    void setAdjVoltage(const json& data);
    void setDH1Mode(const json& data);
    void setDH1Status(const json& data);
    void setDH1Params(const json& data);
    void setDH2Mode(const json& data);
    void setDH2Status(const json& data);
    void setDH2Params(const json& data);
    bool processRailSP(ISState * states, char * names[], int n);
    bool processAdjSP(ISState * states, char * names[], int n);
    bool processEnvOffsetNP(double *values);
    bool processAdjNP(double *values);
    bool processDH1Mode(ISState * states, char * names[], int n);
    bool processDH1TemperatureOffset(double *values);
    bool processDH1Fixed(double *values);
    bool processDH1DewpointOffset(double *values);
    bool processDH1AmbientOffset(double *values);
    bool processDH1MidpointOffset(double *values);
    bool processDH2Mode(ISState * states, char * names[], int n);
    bool processDH2TemperatureOffset(double *values);
    bool processDH2Fixed(double *values);
    bool processDH2DewpointOffset(double *values);
    bool processDH2AmbientOffset(double *values);
    bool processDH2MidpointOffset(double *values);
};
