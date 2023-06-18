#pragma once

#include <libindi/indibase.h>
#include <libindi/defaultdevice.h>


namespace Connection {
  class Serial;
}

class Powerbox : public INDI::DefaultDevice {
  public:
    Powerbox();
    ~Powerbox();
  protected:
    virtual const char *getDefaultName() override;
};
