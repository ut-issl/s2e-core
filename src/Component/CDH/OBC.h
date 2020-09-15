#pragma once
#include "../Abstract/ComponentBase.h"

class OBC: public ComponentBase
{
public:
    OBC(ClockGenerator* clock_gen);
    ~OBC();
    void Initialize();
    double GetCurrent(int port_id) const;
protected:
    void MainRoutine(int count);
};

