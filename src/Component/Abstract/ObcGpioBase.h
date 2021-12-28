#pragma once
#include "../CDH/OBC.h"

class ObcGpioBase
{
public:
  ObcGpioBase(
    const std::vector<int> port_id, 
    OBC* obc
  );
  ~ObcGpioBase();

protected:
  bool Read(const int idx);  // The first arg is the element index for port_id_ vector, not the GPIO port ID for OBC.
  void Write(const int idx, const bool is_high);  // The first arg is the element index for port_id_ vector, not the GPIO port ID for OBC.

private:
  std::vector<int> port_id_;
  OBC* obc_;
};
