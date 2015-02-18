#ifndef THERMO_MAX6675_HH
#define THERMO_MAX6675_HH

#include <stdint.h>

class ThermoMax6675
{
 public:
    ThermoMax6675();
    bool Setup();
    void SetChipSelected(const bool selected);
    uint16_t GetValue();
}; //class ThermoMax6675

#endif //ifndef THERMO_MAX6675_HH
