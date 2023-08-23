#pragma once
#include <string>

class HydraulicSystemMonitor {

private:
  std::string hydraulic_oil_temperature;
  std::string hydraulic_oil_tank_fill_level;
  std::string hydraulic_oil_pressure;

public:
  std::string get_oil_temperature();
  std::string get_oil_tank_fill_level();
  std::string get_oil_pressure();
  HydraulicSystemMonitor(std::string oil_temp, std::string oil_level,
                         std::string oil_pressure);
};