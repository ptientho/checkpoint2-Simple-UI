#include "robot_info/hydraulic_system_monitor.h"
#include <string>

HydraulicSystemMonitor::HydraulicSystemMonitor(std::string oil_temp,
                                               std::string oil_level,
                                               std::string oil_pressure)
    : hydraulic_oil_pressure(oil_pressure),
      hydraulic_oil_tank_fill_level(oil_level),
      hydraulic_oil_temperature(oil_temp) {}

std::string HydraulicSystemMonitor::get_oil_pressure() {

  return this->hydraulic_oil_pressure;
}

std::string HydraulicSystemMonitor::get_oil_tank_fill_level() {

  return this->hydraulic_oil_tank_fill_level;
}

std::string HydraulicSystemMonitor::get_oil_temperature() {

  return this->hydraulic_oil_temperature;
}
