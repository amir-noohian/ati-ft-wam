#pragma once
#include "link4_wrench_sum_math.hpp"
#include "link4_sensor_config.hpp"

namespace link4_wrench_sum {
inline Settings loadSettings(const std::string& path) {
    try {
        libconfig::Config config;
        config.readFile(path.c_str());
        Settings result;
        result.gain = link4_sensor::numericSetting(config.lookup("gain"));
        result.sign = link4_sensor::numericSetting(config.lookup("sign"));
        result.filterHz = link4_sensor::numericSetting(config.lookup("filter_hz"));
        result.torqueLimit = link4_sensor::numericSetting(config.lookup("torque_limit_nm"));
        result.slewLimit = link4_sensor::numericSetting(config.lookup("slew_limit_nm_s"));
        result.rankTolerance = link4_sensor::numericSetting(config.lookup("relative_rank_tolerance"));
        if (config.exists("damping")) result.damping = link4_sensor::numericSetting(config.lookup("damping"));
        result.validate();
        return result;
    } catch (const libconfig::ParseException& e) {
        throw std::runtime_error(path + ":" + std::to_string(e.getLine()) + ": " + e.getError());
    } catch (const libconfig::ConfigException&) {
        throw std::runtime_error("Cannot read wrench-sum settings from " + path);
    }
}
} // namespace link4_wrench_sum
