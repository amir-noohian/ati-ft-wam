#pragma once
#include "link4_nullspace_math.hpp"
#include "link4_sensor_config.hpp"

namespace link4_nullspace {
inline Settings loadSettings(const std::string& path) {
    try {
        libconfig::Config config;
        config.readFile(path.c_str());
        Settings result;
        const std::string task = static_cast<const char*>(config.lookup("task"));
        if (task != "pose" && task != "position") throw std::runtime_error("task must be pose or position");
        result.positionOnly = task == "position";
        result.gain = link4_sensor::numericSetting(config.lookup("gain"));
        result.sign = link4_sensor::numericSetting(config.lookup("sign"));
        result.filterHz = link4_sensor::numericSetting(config.lookup("filter_hz"));
        result.torqueLimit = link4_sensor::numericSetting(config.lookup("torque_limit_nm"));
        result.slewLimit = link4_sensor::numericSetting(config.lookup("slew_limit_nm_s"));
        result.rankTolerance = link4_sensor::numericSetting(config.lookup("relative_rank_tolerance"));
        result.validate();
        return result;
    } catch (const libconfig::ParseException& e) {
        throw std::runtime_error(path + ":" + std::to_string(e.getLine()) + ": " + e.getError());
    } catch (const libconfig::ConfigException&) {
        throw std::runtime_error("Cannot read null-space settings from " + path);
    }
}
} // namespace link4_nullspace
