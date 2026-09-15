#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <libconfig.h++>
#include <cmath>
#include <stdexcept>
#include <string>

namespace link4_sensor {
struct Mounting {
    std::string name;
    Eigen::Vector3d position;
    Eigen::Matrix3d rotation;
};

inline double numericSetting(const libconfig::Setting& setting) {
    if (setting.getType() == libconfig::Setting::TypeInt)
        return static_cast<int>(setting);
    if (setting.getType() == libconfig::Setting::TypeFloat)
        return static_cast<double>(setting);
    throw std::runtime_error("Mounting entries must be numbers");
}

inline Mounting loadMounting(const std::string& path) {
    try {
        libconfig::Config config;
        config.readFile(path.c_str());
        // Preserve compatibility with the original single-mounting file.
        const libconfig::Setting* selected = &config.getRoot();
        std::string name = "default";
        if (config.exists("active_mounting") || config.exists("mountings")) {
            name = static_cast<const char*>(config.lookup("active_mounting"));
            const libconfig::Setting& profiles = config.lookup("mountings");
            if (!profiles.isGroup() || name.empty() || !profiles.exists(name.c_str()))
                throw std::runtime_error("Unknown active_mounting: " + name);
            selected = &profiles[name.c_str()];
        }
        const libconfig::Setting& p = (*selected)["position_m"];
        const libconfig::Setting& r = (*selected)["rotation"];
        if (!(p.isArray() || p.isList()) || p.getLength() != 3 ||
            !(r.isArray() || r.isList()) || r.getLength() != 3)
            throw std::runtime_error("Expected position_m[3] and rotation[3][3]");
        Mounting mounting;
        mounting.name = name;
        for (int i = 0; i < 3; ++i) {
            mounting.position[i] = numericSetting(p[i]);
            if (!(r[i].isArray() || r[i].isList()) || r[i].getLength() != 3)
                throw std::runtime_error("Each rotation row must contain three numbers");
            for (int j = 0; j < 3; ++j)
                mounting.rotation(i,j) = numericSetting(r[i][j]);
        }
        if (!mounting.position.allFinite() || !mounting.rotation.allFinite() ||
            (mounting.rotation.transpose() * mounting.rotation - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
            std::abs(mounting.rotation.determinant() - 1.0) > 1e-6)
            throw std::runtime_error("Expected finite position and a proper orthonormal rotation (determinant +1)");
        return mounting;
    } catch (const libconfig::ParseException& e) {
        throw std::runtime_error(path + ":" + std::to_string(e.getLine()) + ": " + e.getError());
    } catch (const libconfig::ConfigException&) {
        throw std::runtime_error("Cannot read sensor mounting settings from " + path);
    } catch (const std::runtime_error& e) {
        throw std::runtime_error(path + ": " + e.what());
    }
}
} // namespace link4_sensor
