#include "link4_sensor_config.hpp"
#include <cassert>
#include <fstream>
#include <cstdio>
#include <unistd.h>

int main() {
    char path[] = "/tmp/link4-config-test-XXXXXX";
    int fd = mkstemp(path);
    assert(fd >= 0);
    close(fd);
    auto check = [&](const char* contents, bool valid) {
        { std::ofstream file(path); file << contents; }
        bool accepted = false;
        try { link4_sensor::loadMounting(path); accepted = true; }
        catch (const std::runtime_error&) {}
        assert(accepted == valid);
    };
    check("position_m=[1,2,3]; rotation=([1,0,0],[0,1,0],[0,0,1]);", true);
    const auto mounting = link4_sensor::loadMounting(path);
    assert((mounting.position-Eigen::Vector3d(1,2,3)).norm()==0);
    check("position_m=[1,2]; rotation=([1,0,0],[0,1,0],[0,0,1]);", false);
    check("position_m=[1,2,3]; rotation=([1,0,0],[0,1,0],[0,0,-1]);", false);
    check("position_m=[1,2,3]; rotation=([2,0,0],[0,1,0],[0,0,1]);", false);
    check("position_m=[1,2,3];", false);
    check("position_m = broken syntax", false);
    const std::string profiles = "mountings={ a={position_m=[1,2,3]; rotation=([1,0,0],[0,1,0],[0,0,1]);}; b={position_m=[4,5,6]; rotation=([0,1,0],[0,0,1],[1,0,0]);}; };";
    check(("active_mounting=\"a\";" + profiles).c_str(), true);
    assert(link4_sensor::loadMounting(path).position[0] == 1);
    check(("active_mounting=\"b\";" + profiles).c_str(), true);
    const auto b = link4_sensor::loadMounting(path);
    assert(b.name == "b" && b.position[0] == 4 && b.rotation(0,1) == 1);
    check(("active_mounting=\"missing\";" + profiles).c_str(), false);
    check(profiles.c_str(), false);
    std::remove(path);
    bool rejected = false;
    try { link4_sensor::loadMounting(path); }
    catch (const std::runtime_error&) { rejected = true; }
    assert(rejected);
}
