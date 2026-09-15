#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <iostream>
#include <string>
#include "atift_system.hpp"
#include "link4_nullspace_assistance.hpp"
#include "link4_nullspace_config.hpp"
#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include "link4_ft_monitor.hpp"

template<size_t DOF>
int wam_main(int, char**, barrett::ProductManager&, barrett::systems::Wam<DOF>&) {
    std::cerr << "Null-space assistance requires a 7-DOF WAM.\n";
    return 1;
}

template<>
int wam_main<7>(int argc, char** argv, barrett::ProductManager& pm,
               barrett::systems::Wam<7>& wam) {
    const size_t DOF = 7;
    const std::string calibration = "/home/hela/Desktop/ft_libbarrett/cal/FT9236/FT9236.cal";
    const link4_sensor::Mounting mounting = link4_sensor::loadMounting(LINK4_SENSOR_CONFIG_PATH);
    std::cout << "Sensor mounting configuration: " << LINK4_SENSOR_CONFIG_PATH << " (" << mounting.name << ")\n";
    const auto settings = link4_nullspace::loadSettings(LINK4_NULLSPACE_CONFIG_PATH);
    wam.gravityCompensate();
    std::cout << "Keep the sensor unloaded and still during startup tare.\n";
    ATIFTSystem sensor(pm.getExecutionManager(), calibration, "Dev2/ai16:21");
    Link4NullspaceAssistance assistance(mounting, settings, pm.getExecutionManager()->getPeriod());
    barrett::systems::connect(sensor.ftOutput, assistance.ftInput);
    barrett::systems::connect(wam.kinematicsBase.kinOutput, assistance.kinInput);

    Link4FTMonitor<DOF> monitor(mounting);
    barrett::systems::PrintToStream<typename Link4FTMonitor<DOF>::position_type> printPosition(
        pm.getExecutionManager(), "Sensor position in base [m] (x y z): ");
    barrett::systems::PrintToStream<typename Link4FTMonitor<DOF>::ft_type> printBaseFT(
        pm.getExecutionManager(), "Sensor FT in base axes [N, Nm] (Fx Fy Fz Mx My Mz), moment at sensor: ");
    barrett::systems::connect(monitor.positionOutput, printPosition.input);
    barrett::systems::connect(monitor.baseFTOutput, printBaseFT.input);
    barrett::systems::connect(sensor.ftOutput, monitor.ftInput);
    barrett::systems::connect(wam.kinematicsBase.kinOutput, monitor.kinInput);

    std::cout << "Regular-Jacobian null-space assistance: task="
              << (settings.positionOnly ? "position" : "pose")
              << ", gain=" << settings.gain << ", limit=" << settings.torqueLimit << " Nm/joint.\n"
              << "No end-effector holding target. Assistance starts disabled.\n"
              << "f + Enter: toggle assistance; q + Enter: quit.\n";
    bool enabled = false;
    while (true) {
        if (enabled && assistance.fault()) {
            wam.idle();
            enabled = false;
            std::cout << "Null-space assistance DISABLED: fault " << assistance.fault()
                      << " (1=invalid input, 2=task singularity, 3=no feasible scalar slew limit).\n"
                      << "Gravity compensation remains active. Press f to reset/re-enable.\n";
        }
        pollfd input = {STDIN_FILENO, POLLIN, 0};
        const int ready = poll(&input, 1, 250);
        if (ready < 0) { if (errno == EINTR) continue; break; }
        if (ready == 0) continue;
        if (!(input.revents & POLLIN)) break;
        std::string command;
        if (!std::getline(std::cin, command)) break;
        if (command == "q") break;
        if (command != "f") continue;
        if (enabled) {
            wam.idle();
        } else {
            assistance.reset();
            wam.trackReferenceSignal(assistance.output);
        }
        enabled = !enabled;
        std::cout << "Null-space assistance " << (enabled ? "enabled" : "disabled") << ".\n";
    }
    wam.idle();
    wam.gravityCompensate();
    barrett::systems::disconnect(printPosition.input);
    barrett::systems::disconnect(printBaseFT.input);
    barrett::systems::disconnect(monitor.ftInput);
    barrett::systems::disconnect(monitor.kinInput);
    pm.getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE);
    return 0;
}

#include <barrett/standard_main_function.h>
