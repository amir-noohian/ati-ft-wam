#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <iostream>
#include <string>
#include "atift_system.hpp"
#include "link4_ft_assistance.hpp"
#include "link4_ft_monitor.hpp"

template<size_t DOF>
int wam_main(int argc, char** argv, barrett::ProductManager& pm,
             barrett::systems::Wam<DOF>& wam) {
    const std::string calibration = "/home/hela/Desktop/ft_libbarrett/cal/FT9236/FT9236.cal";
    const link4_sensor::Mounting mounting = link4_sensor::loadMounting(LINK4_SENSOR_CONFIG_PATH);
    std::cout << "Sensor mounting configuration: " << LINK4_SENSOR_CONFIG_PATH << " (" << mounting.name << ")\n";
    wam.gravityCompensate();
    std::cout << "Keep the sensor unloaded and still during startup tare.\n";
    ATIFTSystem sensor(pm.getExecutionManager(), calibration, "Dev2/ai16:21");
    // Change to -1 ONLY if the measured force has the opposite physical sign.
    Link4FTAssistance<DOF> assistance(mounting, pm.getExecutionManager()->getPeriod(), 1.0);
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

    std::cout << "Link-4 force + moment assistance ready (gain 1.0, limit 4 Nm/joint).\n"
              << "Gravity compensation only at startup; base-frame readings via PrintToStream.\n"
              << "f + Enter: toggle assistance; q + Enter: quit.\n";
    bool enabled = false;
    char command;
    while (std::cin >> command) {
        if (command == 'q') break;
        if (command != 'f') continue;
        if (enabled) {
            wam.idle();
            wam.gravityCompensate();
        } else {
            assistance.reset();
            wam.trackReferenceSignal(assistance.output);
        }
        enabled = !enabled;
        std::cout << "Assistance " << (enabled ? "enabled" : "disabled") << ".\n";
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

template<>
int wam_main<3>(int, char**, barrett::ProductManager&, barrett::systems::Wam<3>&) {
    std::cerr << "Link-4 assistance requires a 4- or 7-DOF WAM.\n";
    return 1;
}

#include <barrett/standard_main_function.h>
