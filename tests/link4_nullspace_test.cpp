#include "link4_nullspace_config.hpp"
#include "link4_sensor_kinematics.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <chrono>
#include <fstream>
#include <sstream>

using namespace link4_nullspace;
int main() {
    Settings settings = loadSettings("config/link4_nullspace.conf");
    Jacobian j = Jacobian::Zero();
    j.leftCols<6>().setIdentity();
    j.col(6).setOnes();
    Matrix7 n;
    for (bool positionOnly : {false,true}) {
        assert(projector(j,positionOnly,1e-6,n));
        Jacobian task = j;
        if (positionOnly) task.bottomRows<3>().setZero();
        assert((task*n).norm()<1e-12);
        assert((n*n-n).norm()<1e-12);
        assert((n-n.transpose()).norm()<1e-12);
        assert(std::abs(n.trace()-(positionOnly ? 4 : 1))<1e-12);
    }
    assert(projector(j,false,1e-6,n));
    // Confirm this is the regular projector, not a dynamics-weighted projector.
    Matrix7 inverseMass = Matrix7::Identity(); inverseMass(0,0)=2;
    assert((j*inverseMass*n).norm()>0.1);
    Jacobian singular=j; singular.row(5).setZero();
    assert(!projector(singular,false,1e-6,n));
    assert(projector(singular,true,1e-6,n));
    singular=j; singular.row(5)*=1e-9;
    assert(!projector(singular,false,1e-6,n));
    assert(!projector(Jacobian::Zero(),true,1e-6,n));

    Vector7 desired=Vector7::Constant(10), out;
    assert(limit(desired,Vector7::Zero(),4,100,out));
    assert(out.maxCoeff()==4);
    assert(limit(desired,Vector7::Zero(),4,0.01,out));
    assert(std::abs(out.maxCoeff()-0.01)<1e-12);
    // Cannot abruptly rotate the torque vector and preserve a small slew bound.
    Vector7 previous=Vector7::Zero(); previous[0]=1;
    desired.setZero(); desired[1]=1;
    assert(!limit(desired,previous,4,0.01,out));
    assert(out.norm()==0);

    settings.positionOnly=false;
    Controller controller(settings,0.002);
    Jacobian sensor=Jacobian::Zero(); sensor(0,0)=1;
    Wrench wrench=Wrench::Zero(); wrench[0]=100;
    Vector7 last=Vector7::Zero();
    const auto start=std::chrono::steady_clock::now();
    for (int i=0;i<1000;++i) {
        const Vector7 torque=controller.step(sensor,j,wrench);
        assert(controller.fault()==0);
        assert((j*torque).norm()<1e-11);
        assert(torque.cwiseAbs().maxCoeff()<=4+1e-12);
        assert((torque-last).cwiseAbs().maxCoeff()<=0.01+1e-12);
        last=torque;
    }
    assert(last.tail<3>().norm()>0); // Do not zero wrist columns after projection.
    const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    controller.reset();
    wrench[0]=0.001;
    assert(controller.step(sensor,j,wrench).norm()>0); // No deadband.
    controller.reset();
    sensor.setZero(); sensor(5,3)=1;
    wrench.setZero(); wrench[5]=0.01;
    assert(controller.step(sensor,j,wrench).norm()>0); // Moment assistance too.
    controller.reset();
    assert(controller.step(sensor,Jacobian::Zero(),wrench).norm()==0);
    assert(controller.fault()==2);
    assert(controller.step(sensor,j,wrench).norm()==0); // Fault remains latched.
    controller.reset();
    wrench[0]=std::numeric_limits<double>::quiet_NaN();
    assert(controller.step(sensor,j,wrench).norm()==0 && controller.fault()==1);
    Settings invalid=settings; invalid.torqueLimit=-1;
    bool rejected=false;
    try { Controller bad(invalid,0.002); } catch(const std::runtime_error&) { rejected=true; }
    assert(rejected);
    // Integration with the repository's WAM kinematic configuration.
    libconfig::Config robot;
    // Calibration includes are unrelated to kinematics; omit them so this also
    // works with the older libconfig shipped with libbarrett.
    std::ifstream robotFile("config/zeus/wam7w.conf");
    assert(robotFile.good());
    std::ostringstream robotText;
    std::string line;
    while (std::getline(robotFile,line))
        if (line.find("@include") == std::string::npos) robotText << line << '\n';
    robot.readString(robotText.str().c_str());
    bt_kinematics* kin = nullptr;
    assert(bt_kinematics_create(&kin, robot.lookup("wam7w.kinematics").getCSetting(), 7)==0);
    gsl_vector* q = gsl_vector_calloc(7);
    gsl_vector* velocity = gsl_vector_calloc(7);
    const auto mounting = link4_sensor::loadMounting("config/link4_sensor.conf");
    for (bool positionOnly : {false,true}) {
        settings.positionOnly=positionOnly;
        Controller moving(settings,0.002);
        Vector7 previous=Vector7::Zero();
        int valid=0;
        for (int sample=0;sample<100;++sample) {
            for (int k=0;k<7;++k) gsl_vector_set(q,k,0.3-0.15*k+0.0001*sample);
            bt_kinematics_eval(kin,q,velocity);
            Eigen::Matrix3d rotation;
            Eigen::Vector3d position;
            Jacobian sensorJac, toolJac;
            link4_sensor::evaluate<7>(*kin,mounting,rotation,position,sensorJac);
            for (int row=0;row<6;++row)
                for (int col=0;col<7;++col)
                    toolJac(row,col)=gsl_matrix_get(kin->tool_jacobian,row,col);
            wrench << 0.1,0.2,0.1,0.01,0.02,0.01;
            const Vector7 result=moving.step(sensorJac,toolJac,wrench);
            assert(moving.fault()==0);
            if (positionOnly) toolJac.bottomRows<3>().setZero();
            assert((toolJac*result).norm()<1e-10);
            assert((result-previous).cwiseAbs().maxCoeff()<=0.01+1e-12);
            previous=result;
            ++valid;
        }
        assert(valid==100 && previous.norm()>0);
    }
    gsl_vector_free(q); gsl_vector_free(velocity); bt_kinematics_destroy(kin);
    std::cout << "Passed: regular projector, position/pose ranks, singularity rejection, null-preserving limits, full wrench, wrist output, fault/reset, and 100 moving WAM poses per task.\n"
              << "1000 synthetic controller steps: " << seconds << " seconds (not a real-time guarantee).\n";
}
