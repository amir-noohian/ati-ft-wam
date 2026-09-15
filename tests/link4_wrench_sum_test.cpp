#include "link4_wrench_sum_config.hpp"
#include "link4_sensor_kinematics.hpp"
#include <cassert>
#include <fstream>
#include <sstream>
#include <iostream>
#include <limits>

using namespace link4_wrench_sum;
int main() {
    Settings settings=loadSettings("config/link4_wrench_sum.conf");
    Jacobian sensor=Jacobian::Zero(), tool=Jacobian::Zero();
    sensor(0,0)=1;
    tool.leftCols<6>().setIdentity(); tool.col(6).setOnes();
    Wrench wrench=Wrench::Zero(); wrench[0]=1;
    Mapping m;
    assert(mapAndSum(sensor,tool,wrench,1,1e-6,m));
    assert(m.initial[0]==1 && m.initial.tail<3>().norm()==0);
    Wrench expected=Wrench::Constant(-1.0/7); expected[0]+=1;
    assert((m.endEffectorWrench-expected).norm()<1e-12);
    assert((m.returned-tool.transpose()*expected).norm()<1e-12);
    assert((m.sum-m.initial+m.returned).norm()<1e-12);
    assert(m.sum.tail<3>().norm()>0); // Wrist zero only on initial torque.
    assert((tool*(m.initial-m.returned)).norm()<1e-12); // LS normal equations.
    assert((tool*m.sum).norm()<1e-12); // Unbounded residual is a regular null torque.
    // If initial torque is in the end-effector wrench range, subtraction cancels it.
    tool.col(6).setZero();
    assert(mapAndSum(sensor,tool,wrench,1,1e-6,m));
    assert(m.sum.norm()<1e-12);
    // A torque in the null space survives unchanged; returned wrench is zero.
    tool.setZero(); tool.rightCols<6>().setIdentity();
    assert(mapAndSum(sensor,tool,wrench,1,1e-6,m));
    assert(m.endEffectorWrench.norm()<1e-12 && (m.sum-m.initial).norm()<1e-12);
    tool.setZero();
    assert(!mapAndSum(sensor,tool,wrench,1,1e-6,m));
    tool.leftCols<6>().setIdentity(); tool.row(5)*=1e-9;
    assert(!mapAndSum(sensor,tool,wrench,1,1e-6,m));
    tool.leftCols<6>().setIdentity();
    // Regression: common-scalar limiting used to fault on a valid direction change.
    Vector7 previous=Vector7::Zero(), desired=Vector7::Zero(), limited;
    previous[0]=1; desired[1]=1;
    assert(limit(desired,previous,4,0.01,limited));
    assert(std::abs(limited[0]-0.99)<1e-12);
    assert(std::abs(limited[1]-0.01)<1e-12);
    // Reversals, release, and saturation all respect both limits.
    for (int k=0;k<1000;++k) {
        for (int i=0;i<7;++i) desired[i] = (k/100)%3 == 0 ? 20 : ((k/100)%3 == 1 ? -20 : 0);
        assert(limit(desired,previous,4,0.01,limited));
        assert(limited.cwiseAbs().maxCoeff()<=4+1e-12);
        assert((limited-previous).cwiseAbs().maxCoeff()<=0.01+1e-12);
        previous=limited;
    }
    tool.col(6).setOnes(); // Give sensor torques a nonzero null component.
    Controller controller(settings,0.002);
    Vector7 last=Vector7::Zero();
    wrench[0]=10;
    for (int k=0;k<1000;++k) {
        const Vector7 output=controller.step(sensor,tool,wrench);
        assert(controller.fault()==0);
        assert(output.cwiseAbs().maxCoeff()<=4+1e-12);
        assert((output-last).cwiseAbs().maxCoeff()<=0.01+1e-12);
        last=output;
    }
    controller.reset(); wrench[0]=1e-4;
    assert(controller.step(sensor,tool,wrench).norm()>0); // No deadband.
    controller.reset(); sensor.setZero(); sensor(5,3)=1;
    wrench.setZero(); wrench[5]=0.01;
    assert(controller.step(sensor,tool,wrench).norm()>0); // Moment input retained.
    controller.reset();
    assert(controller.step(sensor,Jacobian::Zero(),wrench).norm()==0 && controller.fault()==2);
    assert(controller.step(sensor,tool,wrench).norm()==0); // Latched fault.
    controller.reset(); wrench[0]=std::numeric_limits<double>::quiet_NaN();
    assert(controller.step(sensor,tool,wrench).norm()==0 && controller.fault()==1);

    // Check least-squares remapping with actual WAM and sensor Jacobians.
    libconfig::Config config;
    std::ifstream file("config/zeus/wam7w.conf"); assert(file.good());
    std::ostringstream text; std::string line;
    while(std::getline(file,line)) if(line.find("@include")==std::string::npos) text<<line<<'\n';
    config.readString(text.str().c_str());
    bt_kinematics* kin=nullptr;
    assert(bt_kinematics_create(&kin,config.lookup("wam7w.kinematics").getCSetting(),7)==0);
    gsl_vector* q=gsl_vector_calloc(7); gsl_vector* v=gsl_vector_calloc(7);
    const auto mounting=link4_sensor::loadMounting("config/link4_sensor.conf");
    for(int k=0;k<100;++k) {
        for(int i=0;i<7;++i) gsl_vector_set(q,i,0.3-0.15*i+0.0001*k);
        bt_kinematics_eval(kin,q,v);
        Eigen::Matrix3d r; Eigen::Vector3d p;
        link4_sensor::evaluate<7>(*kin,mounting,r,p,sensor);
        for(int i=0;i<6;++i) for(int j=0;j<7;++j) tool(i,j)=gsl_matrix_get(kin->tool_jacobian,i,j);
        wrench<<0.1,0.2,0.3,0.01,0.02,0.03;
        assert(mapAndSum(sensor,tool,wrench,1,1e-6,m));
        assert(m.initial.tail<3>().norm()==0);
        assert((tool*(m.initial-m.returned)).norm()<1e-10);
        assert((m.sum-m.initial+m.returned).norm()<1e-12);
        assert((tool*m.sum).norm()<1e-10);
    }
    gsl_vector_free(q); gsl_vector_free(v); bt_kinematics_destroy(kin);
    std::cout<<"Passed: seven-equation least squares, subtraction, wrist terms, range/null cases, limits, full wrench, faults, 100 WAM configurations.\n";
}
