#pragma once
#include <barrett/systems.h>
#include <atomic>
#include "link4_wrench_sum_math.hpp"
#include "link4_sensor_kinematics.hpp"
#include "link4_wrench.hpp"

class Link4WrenchSumAssistance : public barrett::systems::System {
public:
    typedef typename barrett::math::Vector<6>::type ft_type;
    typedef typename barrett::units::JointTorques<7>::type jt_type;
    Input<ft_type> ftInput;
    Input<barrett::math::Kinematics<7> > kinInput;
    Output<jt_type> output;
    Link4WrenchSumAssistance(const link4_sensor::Mounting& mounting,
                            const link4_wrench_sum::Settings& settings, double period)
        : System("Link4WrenchSumAssistance"), ftInput(this), kinInput(this),
          output(this, &outputValue_), mounting_(mounting), controller_(settings, period), fault_(0) {}
    ~Link4WrenchSumAssistance() { this->mandatoryCleanUp(); }
    // Demand-driven system: call only while its output is disconnected.
    void reset() { controller_.reset(); fault_.store(0); }
    int fault() const { return fault_.load(); }
protected:
    void operate() {
        const bt_kinematics& kin = *kinInput.getValue().impl;
        Eigen::Matrix3d rotation;
        Eigen::Vector3d position;
        link4_wrench_sum::Jacobian sensorJ, toolJ;
        link4_sensor::evaluate<7>(kin, mounting_, rotation, position, sensorJ);
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 7; ++j)
                toolJ(i,j) = gsl_matrix_get(kin.tool_jacobian, i, j);
        const auto wrench = link4_sensor::rotateWrench(rotation, ftInput.getValue(), 1.0);
        torque_ = controller_.step(sensorJ, toolJ, wrench);
        fault_.store(controller_.fault());
        outputValue_->setData(&torque_);
    }
private:
    Output<jt_type>::Value* outputValue_;
    link4_sensor::Mounting mounting_;
    link4_wrench_sum::Controller controller_;
    std::atomic<int> fault_;
    jt_type torque_;
};
