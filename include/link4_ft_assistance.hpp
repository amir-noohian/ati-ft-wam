#pragma once

#include <barrett/systems.h>
#include <barrett/units.h>
#include <algorithm>
#include <cmath>
#include "link4_sensor_kinematics.hpp"
#include "link4_wrench.hpp"

// Force and moment hand guiding. Input is tared sensor force in N, moment in N m.
// Positive sign means the sensor reports force applied TO the robot.
template<size_t DOF>
class Link4FTAssistance : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
public:
    typedef typename barrett::math::Vector<6>::type ft_type;
    Input<ft_type> ftInput;
    Input<barrett::math::Kinematics<DOF> > kinInput;
    Output<jt_type> output;

    explicit Link4FTAssistance(const link4_sensor::Mounting& mounting, double period, double sign = 1.0)
        : System("Link4FTAssistance"), ftInput(this), kinInput(this),
          output(this, &outputValue_), mounting_(mounting), sign_(sign),
          filterAlpha_(1.0 - std::exp(-2.0 * 3.141592653589793 * 10.0 * period)),
          maxStep_(5.0 * period) {
        filtered_.setZero();
        torque_.setZero();
    }
    ~Link4FTAssistance() { this->mandatoryCleanUp(); }

    // Call only while disconnected from the WAM (this system is demand-driven).
    void reset() {
        filtered_.setZero();
        torque_.setZero();
    }

protected:
    void operate() {
        const ft_type& ft = ftInput.getValue();
        Eigen::Matrix3d rotation;
        Eigen::Vector3d point;
        Eigen::Matrix<double, 6, DOF> jacobian;
        link4_sensor::evaluate<DOF>(*kinInput.getValue().impl, mounting_, rotation, point, jacobian);
        if (!ft.allFinite() || !rotation.allFinite() || !jacobian.allFinite()) {
            filtered_.setZero();
            torque_.setZero();
            outputValue_->setData(&torque_);
            return;
        }
        const Eigen::Matrix<double, 6, 1> wrench =
            link4_sensor::rotateWrench(rotation, ft, sign_);
        filtered_ += filterAlpha_ * (wrench - filtered_);
        // Preserve the current gain; force and moment contributions are summed
        // before the joint torque and slew limits below.
        const Eigen::Matrix<double, DOF, 1> requested =
            1.0 * jacobian.transpose() * filtered_;
        for (size_t j = 0; j < DOF; ++j) {
            const double target = std::max(-4.0, std::min(4.0, requested[j]));
            torque_[j] += std::max(-maxStep_, std::min(maxStep_, target - torque_[j]));
            if (j >= 4) torque_[j] = 0.0;
        }
        outputValue_->setData(&torque_);
    }
private:
    typename Output<jt_type>::Value* outputValue_;
    const link4_sensor::Mounting mounting_;
    double sign_, filterAlpha_, maxStep_;
    Eigen::Matrix<double, 6, 1> filtered_;
    jt_type torque_;
};
