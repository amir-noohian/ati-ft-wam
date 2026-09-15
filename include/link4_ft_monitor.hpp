#pragma once

#include <barrett/systems.h>
#include "link4_sensor_kinematics.hpp"

// Base-frame diagnostic outputs for libbarrett PrintToStream.
template<size_t DOF>
class Link4FTMonitor : public barrett::systems::System {
public:
    typedef typename barrett::math::Vector<6>::type ft_type;
    Input<ft_type> ftInput;
    Input<barrett::math::Kinematics<DOF> > kinInput;

    typedef typename barrett::math::Vector<3>::type position_type;
    Output<position_type> positionOutput;
    Output<ft_type> baseFTOutput;

    explicit Link4FTMonitor(const link4_sensor::Mounting& mounting)
        : System("Link4FTMonitor"), ftInput(this), kinInput(this),
          positionOutput(this, &positionValue_), baseFTOutput(this, &ftValue_), mounting_(mounting) {}
    ~Link4FTMonitor() { this->mandatoryCleanUp(); }

protected:
    void operate() {
        const bt_kinematics& kin = *kinInput.getValue().impl;
        Eigen::Matrix3d worldSensor, worldBase;
        Eigen::Vector3d worldPosition;
        Eigen::Matrix<double, 6, DOF> jacobian;
        link4_sensor::evaluate<DOF>(kin, mounting_, worldSensor, worldPosition, jacobian);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                worldBase(i,j) = gsl_matrix_get(kin.base->rot_to_world, i, j);
        const Eigen::Matrix3d baseSensor = worldBase.transpose() * worldSensor;
        const Eigen::Vector3d p = worldBase.transpose() *
            (worldPosition - link4_sensor::position(*kin.base));
        const ft_type& ft = ftInput.getValue();
        const Eigen::Vector3d f = baseSensor * ft.template head<3>();
        // Rotate moments only: reference point remains the SENSOR origin.
        const Eigen::Vector3d m = baseSensor * ft.template tail<3>();
        position_ = p;
        baseFT_.template head<3>() = f;
        baseFT_.template tail<3>() = m;
        positionValue_->setData(&position_);
        ftValue_->setData(&baseFT_);
    }

private:
    typename Output<position_type>::Value* positionValue_;
    typename Output<ft_type>::Value* ftValue_;
    const link4_sensor::Mounting mounting_;
    position_type position_;
    ft_type baseFT_;
};
