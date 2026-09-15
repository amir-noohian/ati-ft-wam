#include "link4_sensor_kinematics.hpp"
#include <libconfig.h>
#include <cassert>
#include <iostream>

int main() {
    const auto mounting = link4_sensor::loadMounting("config/link4_sensor.conf");
    config_t config;
    config_init(&config);
    assert(config_read_string(&config,
        "kinematics: { moving = ("
        "{alpha_pi=-0.5; a=0.0; d=0.0;},"
        "{alpha_pi=0.5; a=0.0; d=0.0;},"
        "{alpha_pi=-0.5; a=0.045; d=0.55;},"
        "{alpha_pi=0.5; a=-0.045; d=0.0;},"
        "{alpha_pi=-0.5; a=0.0; d=0.3;},"
        "{alpha_pi=0.5; a=0.0; d=0.0;},"
        "{alpha_pi=-0.5; a=0.0; d=0.0609;});"
        "toolplate={alpha_pi=0.5; theta_pi=0.0; a=0.0; d=0.0;};};"));
    bt_kinematics* kin = NULL;
    assert(bt_kinematics_create(&kin, config_lookup(&config, "kinematics"), 7) == 0);
    gsl_vector* q = gsl_vector_calloc(7);
    gsl_vector* v = gsl_vector_calloc(7);
    Eigen::Matrix3d r, rp, rm;
    Eigen::Vector3d p, pp, pm;
    Eigen::Matrix<double,6,7> j, unused;
    assert((mounting.rotation * Eigen::Vector3d::UnitX()
            - Eigen::Vector3d::UnitZ()).norm() == 0);
    assert((mounting.rotation * Eigen::Vector3d::UnitY()
            - Eigen::Vector3d::UnitX()).norm() == 0);
    assert((mounting.rotation * Eigen::Vector3d::UnitZ()
            - Eigen::Vector3d::UnitY()).norm() == 0);
    for (int pose = 0; pose < 10; ++pose) {
        for (int k=0; k<7; ++k) gsl_vector_set(q,k,0.15 * pose - 0.2*k);
        bt_kinematics_eval(kin,q,v);
        link4_sensor::evaluate<7>(*kin,mounting,r,p,j);
        assert(j.rightCols<3>().norm() == 0);
        for (int k=0; k<7; ++k) {
            const double value = gsl_vector_get(q,k), h=1e-6;
            gsl_vector_set(q,k,value+h);
            bt_kinematics_eval(kin,q,v);
            link4_sensor::evaluate<7>(*kin,mounting,rp,pp,unused);
            gsl_vector_set(q,k,value-h);
            bt_kinematics_eval(kin,q,v);
            link4_sensor::evaluate<7>(*kin,mounting,rm,pm,unused);
            gsl_vector_set(q,k,value);
            assert(((pp-pm)/(2*h) - j.block<3,1>(0,k)).norm() < 1e-8);
            Eigen::Matrix3d skew = ((rp-rm)/(2*h))*r.transpose();
            Eigen::Vector3d omega(skew(2,1),skew(0,2),skew(1,0));
            assert((omega-j.block<3,1>(3,k)).norm() < 1e-8);
        }
    }
    gsl_vector_free(q);
    gsl_vector_free(v);
    bt_kinematics_destroy(kin);
    config_destroy(&config);
    std::cout << "Passed: mounting axis, sensor position/angular finite differences at 10 poses, zero wrist columns.\n";
}
