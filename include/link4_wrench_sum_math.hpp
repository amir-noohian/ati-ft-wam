#pragma once
#include <Eigen/Core>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace link4_wrench_sum {
using Vector7 = Eigen::Matrix<double,7,1>;
using Wrench = Eigen::Matrix<double,6,1>;
using Jacobian = Eigen::Matrix<double,6,7>;
using Matrix7 = Eigen::Matrix<double,7,7>;

struct Settings {
    double gain = 1.0, sign = 1.0, filterHz = 10.0;
    double torqueLimit = 4.0, slewLimit = 5.0, rankTolerance = 1e-6, damping = 1e-3;
    void validate() const {
        if (!std::isfinite(gain) || gain < 0 || (sign != 1 && sign != -1) ||
            !std::isfinite(filterHz) || filterHz <= 0 ||
            !std::isfinite(torqueLimit) || torqueLimit <= 0 ||
            !std::isfinite(slewLimit) || slewLimit <= 0 ||
            !std::isfinite(rankTolerance) || rankTolerance <= 0 || rankTolerance >= 1 ||
            !std::isfinite(damping) || damping < 0)
            throw std::runtime_error("Invalid wrench-sum assistance settings");
    }
};

struct Mapping {
    Vector7 initial;
    Wrench endEffectorWrench;
    Vector7 returned;
    Vector7 sum;
};

// Solve J_ee^T W_ee ~= tau_initial in least squares, using all seven
// equations, including tau_initial[4:7] = 0. J_ee is 6x7, so it has no
// ordinary inverse. (J_ee^T)^+ = U Sigma^-1 V^T for J_ee = U Sigma V^T.
inline bool mapAndSum(const Jacobian& sensorJ, const Jacobian& toolJ,
                      const Wrench& wrench, double gain, double tolerance, double damping,
                      Mapping& result) {
    result.initial.setZero(); result.endEffectorWrench.setZero();
    result.returned.setZero(); result.sum.setZero();
    if (!sensorJ.allFinite() || !toolJ.allFinite() || !wrench.allFinite()) return false;
    result.initial = gain * sensorJ.transpose() * wrench;
    result.initial.tail<3>().setZero();
    Eigen::JacobiSVD<Jacobian> svd(toolJ, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& sigma = svd.singularValues();
    if (sigma[0] <= 0) return false;
    const double lambda = std::max(damping, tolerance * sigma[0]);
    const Wrench coefficients = svd.matrixV().leftCols<6>().transpose() * result.initial;
    Wrench dampedCoefficients = coefficients;
    for (int i = 0; i < 6; ++i) {
        const double denom = sigma[i] * sigma[i] + lambda * lambda;
        dampedCoefficients[i] = coefficients[i] * sigma[i] / denom;
    }
    result.endEffectorWrench = svd.matrixU() * dampedCoefficients;
    result.returned = toolJ.transpose() * result.endEffectorWrench;
    const Vector7 firstTorque = result.initial;
    const Vector7 secondTorque = result.returned;
    result.sum = firstTorque - secondTorque; // Subtract the second torque from the first one.
    return result.initial.allFinite() && result.endEffectorWrench.allFinite() &&
           result.returned.allFinite() && result.sum.allFinite();
}

// Retain per-joint limits to avoid scalar-limiter shutdowns on direction changes.
// These limits can disturb the raw residual's null-space property when active.
inline bool limit(const Vector7& desired, const Vector7& previous,
                  double torqueLimit, double step, Vector7& result) {
    result.setZero();
    if (!desired.allFinite() || !previous.allFinite() ||
        !std::isfinite(torqueLimit) || torqueLimit <= 0 ||
        !std::isfinite(step) || step <= 0) return false;
    for (int i = 0; i < 7; ++i) {
        const double target = std::max(-torqueLimit, std::min(torqueLimit, desired[i]));
        const double change = std::max(-step, std::min(step, target - previous[i]));
        result[i] = previous[i] + change;
    }
    return result.allFinite();
}

class Controller {
public:
    // Latched faults: 1=invalid input, 2=task singularity, 3=invalid limiter input/output.
    Controller(const Settings& settings, double period) : settings_(settings), period_(period) {
        settings_.validate();
        if (!std::isfinite(period) || period <= 0) throw std::runtime_error("Invalid period");
        alpha_ = 1.0 - std::exp(-2.0 * 3.141592653589793 * settings.filterHz * period);
        reset();
    }
    void reset() { filtered_.setZero(); output_.setZero(); fault_ = 0; }
    int fault() const { return fault_; }
    const Vector7& step(const Jacobian& sensorJ, const Jacobian& toolJ, const Wrench& worldWrench) {
        if (fault_) return output_;
        if (!sensorJ.allFinite() || !worldWrench.allFinite()) return fail(1);
        if (!toolJ.allFinite()) return fail(1);
        filtered_ += alpha_ * (settings_.sign * worldWrench - filtered_);
        Mapping mapping;
        if (!mapAndSum(sensorJ, toolJ, filtered_, settings_.gain, settings_.rankTolerance,
                       settings_.damping, mapping))
            return fail(2);
        const Vector7& desired = mapping.sum;
        Vector7 limited;
        if (!limit(desired, output_, settings_.torqueLimit, settings_.slewLimit * period_, limited))
            return fail(3);
        output_ = limited;
        return output_;
    }
private:
    const Vector7& fail(int code) { fault_ = code; filtered_.setZero(); output_.setZero(); return output_; }
    Settings settings_;
    double period_, alpha_;
    Wrench filtered_;
    Vector7 output_;
    int fault_;
};
} // namespace link4_wrench_sum
