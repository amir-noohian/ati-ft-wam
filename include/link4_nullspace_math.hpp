#pragma once
#include <Eigen/Core>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace link4_nullspace {
using Vector7 = Eigen::Matrix<double,7,1>;
using Wrench = Eigen::Matrix<double,6,1>;
using Jacobian = Eigen::Matrix<double,6,7>;
using Matrix7 = Eigen::Matrix<double,7,7>;

struct Settings {
    bool positionOnly = false;
    double gain = 1.0, sign = 1.0, filterHz = 10.0;
    double torqueLimit = 4.0, slewLimit = 5.0, rankTolerance = 1e-6;
    void validate() const {
        if (!std::isfinite(gain) || gain < 0 || (sign != 1 && sign != -1) ||
            !std::isfinite(filterHz) || filterHz <= 0 ||
            !std::isfinite(torqueLimit) || torqueLimit <= 0 ||
            !std::isfinite(slewLimit) || slewLimit <= 0 ||
            !std::isfinite(rankTolerance) || rankTolerance <= 0 || rankTolerance >= 1)
            throw std::runtime_error("Invalid null-space assistance settings");
    }
};

// N = I - J^+ J = V_null V_null^T. Fixed-size SVD; no damped inverse.
inline bool projector(Jacobian task, bool positionOnly, double tolerance, Matrix7& n) {
    n.setZero();
    if (!task.allFinite()) return false;
    const int rank = positionOnly ? 3 : 6;
    if (positionOnly) task.bottomRows<3>().setZero();
    Eigen::JacobiSVD<Jacobian> svd(task, Eigen::ComputeFullV);
    const auto& singular = svd.singularValues();
    if (singular[0] <= 0 || singular[rank-1] <= tolerance * singular[0]) return false;
    for (int i = rank; i < 7; ++i)
        n.noalias() += svd.matrixV().col(i) * svd.matrixV().col(i).transpose();
    return n.allFinite();
}

// Use one scalar for all joints to retain membership in the current null space.
// Select the largest scale in [0,1] satisfying amplitude and per-cycle slew bounds.
// A changed null space/direction can leave no feasible scalar: caller faults to zero.
inline bool limit(const Vector7& desired, const Vector7& previous,
                  double torqueLimit, double step, Vector7& result) {
    result.setZero();
    if (!desired.allFinite() || !previous.allFinite()) return false;
    double lo = 0, hi = 1;
    for (int i = 0; i < 7; ++i) {
        const double lower = std::max(-torqueLimit, previous[i] - step);
        const double upper = std::min(torqueLimit, previous[i] + step);
        if (std::abs(desired[i]) < 1e-14) {
            if (lower > 0 || upper < 0) return false;
        } else {
            double a = lower / desired[i], b = upper / desired[i];
            if (a > b) std::swap(a,b);
            lo = std::max(lo,a);
            hi = std::min(hi,b);
        }
    }
    if (lo > hi) return false;
    result = hi * desired;
    return result.allFinite();
}

class Controller {
public:
    // Latched faults: 1=invalid input, 2=task singularity, 3=limiter infeasible.
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
        Matrix7 n;
        if (!toolJ.allFinite()) return fail(1);
        if (!projector(toolJ, settings_.positionOnly, settings_.rankTolerance, n)) return fail(2);
        filtered_ += alpha_ * (settings_.sign * worldWrench - filtered_);
        const Vector7 desired = n * (settings_.gain * sensorJ.transpose() * filtered_);
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
} // namespace link4_nullspace
