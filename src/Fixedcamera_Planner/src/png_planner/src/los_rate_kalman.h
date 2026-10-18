#pragma once

#include <algorithm>

class ScalarRateKalmanFilter {
public:
    ScalarRateKalmanFilter() = default;

    void setNoise(double process_noise_position,
                  double process_noise_rate,
                  double measurement_noise) {
        process_noise_position_ = std::max(process_noise_position, 1e-9);
        process_noise_rate_ = std::max(process_noise_rate, 1e-9);
        measurement_noise_ = std::max(measurement_noise, 1e-9);
    }

    void reset() {
        initialized_ = false;
        value_ = 0.0;
        rate_ = 0.0;
        p00_ = 1.0;
        p01_ = 0.0;
        p10_ = 0.0;
        p11_ = 1.0;
    }

    void update(double measurement, double dt) {
        dt = std::max(dt, 1e-4);

        if (!initialized_) {
            initialized_ = true;
            value_ = measurement;
            rate_ = 0.0;
            p00_ = 1.0;
            p01_ = 0.0;
            p10_ = 0.0;
            p11_ = 1.0;
            return;
        }

        value_ += dt * rate_;

        const double predicted_p00 =
            p00_ + dt * (p10_ + p01_) + dt * dt * p11_ +
            process_noise_position_ * dt * dt;
        const double predicted_p01 =
            p01_ + dt * p11_;
        const double predicted_p10 =
            p10_ + dt * p11_;
        const double predicted_p11 =
            p11_ + process_noise_rate_ * dt;

        const double innovation = measurement - value_;
        const double innovation_cov = predicted_p00 + measurement_noise_;
        const double k0 = predicted_p00 / innovation_cov;
        const double k1 = predicted_p10 / innovation_cov;

        value_ += k0 * innovation;
        rate_ += k1 * innovation;

        p00_ = (1.0 - k0) * predicted_p00;
        p01_ = (1.0 - k0) * predicted_p01;
        p10_ = p01_;
        p11_ = predicted_p11 - k1 * predicted_p01;
    }

    double value() const { return value_; }
    double rate() const { return rate_; }

private:
    bool initialized_ = false;

    double process_noise_position_ = 1e-4;
    double process_noise_rate_ = 5e-1;
    double measurement_noise_ = 2e-4;

    double value_ = 0.0;
    double rate_ = 0.0;
    double p00_ = 1.0;
    double p01_ = 0.0;
    double p10_ = 0.0;
    double p11_ = 1.0;
};

struct LegacyLosKalmanTuning {
    double q_position;
    double q_rate;
    double r_measurement;
};

inline LegacyLosKalmanTuning mapLegacyLosChainToKalman(double alpha,
                                                       int window_size) {
    alpha = std::max(0.05, std::min(alpha, 0.95));
    const double window = std::max(2, window_size);

    // Empirical mapping anchored at the old default:
    // alpha=0.2, window=3 -> q_pos=1e-4, q_rate=5e-1, r=2e-4.
    // Stronger smoothing (smaller alpha / larger window) increases R and reduces Q.
    const double alpha_scale = alpha / 0.2;
    const double window_scale = 3.0 / window;

    LegacyLosKalmanTuning tuning;
    tuning.q_position = std::max(1e-6, 1e-4 * alpha_scale * window_scale);
    tuning.q_rate = std::max(1e-3, 5e-1 * alpha_scale * window_scale);
    tuning.r_measurement =
        std::max(1e-6, 2e-4 * (0.2 / alpha) * (0.2 / alpha) / window_scale);
    return tuning;
}
