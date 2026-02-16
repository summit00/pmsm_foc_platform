#include <algorithm>

class PIController {
public:
    PIController(float kp, float ki) : kp_(kp), ki_(ki) {}

    float compute(float setpoint, float measured, float min, float max) {
        float error = setpoint - measured;
        float prop = kp_ * error;

        float potential_integral = integral_ + (ki_ * error);
        
        float total_output = prop + potential_integral;

        if (total_output >= min && total_output <= max) {
            integral_ = potential_integral;
        }

        return std::clamp(prop + integral_, min, max);
    }

    void setGains(float kp, float ki) {
        kp_ = kp; ki_ = ki;
    }

    void getGains(float& kp, float& ki) const {
        kp = kp_; ki = ki_;
    }

    void reset() {
        integral_ = 0.0f;
    }

private:
    float kp_, ki_;
    float integral_ = 0.0f;
};