#include <ControlRegulator.h>

ControlRegulator::ControlRegulator(double cycle, double min, double max)
    : cycle(cycle), minLimit(min), maxLimit(max) {}

ControlRegulator::~ControlRegulator() {}

void ControlRegulator::setGain(GainParams gain) {
    this->gain = gain;
}

void ControlRegulator::setLimits(double min, double max) {
    this->minLimit = min;
    this->maxLimit = max;
}

void ControlRegulator::setDerivativeFilter(double coeff) {
    this->lowPassFilterCoefficient = coeff;
}

double ControlRegulator::compute(double error) {
    prop = (error - pre_error);
    deriv = (prop - pre_prop);
    low_pass_deriv_ += deriv / lowPassFilterCoefficient;

    output += gain.Kp * prop + gain.Ki * error * cycle + gain.Kd * low_pass_deriv_;

    if (output > maxLimit || output < minLimit) output -= gain.Ki * error * cycle;

    pre_error = error;
    pre_prop = prop;

    return saturate(output, minLimit, maxLimit);
}

double ControlRegulator::saturate(double value, double min, double max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}
