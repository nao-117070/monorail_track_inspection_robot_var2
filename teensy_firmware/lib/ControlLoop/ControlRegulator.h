#ifndef CONTROLREGULATOR_H
#define CONTROLREGULATOR_H

#include <algorithm>

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} GainParams;

class ControlRegulator {
  private:
    GainParams gain = {0.0, 0.0, 0.0};
    double cycle;
    double minLimit, maxLimit;
    double prop = 0, deriv = 0;
    double pre_error = 0, pre_prop = 0;
    double low_pass_deriv_ = 0;
    double output = 0;
    double lowPassFilterCoefficient = 8;

  public:
    ControlRegulator(): cycle(0), minLimit(0), maxLimit(0) {}
    ControlRegulator(double cycle, double min, double max);
    ~ControlRegulator();

    void setGain(GainParams gain);
    void setLimits(double min, double max);
    void setDerivativeFilter(double coeff);
    double compute(double error);
    double saturate(double value, double min, double max);
};

#endif
