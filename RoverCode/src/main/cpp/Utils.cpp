#include "Utils.h"
#include <fmt/core.h>

double Utils::Deadzone(double in, double threshold) {
    return (abs(in) >= threshold ? in : 0);
}