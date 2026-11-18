#ifndef UTL80A_WRENCH_MOTION_FILTER_H
#define UTL80A_WRENCH_MOTION_FILTER_H

#include <math.h>
#include <stdint.h>

class WrenchMotionFilter {
 public:
  void reset() { count_ = 0; }

  bool update(const float input[6], float output[6]) {
    if (count_ < 2) {
      for (uint8_t axis = 0; axis < 6; ++axis) {
        if (count_ == 0) left_[axis] = input[axis];
        else center_[axis] = input[axis];
      }
      ++count_;
      return false;
    }

    // Per-axis thresholds are in N or N*m. The slew cap is per new sample.
    static const float spikeThreshold[] = {10.0f, 10.0f, 6.0f, .30f, .35f, .40f};
    static const float maxStep[] = {12.0f, 12.0f, 8.0f, .50f, .50f, .65f};
    for (uint8_t axis = 0; axis < 6; ++axis) {
      const float middle = center_[axis];
      const float a = middle - left_[axis];
      const float b = middle - input[axis];
      const float departure = fminf(fabsf(a), fabsf(b));
      const bool reversal = (a > 0.0f && b > 0.0f) || (a < 0.0f && b < 0.0f);
      const bool isolated = reversal && departure > spikeThreshold[axis]
                            && fabsf(input[axis] - left_[axis]) <= 0.5f * departure;
      const float candidate = isolated ? 0.5f * (left_[axis] + input[axis]) : middle;
      // At start, seed from the first screened output rather than the unscreened left sample.
      const float previous = count_ == 2 ? candidate : last_[axis];
      const float step = fmaxf(-maxStep[axis], fminf(maxStep[axis], candidate - previous));
      last_[axis] = output[axis] = previous + step;
      left_[axis] = middle;
      center_[axis] = input[axis];
    }
    count_ = 3;
    return true;
  }

 private:
  uint8_t count_ = 0;
  float left_[6] = {}, center_[6] = {}, last_[6] = {};
};

#endif
