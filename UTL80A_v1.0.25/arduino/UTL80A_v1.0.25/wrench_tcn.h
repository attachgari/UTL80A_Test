#pragma once

#include <math.h>
#include <stdint.h>

#include "tcn_weights.h"

class WrenchTcn {
 public:
  void reset() { count_ = 0; next_ = 0; }

  void linear(const float delta[8], float wrench[6]) const {
    for (uint8_t axis = 0; axis < 6; ++axis) {
      float sum = 0;
      for (uint8_t coil = 0; coil < 8; ++coil) {
        sum += TCN_LINEAR[axis][coil] * delta[coil] * TCN_INV_RAW_SCALE[coil];
      }
      wrench[axis] = sum * TCN_OUTPUT_SCALE[axis];
    }
  }

  bool update(uint32_t now, const float delta[8], float wrench[6]) {
    linear(delta, wrench);
    keep(now, delta);
    float history[7][8];
    if (!sampleHistory(now, history)) return false;

    float first[24];
    for (uint8_t block = 0; block < 3; ++block) {
      for (uint8_t output = 0; output < 8; ++output) {
        float sum = 0;
        for (uint8_t tap = 0; tap < 3; ++tap) {
          for (uint8_t coil = 0; coil < 8; ++coil) {
            sum += TCN_CONV1[output][8 * tap + coil] * history[2 * block + tap][coil];
          }
        }
        first[8 * block + output] = tanhf(sum);
      }
    }
    float second[8];
    for (uint8_t output = 0; output < 8; ++output) {
      float sum = 0;
      for (uint8_t j = 0; j < 24; ++j) sum += TCN_CONV2[output][j] * first[j];
      second[output] = tanhf(sum);
    }
    for (uint8_t axis = 0; axis < 6; ++axis) {
      for (uint8_t j = 0; j < 8; ++j) wrench[axis] += TCN_HEAD[axis][j] * second[j];
    }
    return true;
  }

 private:
  static constexpr uint8_t kSlots = 20;
  static constexpr uint32_t kStepUs = 20000;
  static constexpr uint32_t kMaxInterpolationGapUs = 65000;
  float samples_[kSlots][8];
  uint32_t time_[kSlots];
  uint8_t next_ = 0;
  uint8_t count_ = 0;

  uint8_t index(uint8_t ageInFrames) const {
    return (next_ + kSlots - 1 - ageInFrames) % kSlots;
  }

  void keep(uint32_t now, const float delta[8]) {
    time_[next_] = now;
    for (uint8_t i = 0; i < 8; ++i) samples_[next_][i] = delta[i] * TCN_INV_RAW_SCALE[i];
    next_ = (next_ + 1) % kSlots;
    if (count_ < kSlots) ++count_;
  }

  bool sampleHistory(uint32_t now, float result[7][8]) const {
    uint8_t olderOffset = 0;
    for (uint8_t tap = 0; tap < 7; ++tap) {
      const uint32_t lag = tap * kStepUs;
      while (olderOffset < count_ && now - time_[index(olderOffset)] < lag) ++olderOffset;
      if (olderOffset == count_) return false;
      const uint8_t older = index(olderOffset);
      const uint32_t olderAge = now - time_[older];
      uint8_t newer = older;
      float mix = 0;
      if (olderAge != lag) {
        newer = index(olderOffset - 1);
        const uint32_t newerAge = now - time_[newer];
        if (olderAge - newerAge > kMaxInterpolationGapUs) return false;
        mix = (float)(olderAge - lag) / (float)(olderAge - newerAge);
      }
      for (uint8_t i = 0; i < 8; ++i) {
        result[tap][i] = samples_[older][i] + mix * (samples_[newer][i] - samples_[older][i]);
      }
    }
    return true;
  }
};
