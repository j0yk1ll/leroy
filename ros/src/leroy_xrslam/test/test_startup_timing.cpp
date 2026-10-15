// Runtime tests for the same ROS-free predicate used by the XRSLAM adapter.
#include "leroy_xrslam/startup_timing.hpp"

#include <cstdio>

namespace {
bool check(const char* name, double image, double offset, double imu, bool expected) {
  const bool result = leroy_xrslam::camera_after_first_forwarded_imu(
      image, offset, imu);
  if (result != expected) {
    std::fprintf(stderr,
                 "FAILED: %s: image=%.6f offset=%.6f first_imu=%.6f "
                 "expected=%d actual=%d\n",
                 name, image, offset, imu, expected, result);
    return false;
  }
  return true;
}
}  // namespace

int main() {
  bool ok = true;
  // Negative offset: the raw image header may be AFTER the first IMU sample,
  // yet the corrected image time can still be BEFORE that sample.
  ok &= check("negative-before", 10.010, -0.020, 10.000, false);
  ok &= check("negative-equal",  10.020, -0.020, 10.000, false);
  ok &= check("negative-after",  10.030, -0.020, 10.000, true);

  // Zero offset: frame header and effective time coincide.
  ok &= check("zero-before", 9.990, 0.0, 10.000, false);
  ok &= check("zero-equal",  10.000, 0.0, 10.000, false);
  ok &= check("zero-after",  10.010, 0.0, 10.000, true);

  // Positive offset can make an EARLIER image safe after the initial IMU.
  ok &= check("positive-before", 9.970, 0.020, 10.000, false);
  ok &= check("positive-equal",  9.980, 0.020, 10.000, false);
  ok &= check("positive-after",  9.990, 0.020, 10.000, true);

  // An unset first IMU time must never permit camera frame delivery.
  ok &= check("no-imu-forwarded", 10.500, 0.020, -1.0, false);
  return ok ? 0 : 1;
}
