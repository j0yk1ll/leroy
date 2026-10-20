// M23T: component-child-only OpenCV API threading differential probe.
// Diagnostic: cv::setNumThreads(1) changes scheduler behavior, NOT proven TBB usage.
// Compile locally against the existing Jazzy image OpenCV headers/linker ABI.
// No ROS API, camera, serial, filesystem writes, worker spawn or signal handling.
#include <opencv2/core.hpp>

#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

namespace {
bool component_child() {
  char exe[PATH_MAX];
  const ssize_t len = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
  if (len <= 0 || static_cast<size_t>(len) >= sizeof(exe)) {
    return false;
  }
  exe[len] = '\0';
  const char *basename = std::strrchr(exe, '/');
  return std::strcmp(basename ? basename + 1 : exe, "component_container") == 0;
}
}  // namespace

extern "C" __attribute__((constructor)) void m23t_opencv_thread_setup() {
  if (std::getenv("M23T_THREAD_PROBE_ONLY") == nullptr ||
      std::strcmp(std::getenv("M23T_THREAD_PROBE_ONLY"), "1") != 0 ||
      !component_child()) {
    return;
  }

  const char *profile = std::getenv("M23T_OPENCV_THREAD_PROFILE");
  if (profile == nullptr ||
      (std::strcmp(profile, "default") != 0 &&
       std::strcmp(profile, "single") != 0)) {
    std::fprintf(stderr, "M23T_OPENCV_THREAD_BLOCKED=INVALID_PROFILE\n");
    std::fflush(stderr);
    return;
  }

  const int before = cv::getNumThreads();
  if (std::strcmp(profile, "single") == 0) {
    // Explicitly configure OpenCV *before* RectifyNode is loaded.
    // This is not a guarantee of active worker count or TBB lifecycle.
    cv::setNumThreads(1);
  }
  const int after = cv::getNumThreads();
  std::fprintf(stderr,
      "M23T_OPENCV_THREAD_SETUP profile=%s before=%d after=%d pid=%ld\n",
      profile, before, after, static_cast<long>(getpid()));
  std::fflush(stderr);
}
