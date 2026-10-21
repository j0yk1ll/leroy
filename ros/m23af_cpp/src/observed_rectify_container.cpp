// M23AF: M23AC stock-factory + RTLD_NOLOAD control, with two
// read-only executable-file mapping snapshots in the *same* process.
// No LD_PRELOAD/interposing, no ptrace, no signal-handler replacement.
#include <dlfcn.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

#ifndef RTLD_NODELETE
#error "M23AF expects glibc RTLD_NODELETE available (even in control build)"
#endif

namespace
{

bool valid_run_id(const std::string & value)
{
  if (value.size() != 32) {
    return false;
  }
  for (char c : value) {
    if (!((c >= 'a' && c <= 'f') || (c >= '0' && c <= '9'))) {
      return false;
    }
  }
  return true;
}

void snapshot_executable_maps(const std::string & phase, const std::string & run_id)
{
  // /proc/self/maps is the current process's kernel-maintained virtual map
  // metadata. No memory from any mapped segment is opened or read.
  std::ifstream maps("/proc/self/maps");
  const long pid = static_cast<long>(::getpid());
  if (!maps.is_open()) {
    std::fprintf(stderr, "M23AF_MAP_ERROR phase=%s run_id=%s pid=%ld OPEN_FAILED\n",
      phase.c_str(), run_id.c_str(), pid);
    std::fflush(stderr);
    return;
  }

  std::fprintf(stderr, "M23AF_MAP_BEGIN phase=%s run_id=%s pid=%ld\n",
    phase.c_str(), run_id.c_str(), pid);
  std::string line;
  unsigned long count = 0;
  while (std::getline(maps, line)) {
    // Include only file-backed executable mappings with a filesystem path.
    if (line.find(" r-xp ") == std::string::npos &&
        line.find(" r-xs ") == std::string::npos)
    {
      continue;
    }
    if (line.find(" /") == std::string::npos) {
      continue;
    }
    std::fprintf(stderr, "M23AF_MAP_ENTRY phase=%s run_id=%s pid=%ld text=%s\n",
      phase.c_str(), run_id.c_str(), pid, line.c_str());
    ++count;
    if (count > 2048) {
      std::fprintf(stderr, "M23AF_MAP_ERROR phase=%s run_id=%s pid=%ld TOO_MANY\n",
        phase.c_str(), run_id.c_str(), pid);
      break;
    }
  }
  if (maps.bad()) {
    std::fprintf(stderr, "M23AF_MAP_ERROR phase=%s run_id=%s pid=%ld READ_FAILED\n",
      phase.c_str(), run_id.c_str(), pid);
  }
  std::fprintf(stderr, "M23AF_MAP_END phase=%s run_id=%s pid=%ld count=%lu\n",
    phase.c_str(), run_id.c_str(), pid, count);
  std::fflush(stderr);
}

class ObservedComponentManager : public rclcpp_components::ComponentManager
{
public:
  ObservedComponentManager(
    const std::weak_ptr<rclcpp::Executor> & executor, const std::string & run_id)
  : rclcpp_components::ComponentManager(executor), run_id_(run_id)
  {}

  std::shared_ptr<rclcpp_components::NodeFactory> create_component_factory(
    const ComponentResource & resource) override
  {
    // EXACT existing M23AC sequence: normal stock factory, RTLD_NOLOAD
    // on librectify.so without NODELETE, immediate matching dlclose.
    auto factory = rclcpp_components::ComponentManager::create_component_factory(resource);
    if (resource.first != "image_proc::RectifyNode") {
      return factory;
    }
    if (!factory || seen_rectifier_) {
      throw rclcpp_components::ComponentManagerException(
              "M23AF: unexpected duplicate or absent rectifier factory");
    }
    const auto & path = resource.second;
    const std::string suffix = "/librectify.so";
    if (path.size() < suffix.size() ||
        path.compare(path.size() - suffix.size(), suffix.size(), suffix) != 0)
    {
      throw rclcpp_components::ComponentManagerException(
              "M23AF: unexpected rectifier library identity");
    }
    const int flags = RTLD_NOW | RTLD_NOLOAD;  // control ONLY
    void * handle = ::dlopen(path.c_str(), flags);
    if (!handle) {
      throw rclcpp_components::ComponentManagerException(
              "M23AF: already-loaded librectify handle absent");
    }
    if (::dlclose(handle) != 0) {
      throw rclcpp_components::ComponentManagerException(
              "M23AF: diagnostic dlclose failed");
    }
    seen_rectifier_ = true;
    std::fprintf(stderr, "M23AC_FACTORY_HOOK=VERIFIED policy=control\n");
    std::fflush(stderr);
    snapshot_executable_maps("post_factory", run_id_);
    return factory;
  }

private:
  std::string run_id_;
  bool seen_rectifier_ = false;
};

}  // namespace

int main(int argc, char * argv[])
{
  const char * policy_env = std::getenv("M23AC_LIBRARY_POLICY");
  const char * id_env = std::getenv("M23AF_RUN_ID");
  const std::string policy = policy_env ? policy_env : "";
  const std::string run_id = id_env ? id_env : "";
  if (policy != "control" || !valid_run_id(run_id)) {
    std::fputs("M23AF_PRECONDITION_BLOCKED=POLICY_OR_RUN_ID\n", stderr);
    return 2;
  }

  rclcpp::init(argc, argv);
  // This callback fires when ROS context shutdown is signaled, not in a
  // user-installed POSIX signal handler. Its mapping snapshot is BEFORE
  // normal ComponentManager destructor/loaders_ cleanup, but does NOT
  // attest the later dlclose/unmap order.
  rclcpp::on_shutdown([run_id]() {
      snapshot_executable_maps("on_shutdown", run_id);
    });
  std::shared_ptr<ObservedComponentManager> node;

  // Preserve component_container's stock executor/manager lifetime order.
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  node = std::make_shared<ObservedComponentManager>(exec, run_id);
  exec->add_node(node);
  exec->spin();
  return 0;
}
