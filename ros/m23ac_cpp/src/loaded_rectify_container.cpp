// M23AC: preserve stock ComponentManager factory load, then explicitly
// promote its already-loaded librectify.so with or without RTLD_NODELETE.
// This is a synthetic-only process-lifetime experiment, not a fix.
#include <dlfcn.h>

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

#ifndef RTLD_NODELETE
#error "M23AC requires glibc RTLD_NODELETE"
#endif

namespace
{

class ObservedComponentManager : public rclcpp_components::ComponentManager
{
public:
  ObservedComponentManager(
    const std::weak_ptr<rclcpp::Executor> & executor, bool nodelete)
  : rclcpp_components::ComponentManager(executor), nodelete_(nodelete)
  {}

  std::shared_ptr<rclcpp_components::NodeFactory> create_component_factory(
    const ComponentResource & resource) override
  {
    // Preserve exact stock loading/registration before touching the DSO.
    auto factory = rclcpp_components::ComponentManager::create_component_factory(resource);
    if (resource.first != "image_proc::RectifyNode") {
      return factory;
    }
    if (!factory || seen_rectifier_) {
      throw rclcpp_components::ComponentManagerException(
              "M23AC: unexpected duplicate or absent rectifier factory");
    }

    const auto & path = resource.second;
    const std::string suffix = "/librectify.so";
    if (path.size() < suffix.size() ||
      path.compare(path.size() - suffix.size(), suffix.size(), suffix) != 0)
    {
      throw rclcpp_components::ComponentManagerException(
              "M23AC: unexpected rectifier library identity");
    }

    // In both arms: same RTLD_NOLOAD lookup, same dlclose, same hook timing.
    // Intervention differs ONLY by RTLD_NODELETE applied after stock load.
    const int flags = RTLD_NOW | RTLD_NOLOAD | (nodelete_ ? RTLD_NODELETE : 0);
    void * handle = ::dlopen(path.c_str(), flags);
    if (!handle) {
      throw rclcpp_components::ComponentManagerException(
              "M23AC: already-loaded librectify handle was not found");
    }
    const int close_status = ::dlclose(handle);
    if (close_status != 0) {
      throw rclcpp_components::ComponentManagerException(
              "M23AC: dlclose of additional diagnostic reference failed");
    }
    seen_rectifier_ = true;
    std::fprintf(stderr, "M23AC_FACTORY_HOOK=VERIFIED policy=%s\n",
      nodelete_ ? "nodelete" : "control");
    std::fflush(stderr);
    return factory;
  }

private:
  bool nodelete_;
  bool seen_rectifier_ = false;
};

}  // namespace

int main(int argc, char * argv[])
{
  const char * env = std::getenv("M23AC_LIBRARY_POLICY");
  const std::string policy = env ? env : "";
  if (policy != "control" && policy != "nodelete") {
    std::fputs("M23AC_PRECONDITION_BLOCKED=INVALID_POLICY\n", stderr);
    return 2;
  }

  rclcpp::init(argc, argv);
  std::shared_ptr<ObservedComponentManager> node;

  // Same lifetime ordering as stock Jazzy component_container.cpp:
  // executor declared after manager, therefore destroyed first.
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  node = std::make_shared<ObservedComponentManager>(
    exec, policy == "nodelete");
  exec->add_node(node);
  exec->spin();
  return 0;
}
