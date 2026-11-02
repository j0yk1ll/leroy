from setuptools import find_packages, setup

package_name = "leroy_navigation"

setup(name=package_name, version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/leroy_navigation"]),
                  ("share/leroy_navigation", ["package.xml"])],
      install_requires=["setuptools"], zip_safe=True,
      maintainer="Leroy", maintainer_email="maintainer@example.com",
      description="Non-actuating holonomic goal controller",
      license="MIT", entry_points={"console_scripts": [
          "goal_controller = leroy_navigation.goal_node:main",
          "depth_grid = leroy_navigation.depth_grid_node:main",
      ]})
