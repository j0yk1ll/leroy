from setuptools import setup

setup(name="leroy_capture", version="0.1.0", packages=["leroy_capture"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/leroy_capture"]),
                  ("share/leroy_capture", ["package.xml"])],
      install_requires=["setuptools"], zip_safe=True,
      maintainer="Leroy contributors", maintainer_email="maintainer@example.com",
      description="Passive bounded sensor capture", license="Apache-2.0",
      entry_points={"console_scripts": ["leroy_capture_recorder = leroy_capture.recorder:main"]})
