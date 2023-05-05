# colcon-lint

Colcon extension for linting ROS package dependencies.

This package is an extension of [colcon-core](https://github.com/colcon/colcon-core). Similar to [catkin-lint](https://github.com/fkie/catkin_lint) in ROS, it checks whether the dependencies of ROS2 packages are correctly described in the `package.xml`.

Currently, the extension checks the `exec_depend` section and verifies the launch files and Python scripts' imports.
The verification of `build_depend` is not yet implemented.
However, it is recommended to use [ament_cmake_auto](https://github.com/ament/ament_cmake/tree/rolling/ament_cmake_auto) to verify that there are no build errors.

## Installation

To use this extension, please execute the following:

```bash
git clone https://github.com/Tacha-S/colcon-lint.git
cd colcon-lint
sudo pip install .
```

## How to use

Please execute as follows. The package specification option is the same as other colcon commands.

```bash
colcon lint --packages-select <package-name>
```

Replace `<package-name>` with the name of the package you want to check.
