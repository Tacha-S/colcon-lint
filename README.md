# colcon-lint

Colcon extension for linting ROS package dependencies.

This package is an extension of [colcon-core](https://github.com/colcon/colcon-core). Similar to [catkin-lint](https://github.com/fkie/catkin_lint) in ROS, it checks whether the dependencies of ROS2 packages are correctly described in the `package.xml`.

## Installation

To use this extension, please execute the following:

```bash
sudo apt install apt-rdepends
pip install colcon-lint
```

## How to use

Please execute as follows. The package specification option is the same as other colcon commands.

```bash
colcon lint --packages-select <package-name>
```

Replace `<package-name>` with the name of the package you want to check.

### Options

| Option | Description |
| --- | --- |
| package specification options | The same as other colcon commands. |
| logging options | The same as other colcon commands. |
| `--quick` | This option only checks for directly declared dependencies in the package.xml. It cannot detect dependencies that are resolved through recursively declared packages. |

## Tips

To run faster, you can build with the `-Wno-dev --trace-expand --trace-redirect=trace.log` cmake option.

```bash
colcon build --cmake-args -Wno-dev --trace-expand --trace-redirect=trace.log
```
