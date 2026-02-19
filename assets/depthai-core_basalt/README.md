# depthai-core BasaltVIO Patches

These are patched versions of two files from [depthai-core](https://github.com/luxonis/depthai-core) that fix
several bugs in the `BasaltVIO` implementation required for correct VIO operation.

See the full explanation of each bug and fix in [BASALT_GUIDE.md](../../BASALT_GUIDE.md#required-depthai-core-patches).

## Files

| Patched File (here) | Original Location in depthai-core |
|---|---|
| `include/depthai/basalt/BasaltVIO.hpp` | `include/depthai/basalt/BasaltVIO.hpp` |
| `src/basalt/BasaltVIO.cpp` | `src/basalt/BasaltVIO.cpp` |

## How to Apply

Copy the files into your local depthai-core source, then rebuild:

```bash
cp include/depthai/basalt/BasaltVIO.hpp  $DEV_HOME/depthai-core/include/depthai/basalt/BasaltVIO.hpp
cp src/basalt/BasaltVIO.cpp              $DEV_HOME/depthai-core/src/basalt/BasaltVIO.cpp

cd $DEV_HOME/depthai-core
cmake --build build -j8
sudo cmake --install build
sudo ldconfig
```

Then rebuild depthai-ros (ABI changed due to `std::atomic<bool>`):

```bash
cd $DEV_HOME/ros2_ws
source /opt/ros/jazzy/setup.bash
MAKEFLAGS="-j4" colcon build --packages-select depthai_ros_driver --parallel-workers 1
source install/setup.bash
```

## Bugs Fixed

1. **Accel/Gyro bias not applied** — bias values ignored/mangled, causing km-scale VIO drift
2. **Gyro bias error message typo** — said "9 elements" instead of "12 elements"
3. **`stop()` null queue crash** — intermittent segfault if shutdown before first stereo frame
4. **Race condition on `initialized` flag** — `bool` → `std::atomic<bool>` with acquire/release ordering
5. **Same IMU extrinsic for both cameras** — right camera T_i_c now derived from left + EEPROM stereo baseline
