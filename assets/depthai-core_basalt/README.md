# Basalt VIO Setup Guide — OAK-FFC-3P with depthai-ros

How to configure and run Basalt Visual-Inertial Odometry using `depthai-ros` on the OAK-FFC-3P, using calibration results from [basalt_ros2](https://github.com/roboticsmick/basalt_ros2).

---

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Heap Corruption Bug Fixes (Critical)](#heap-corruption-bug-fixes-critical)
- [Required depthai-core Patches](#required-depthai-core-patches)
- [Understanding Your Calibration Files](#understanding-your-calibration-files)
- [Merging Calibrations (Optional — for Photogrammetry)](#merging-calibrations-optional--for-photogrammetry)
- [Step 1: Create the VIO YAML Config](#step-1-create-the-vio-yaml-config)
- [Step 2: Tune the Basalt Algorithm Config (Optional)](#step-2-tune-the-basalt-algorithm-config-optional)
- [Step 3: Launch VIO](#step-3-launch-vio)
- [Step 4: Verify VIO Output](#step-4-verify-vio-output)
- [Running VIO with RTABMap SLAM](#running-vio-with-rtabmap-slam)
- [Parameter Reference](#parameter-reference)
- [Troubleshooting](#troubleshooting)

---

## Overview

The `depthai-ros` driver includes a built-in **Basalt VIO** pipeline node (`dai::node::BasaltVIO`) that runs visual-inertial odometry on-device. It:

1. Takes **stereo images** (left + right) and **IMU data** as input
2. Runs the Basalt VIO algorithm
3. Outputs **6-DOF pose estimates** as `nav_msgs/Odometry` on the `/oak/vio/odometry` topic
4. Optionally publishes TF transforms (`odom` → `oak_parent_frame`)

The VIO node is created automatically when `i_enable_vio: true` is set in any stereo-capable pipeline (RGBD, Stereo, Depth, RGBStereo).

### Architecture

```
OAK-FFC-3P Device
├── CAM_B (left OV9282)  ──→ ┐
├── CAM_C (right OV9282) ──→ ├──→ BasaltVIO node ──→ TransformData ──→ ROS2 Odometry
└── BMI270 IMU           ──→ ┘
```

The BasaltVIO node runs on the host (via depthai-core), NOT on the OAK's VPU. It uses:
- **Device EEPROM** calibration for camera intrinsics and stereo extrinsics (factory calibration)
- **ROS parameters** to override IMU extrinsics, biases, and noise (from your Basalt calibration)
- **Basalt JSON config** for algorithm tuning (optical flow, optimization, keyframes, etc.)

---

## Prerequisites

1. **depthai-core built from source with Basalt support:**

   ```bash
   # Workspace-based installation (recommended, no sudo needed)
   cmake -S . -B build \
       -DDEPTHAI_BASALT_SUPPORT=ON \
       -DCMAKE_BUILD_TYPE=Release \
       -DCMAKE_INSTALL_PREFIX=/media/logic/USamsung/ros2_ws/install
   cmake --build build -j8
   cmake --install build
   # NO sudo or ldconfig needed for workspace installation
   ```

   See the [depthai-ros README](README.md) for full build instructions.

2. **depthai-ros workspace built:**

   ```bash
   cd /media/logic/USamsung/ros2_ws
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash  # Source workspace to find depthai-core

   # CRITICAL: Pass CMAKE_PREFIX_PATH to find vcpkg dependencies
   MAKEFLAGS="-j8" colcon build --packages-select depthai_ros_driver \
     --parallel-workers 1 \
     --cmake-args -DCMAKE_BUILD_TYPE=Release \
     -DCMAKE_PREFIX_PATH="/media/logic/USamsung/ros2_ws/install/lib/cmake;/media/logic/USamsung/depthai-core/build/vcpkg_installed/x64-linux/share"
   ```

   **Why CMAKE_PREFIX_PATH is needed:** depthai-core depends on vcpkg packages (nlohmann_json, XLink, OpenCV, xtensor) that must be found during cmake configuration.

### Building with AddressSanitizer (ASAN) for debugging

**ASAN is for development/debugging only.** Production builds should use Release without ASAN.

```bash
cd /media/logic/USamsung/depthai-core
rm -rf build                # Recommended when changing compile/link flags

cmake -S . -B build \
  -DDEPTHAI_BASALT_SUPPORT=ON \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_INSTALL_PREFIX=/media/logic/USamsung/ros2_ws/install \
  -DCMAKE_C_FLAGS="-fsanitize=address -fno-omit-frame-pointer -g" \
  -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer -g" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address" \
  -DCMAKE_SHARED_LINKER_FLAGS="-fsanitize=address"

cmake --build build -j8
cmake --install build  # No sudo needed for workspace installation
```

Then launch with ASAN runtime:

```bash
source /media/logic/USamsung/ros2_ws/install/setup.bash
ros2 launch depthai_ros_driver driver.launch.py \
  params_file:=/media/logic/USamsung/ros2_ws/src/depthai-ros/depthai_ros_driver/config/oak_ffc_3p_vio.yaml
```

Check ASAN report:
```bash
cat /tmp/asan_depthai.* | grep -E "ERROR|SUMMARY|double-free|use-after-free"
```

3. **Basalt calibration completed** (stereo + IMU at minimum). See the [basalt_ros2 README](https://github.com/roboticsmick/basalt_ros2) for the calibration workflow.

---

## Heap Corruption Bug Fixes (Critical)

**Status:** ✅ **FIXED** — Applied to both `BasaltVIO.hpp` and `BasaltVIO.cpp`

When launching VIO with ASAN enabled, the upstream depthai-core exhibited deterministic heap corruption crashes ("free(): unaligned chunk detected in tcache 2", "double free or corruption (!prev)", "SEGV on unknown address", "attempting double-free") immediately after "Robot initialized" or after running for several seconds. These were caused by several distinct race conditions and async callback lifetime issues.

### Issue 1: TFPublisher Async Parameter Setting Race

**Files affected:** `depthai_bridge/include/depthai_bridge/TFPublisher.hpp`, `depthai_bridge/src/TFPublisher.cpp`

**Problem:**
- `TFPublisher::publishDescription()` called `paramClient->set_parameters()` asynchronously and **discarded the returned Future**
- The async callback continued after the function returned, potentially accessing destroyed TFPublisher members
- This caused a double-free in ROS2 parameter client internals

**Fix:**
```cpp
// In publishDescription():
auto result = paramClient->set_parameters({robotDescr});
if(result.valid()) {
    try {
        result.wait_for(std::chrono::seconds(2));
    } catch(const std::exception& e) {
        RCLCPP_WARN(logger, "Exception while waiting for set_parameters: %s", e.what());
    }
}
```

**Why it works:** Waiting for the future synchronously ensures the async operation completes before `publishDescription()` returns, so no callback accesses destroyed members.

---

### Issue 2: BasaltVIO leftImg Thread Race Condition

**Files affected:** `include/depthai/basalt/BasaltVIO.hpp`, `src/basalt/BasaltVIO.cpp`

**Problem:**
- The `leftImg` shared_ptr was accessed by two threads without synchronization:
  - `stereoCB()` **writes** `leftImg = imgFrame;` in the device callback thread
  - `run()` **reads** `if(leftImg) passthrough.send(leftImg);` in the VIO worker thread
- Both threads modify the shared_ptr's reference count simultaneously
- This causes "attempting double-free" or "use-after-free" errors on the shared_ptr control block

**Fix (Part 1 - Add Mutex):**

In `BasaltVIO.hpp`:
```cpp
#include <mutex>

class BasaltVIO : public ... {
    // ...
    std::shared_ptr<ImgFrame> leftImg;
    mutable std::mutex leftImgMutex;  // Protects leftImg from simultaneous access
```

In `BasaltVIO.cpp`, `stereoCB()` method:
```cpp
for(auto& msg : *group) {
    std::shared_ptr<ImgFrame> imgFrame = std::dynamic_pointer_cast<ImgFrame>(msg.second);
    if(i == 0) {
        std::lock_guard<std::mutex> lock(leftImgMutex);
        leftImg = imgFrame;  // Protected write
    }
    // ...
}
```

**Why:** Ensures only one thread at a time modifies the leftImg shared_ptr.

---

### Issue 3: Refcount Race During Message Send

**Files affected:** `src/basalt/BasaltVIO.cpp`

**Problem:**
- Even with the mutex protecting the `leftImg = imgFrame;` assignment, the subsequent `passthrough.send(leftImg)` was **still holding the mutex**
- `send()` internally queues the message and modifies the shared_ptr refcount
- `stereoCB()` could enter the mutex and decrement the refcount while `send()` was still incrementing it
- This causes a SEGV in `std::_Sp_counted_base<>::_M_release_last_use_cold()`

**Fix:**

In `BasaltVIO.cpp`, `run()` method:
```cpp
// Copy leftImg under lock, then send outside lock to avoid refcount races
std::shared_ptr<ImgFrame> imgToSend;
{
    std::lock_guard<std::mutex> lock(leftImgMutex);
    imgToSend = leftImg;  // Atomic copy under lock
}
if(imgToSend) passthrough.send(imgToSend);  // Send outside lock
```

**Why it works:**
1. The **copy** of the shared_ptr happens atomically under the lock
2. This increments the refcount once, safely, while the mutex prevents stereoCB from interfering
3. The **send()** call happens without the lock, so stereoCB can freely update leftImg in parallel
4. When stereoCB updates leftImg, the refcount for the old object decrements cleanly
5. When run() sends imgToSend, the refcount for the copy decrements cleanly
6. No two threads are modifying the same refcount simultaneously

---

### Issue 4: ImgFrame Refcount Race in stereoCB Callback

**Files affected:** `src/basalt/BasaltVIO.cpp`

**Problem:**
- The `stereoCB()` callback received `ImgFrame` shared_ptrs from the message queue
- The callback held onto these references throughout the entire loop processing
- While stereoCB was accessing the frame, another thread could be releasing its reference
- This caused concurrent reference count modifications, leading to "attempting double-free" errors
- The ImgTransformation destructor would be called multiple times on the same memory

**Root Cause:**
When deserializing incoming images from XLinkInHost, the device thread creates ImgFrames. These are passed to the stereoCB callback via MessageGroup. If both:
1. The callback thread still holds a shared_ptr to the frame
2. The device input thread's reference count drops to zero
Then the frame is deleted while the callback is still using it, causing a use-after-free race.

**Fix:**

In `BasaltVIO.cpp`, `stereoCB()` method, extract data then immediately release the imgFrame:

```cpp
for(auto& msg : *group) {
    // Extract all needed data from imgFrame before releasing the shared_ptr
    // to avoid race conditions with concurrent thread access.
    std::shared_ptr<ImgFrame> imgFrame = std::dynamic_pointer_cast<ImgFrame>(msg.second);
    if(!imgFrame) continue;

    auto t = imgFrame->getTimestamp();
    int64_t tNS = std::chrono::time_point_cast<std::chrono::nanoseconds>(t).time_since_epoch().count();
    auto exposure = imgFrame->getExposureTime();
    int exposureMS = std::chrono::duration_cast<std::chrono::milliseconds>(exposure).count();
    size_t width = imgFrame->getWidth();
    size_t height = imgFrame->getHeight();
    size_t fullSize = width * height;
    const auto srcData = imgFrame->getData();  // Make a copy to avoid holding reference
    const uint8_t* dataIN = srcData.data();
    size_t dataSize = srcData.size();

    // Release imgFrame reference immediately after copying data
    imgFrame.reset();

    if(dataSize < fullSize) {
        std::cerr << "BasaltVIO::stereoCB: frame data size (" << dataSize << ") < expected (" << fullSize
                  << ") — skipping frame to avoid overflow\n";
        return;
    }

    data->img_data[i].img = std::make_shared<basalt::ManagedImage<uint16_t>>(width, height);
    data->t_ns = tNS;
    data->img_data[i].exposure = exposureMS;
    uint16_t* data_out = data->img_data[i].img->ptr;
    for(size_t j = 0; j < fullSize; j++) {
        int val = dataIN[j];
        val = val << 8;
        data_out[j] = val;
    }
    i++;
}
```

**Why it works:**

1. **Early release**: `imgFrame.reset()` is called immediately after extracting all needed data
2. **Data copy**: `srcData = imgFrame->getData()` makes a copy of the vector, so we don't hold a reference to the original frame's data
3. **Primitive copying**: Width, height, timestamp, and exposure are primitives — copying them doesn't hold any shared resources
4. **No concurrent refcount modifications**: The imgFrame's refcount is decremented while no other thread is modifying it
5. **Safe processing**: All pixel data processing happens on the copied data, not the original frame

---

### Verification

After applying these four fixes and rebuilding:

```bash
rm -f /tmp/asan_depthai.*
source $DEV_HOME/ros2_ws/install/setup.bash
ros2 launch depthai_ros_driver driver.launch.py \
  params_file:=$DEV_HOME/ros2_ws/src/depthai-ros/depthai_ros_driver/config/oak_ffc_3p_vio.yaml \
  use_asan:=true
```

**Expected result:** VIO initializes and runs without ASAN errors. The process should run for extended periods (minutes) without crashing.

**Check for errors:**
```bash
cat /tmp/asan_depthai.* | grep -E "ERROR|SUMMARY|double-free|use-after-free"
```

If clean, you should see no errors (or only pre-existing ROS2 middleware issues like `new_delete_type_mismatch`).

---

## Required depthai-core Patches

The upstream `depthai-core` BasaltVIO implementation has several bugs that must be fixed for correct operation. Apply these patches to your local depthai-core source before building.

### Patch Files

All changes are in two files:

- `include/depthai/basalt/BasaltVIO.hpp`
- `src/basalt/BasaltVIO.cpp`

### Bug 1: Accel/Gyro Bias Not Applied Correctly

**Problem:** The original `initialize()` method incorrectly transforms the bias values from Basalt calibration. It skips the additive bias components `[b_x, b_y, b_z]` and incorrectly adds `+1` to scale matrix elements. This results in massive VIO drift (thousands of meters) because the raw accelerometer bias (~1.75 m/s²) is double-integrated without correction.

**Fix** (`BasaltVIO.cpp`, in `initialize()`): Assign the Basalt calibration values directly to `CalibAccelBias` (9 elements) and `CalibGyroBias` (12 elements) without transformation:

```cpp
// IMU calibration: bias and noise are per-IMU (set once, outside the camera loop)
if(accelBias.has_value()) {
    basalt::CalibAccelBias<Scalar> accel_bias;
    for(int j = 0; j < 9; j++) {
        accel_bias.getParam()[j] = accelBias.value()[j];
    }
    pimpl->calib->calib_accel_bias = accel_bias;
}
if(gyroBias.has_value()) {
    basalt::CalibGyroBias<Scalar> gyro_bias;
    for(int j = 0; j < 12; j++) {
        gyro_bias.getParam()[j] = gyroBias.value()[j];
    }
    pimpl->calib->calib_gyro_bias = gyro_bias;
}
```

Also moved bias/noise assignment **outside** the per-camera `for` loop since these are per-IMU, not per-camera.

### Bug 2: Gyro Bias Validation Error Message Typo

**Problem:** `setGyroBias()` validates for 12 elements but the error message said "must have 9 elements".

**Fix** (`BasaltVIO.cpp`, line 181): Changed error message to "must have 12 elements".

### Bug 3: `stop()` Crashes on Null Queues (Intermittent Segfault)

**Problem:** If the node shuts down before the first stereo frame arrives (e.g., USB hiccup, pipeline timing), `stop()` unconditionally dereferences `imageDataQueue` and `imuDataQueue` which are `nullptr` until `initialize()` runs. This causes an intermittent segfault after "Driver ready!".

**Fix** (`BasaltVIO.cpp`, `stop()`):

```cpp
void BasaltVIO::stop() {
    if(pimpl->imageDataQueue) pimpl->imageDataQueue->push(nullptr);
    if(pimpl->imuDataQueue) pimpl->imuDataQueue->push(nullptr);
    ThreadedHostNode::stop();
}
```

### Bug 4: Race Condition on `initialized` Flag

**Problem:** `initialized` was a plain `bool` written by the `stereoCB` thread and read by the `run()` thread. Without memory barriers, the CPU/compiler can reorder writes so that `run()` sees `initialized = true` before the queue pointers are fully visible, causing a segfault.

**Fix** (`BasaltVIO.hpp`): Add `#include <atomic>` and change:

```cpp
// Before:
bool initialized = false;
// After:
std::atomic<bool> initialized{false};
```

**Fix** (`BasaltVIO.cpp`): Use explicit memory ordering:

```cpp
// In run():
if(!initialized.load(std::memory_order_acquire)) continue;

// In stereoCB():
if(!initialized.load(std::memory_order_acquire)) { ... }

// At end of initialize():
initialized.store(true, std::memory_order_release);
```

### Bug 5: Same IMU Extrinsic Used for Both Stereo Cameras

**Problem:** When using `i_override_imu_extrinsics`, the same `T_imu_cam` transform is pushed for both the left and right cameras, giving them a zero stereo baseline. The EEPROM path correctly queries per-camera extrinsics.

**Fix** (`BasaltVIO.cpp`, in `initialize()`): For the first camera (cam0), use the override directly. For subsequent cameras, compute `T_i_cn = T_i_c0 * getCameraExtrinsics(cam_n, cam_0)` using the stereo baseline from the device EEPROM:

```cpp
int camIdx = 0;
basalt::Calibration<Scalar>::SE3 T_i_c0_override;
CameraBoardSocket firstCamSocket{};
for(const auto& frame : frames) {
    // ...
    if(imuExtrinsics.has_value()) {
        if(camIdx == 0) {
            // First camera: use override directly
            T_i_c0_override = basalt::Calibration<Scalar>::SE3(q, trans);
            firstCamSocket = camID;
            pimpl->calib->T_i_c.push_back(T_i_c0_override);
        } else {
            // Subsequent cameras: derive from stereo baseline in EEPROM
            auto stereoExtr = calibHandler.getCameraExtrinsics(camID, firstCamSocket, useSpecTranslation);
            // ... parse rotation R and translation (cm → m) ...
            basalt::Calibration<Scalar>::SE3 T_c0_cn(q, trans);
            pimpl->calib->T_i_c.push_back(T_i_c0_override * T_c0_cn);
        }
    }
    // ...
    camIdx++;
}
```

### Rebuilding depthai-core After Patching

```bash
cd $DEV_HOME/depthai-core

# Rebuild (limit to 8 threads to avoid OOM on 32GB systems)
cmake --build build -j8

# Install
sudo cmake --install build
sudo ldconfig
```

### Rebuilding depthai-ros After depthai-core Changes

Since the header changed (ABI change from `bool` to `std::atomic<bool>`), depthai-ros must be rebuilt:

```bash
cd $DEV_HOME/ros2_ws
source /opt/ros/jazzy/setup.bash

# Limit parallel compilation to avoid OOM
MAKEFLAGS="-j4" colcon build --packages-select depthai_ros_driver --parallel-workers 1
source install/setup.bash
```

---

## Understanding Your Calibration Files

The Basalt calibration pipeline produces results in two (or three) stages. Understanding which file to use for VIO is critical.

### Calibration File Inventory

| File | Cameras | IMU | Use For |
|------|---------|-----|---------|
| `stereo_imu_calibration_results/calibration.json` | 2 (left + right stereo) | Yes (calibrated biases + noise) | **VIO** |
| `basalt_calibration_results/calibration.json` | 3 (left + RGB + right) | Yes (from separate IMU cal) | 3-camera extrinsics |
| `merged_calibration_results/calibration.json` | 3 (merged) | Yes | Photogrammetry pipeline |

### For VIO: Use the 2-Camera Stereo+IMU Calibration

**The VIO only needs the stereo pair and IMU.** Use:

```
$DEV_HOME/basalt_calibration/stereo_imu_calibration_results/calibration.json
```

This file contains:
- `T_imu_cam[0]` — Transform from IMU to left camera (cam0)
- `T_imu_cam[1]` — Transform from IMU to right camera (cam1)
- `calib_accel_bias` — 9-element accelerometer bias vector
- `calib_gyro_bias` — 12-element gyroscope bias vector
- `accel_noise_std` — Accelerometer noise standard deviation `[x, y, z]`
- `gyro_noise_std` — Gyroscope noise standard deviation `[x, y, z]`
- `imu_update_rate` — Calibrated IMU rate (Hz)

### Mapping Calibration Values to VIO Parameters

The depthai-ros VIO node accepts overrides for IMU-related calibration through ROS parameters. Here's the mapping from the Basalt `calibration.json` structure:

| Basalt calibration.json field | VIO ROS parameter | Enable flag |
|-------------------------------|-------------------|-------------|
| `T_imu_cam[0]` (left cam) | `i_imu_extr_{x,y,z,qx,qy,qz,qw}` | `i_override_imu_extrinsics: true` |
| `calib_accel_bias` (9 values) | `i_acc_bias` | `i_set_acc_bias: true` |
| `calib_gyro_bias` (12 values) | `i_gyro_bias` | `i_set_gyro_bias: true` |
| `accel_noise_std` (3 values) | `i_acc_noise_std` | `i_set_acc_noise_std: true` |
| `gyro_noise_std` (3 values) | `i_gyro_noise_std` | `i_set_gyro_noise_std: true` |

Camera intrinsics and stereo extrinsics come from the **device EEPROM** (factory calibration). The Basalt calibration overrides the IMU parameters which are typically less accurate in factory calibration.

---

## Merging Calibrations (Optional — for Photogrammetry)

> **Skip this section if you only need VIO.** VIO uses the 2-camera stereo+IMU calibration directly.

The merge script combines the stereo+IMU calibration (accurate IMU parameters) with the 3-camera calibration (RGB extrinsics) into a single file for the photogrammetry pipeline.

### How to Run the Merge Script

```bash
python3 $DEV_HOME/basalt_ros2/scripts/merge_calibrations.py \
  --stereo-imu $DEV_HOME/basalt_calibration/stereo_imu_calibration_results/calibration.json \
  --three-cam $DEV_HOME/basalt_calibration/basalt_calibration_results/calibration.json \
  --output $DEV_HOME/basalt_calibration/merged_calibration_results/calibration.json
```

**Dependencies:**

```bash
pip install numpy scipy
```

**What it does:**

1. Takes stereo intrinsics + IMU parameters from the stereo+IMU calibration (Steps 1-2)
2. Takes RGB intrinsics + relative RGB extrinsics from the 3-camera calibration (Step 3)
3. Computes `T_imu_rgb` by chaining: `T_left_rgb = T_imu_left_3cam⁻¹ × T_imu_rgb_3cam`, then `T_imu_rgb = T_imu_left_stereo × T_left_rgb`
4. Outputs a merged file with cam0=left, cam1=right, cam2=RGB — all in the stereo+IMU reference frame

**Camera index defaults** (matching OAK-FFC-3P topic ordering):

| Calibration | cam0 | cam1 | cam2 |
|-------------|------|------|------|
| Stereo+IMU | Left (CAM_B) | Right (CAM_C) | — |
| 3-camera | Left (CAM_B) | RGB (CAM_A) | Right (CAM_C) |

If your camera ordering differs, use the index override flags:

```bash
python3 merge_calibrations.py \
  --stereo-imu ... \
  --three-cam ... \
  --output ... \
  --stereo-left-idx 0 \
  --stereo-right-idx 1 \
  --three-cam-left-idx 0 \
  --three-cam-rgb-idx 1
```

### Use Case: Photogrammetry Pipeline

The merged calibration enables projecting VIO poses onto RGB frames:

```
Stereo + IMU → Basalt VIO → T_world_imu (at stereo rate)
                                  ↓
T_world_rgb = T_world_imu × T_imu_rgb → interpolate to RGB timestamps
                                  ↓
RGB images + accurate poses + stereo depth → photogrammetry / SFM
```

---

## Step 1: Create the VIO YAML Config

Create a VIO configuration file that combines your camera setup from `oak_ffc_3p_stereo_disparity.yaml` with calibration overrides from your Basalt results.

### Key Differences: Stereo Depth Config vs VIO Config

| Setting | Stereo Depth (`oak_ffc_3p_stereo_disparity.yaml`) | VIO (`vio.yaml`) |
|---------|---------------------------------------------------|-------------------|
| Resolution | 1280x800 (full stereo) | 640x400 (downsampled for speed) |
| FPS | 30 | 60 (higher for better IMU integration) |
| Pipeline | RGBD with depth filters | Stereo with VIO enabled |
| IMU rate | 200 Hz | 400 Hz (max supported by BMI270) |

VIO uses lower resolution and higher framerate because:
- Optical flow tracking works well at 640x400
- Higher FPS gives better temporal sampling for IMU preintegration
- Lower resolution reduces computational load

### VIO Config File

Create `$DEV_HOME/ros2_ws/src/depthai-ros/depthai_ros_driver/config/oak_ffc_3p_vio.yaml`:

> **Important:** The `i_override_imu_extrinsics`, `i_set_*_bias`, and `i_set_*_noise_std` values below come directly from your `stereo_imu_calibration_results/calibration.json`. If you re-run calibration, update these values.

```yaml
# OAK-FFC-3P Basalt VIO Configuration
# Stereo + IMU → Visual-Inertial Odometry
#
# Calibration source: stereo_imu_calibration_results/calibration.json
# Camera: OAK-FFC-3P with OV9282 stereo (CAM_B/CAM_C) + BMI270 IMU
/oak:
  ros__parameters:
    pipeline_gen:
      i_nn_type: none
      i_pipeline_type: Stereo
      i_enable_imu: true
      i_enable_vio: true

    # === VIO NODE SETTINGS ===
    vio:
      # --- Resolution & FPS ---
      # VIO uses lower resolution than stereo depth for speed
      i_width: 640
      i_height: 400
      i_fps: 60.0
      i_board_socket_id: 1  # CAM_B (left OV9282)

      # --- IMU ---
      i_imu_update_rate: 400  # BMI270 max rate

      # --- IMU Extrinsics (T_imu_cam[0] from Basalt calibration) ---
      # Transform from IMU to left camera frame
      i_override_imu_extrinsics: true
      i_imu_extr_x: -0.061409340773537619
      i_imu_extr_y: 0.052009437125282959
      i_imu_extr_z: -0.01408594567155951
      i_imu_extr_qx: 0.49978286617904008
      i_imu_extr_qy: -0.5027659780640653
      i_imu_extr_qz: -0.5007287704258576
      i_imu_extr_qw: 0.49670328813382788

      # --- Accelerometer Bias (9-element vector from calib_accel_bias) ---
      i_set_acc_bias: true
      i_acc_bias: [1.7551297205505288, 0.13549662999103519, -0.10387422832059688,
                   0.00008330762953156286, 0.0000057108229485075309, 0.00002305400408125457,
                   -0.00005815733711713035, 0.00026820041781779275, -0.000003959719132717111]

      # --- Gyroscope Bias (12-element vector from calib_gyro_bias) ---
      i_set_gyro_bias: true
      i_gyro_bias: [0.0017533344769401496, -0.001566063358788915, -0.0006372898046244716,
                    0.00005397485991596637, 0.0000048686858679756359, 0.000001848503523484684,
                    -0.00001524400445348047, 0.000034649006279934928, -0.000013906028441460435,
                    0.0000024479718214005174, -0.000002739126215133143, 0.00003687581130193044]

      # --- IMU Noise Parameters ---
      # From Basalt calibration (BMI270 datasheet values used during calibration)
      i_set_acc_noise_std: true
      i_acc_noise_std: [0.02, 0.02, 0.02]

      i_set_gyro_noise_std: true
      i_gyro_noise_std: [0.0005, 0.0005, 0.0005]

      # --- TF Publishing ---
      i_publish_tf: true
      i_frame_id: "odom"
      i_child_frame_id: "oak_parent_frame"

      # --- Basalt Algorithm Config ---
      # Path to Basalt VIO algorithm parameters (optical flow, optimization, etc.)
      # Default uses the built-in config. To customize, copy and modify:
      #   depthai_ros_driver/config/custom/depthai_ros_driver_default_vio.json
      # i_config_path: "/absolute/path/to/custom_vio_config.json"

    # === STEREO (for depth — runs alongside VIO) ===
    stereo:
      i_depth_preset: DEFAULT
      i_fps: 60.0
      i_left_socket_id: 1   # CAM_B
      i_right_socket_id: 2  # CAM_C

    # === LEFT/RIGHT CAMERA EXPOSURE ===
    left:
      r_set_man_exposure: false
      r_set_auto_exposure_limit: true
      r_auto_exposure_limit: 6000

    right:
      r_set_man_exposure: false
      r_set_auto_exposure_limit: true
      r_auto_exposure_limit: 6000

    # === IMU ===
    imu:
      i_batch_report_threshold: 1
      i_max_batch_reports: 10
      i_acc_freq: 400
      i_gyro_freq: 400
```

### Extracting Values from Your Calibration

If you re-calibrate and need to update the YAML, use this one-liner to extract the values:

```bash
# Print T_imu_cam[0] (left camera extrinsics)
python3 -c "
import json
with open('$DEV_HOME/basalt_calibration/stereo_imu_calibration_results/calibration.json') as f:
    cal = json.load(f)['value0']
t = cal['T_imu_cam'][0]
print(f'i_imu_extr_x: {t[\"px\"]}')
print(f'i_imu_extr_y: {t[\"py\"]}')
print(f'i_imu_extr_z: {t[\"pz\"]}')
print(f'i_imu_extr_qx: {t[\"qx\"]}')
print(f'i_imu_extr_qy: {t[\"qy\"]}')
print(f'i_imu_extr_qz: {t[\"qz\"]}')
print(f'i_imu_extr_qw: {t[\"qw\"]}')
print()
print(f'i_acc_bias: {cal[\"calib_accel_bias\"]}')
print(f'i_gyro_bias: {cal[\"calib_gyro_bias\"]}')
print(f'i_acc_noise_std: {cal[\"accel_noise_std\"]}')
print(f'i_gyro_noise_std: {cal[\"gyro_noise_std\"]}')
"
```

---

## Step 2: Tune the Basalt Algorithm Config (Optional)

The Basalt VIO algorithm is controlled by a separate JSON config file. The default is installed at:

```
depthai_ros_driver/config/custom/depthai_ros_driver_default_vio.json
```

The defaults work well for most cases. Only tune if VIO performance is poor.

### Key Parameters to Consider

| Parameter | Default | Description | When to Change |
|-----------|---------|-------------|----------------|
| `optical_flow_skip_frames` | 1 | Process every Nth frame | Increase if CPU-limited |
| `optical_flow_levels` | 3 | Pyramid levels for tracking | Increase for fast motion |
| `optical_flow_max_iterations` | 5 | LK iterations per level | Increase for better accuracy |
| `vio_max_states` | 3 | Camera poses in optimization window | Increase for more accuracy (slower) |
| `vio_max_kfs` | 7 | Maximum keyframes | Increase for larger environments |
| `vio_obs_std_dev` | 0.5 | Observation noise (pixels) | Increase if features are noisy |
| `vio_min_triangulation_dist` | 0.05 | Min baseline for triangulation (m) | Decrease for close objects |
| `optical_flow_image_safe_radius` | 472.0 | Radius for valid feature detection | Adjust for your resolution |

### Creating a Custom Config

```bash
# Copy the default
cp $DEV_HOME/ros2_ws/install/depthai_ros_driver/share/depthai_ros_driver/config/custom/depthai_ros_driver_default_vio.json \
   $DEV_HOME/ros2_ws/config/oak_ffc_3p_vio_config.json

# Edit as needed
# Then reference it in your vio.yaml:
#   i_config_path: "/absolute/path/to/oak_ffc_3p_vio_config.json"
```

### Recommended Adjustments for OAK-FFC-3P at 640x400

The `optical_flow_image_safe_radius` in the default config (472.0) is calibrated for ~640x480. For 640x400 images, you might reduce it slightly:

```json
"config.optical_flow_image_safe_radius": 350.0
```

This prevents feature detection near image borders where distortion is highest.

---

## Step 3: Launch VIO

### Option A: Using the VIO Launch File with Custom Config

```bash
source /opt/ros/jazzy/setup.bash
source $DEV_HOME/ros2_ws/install/setup.bash

ros2 launch depthai_ros_driver vio.launch.py \
  params_file:=$DEV_HOME/ros2_ws/src/depthai-ros/depthai_ros_driver/config/oak_ffc_3p_vio.yaml \
  use_rviz:=True
```

### Option B: Using the Standard Driver Launch with VIO Config

```bash
ros2 launch depthai_ros_driver driver.launch.py \
  params_file:=$DEV_HOME/ros2_ws/src/depthai-ros/depthai_ros_driver/config/oak_ffc_3p_vio.yaml
```

### Option C: VIO + Stereo Depth (Full Pipeline)

If you want both VIO odometry AND stereo depth output simultaneously, make sure your config has both `i_enable_vio: true` and stereo depth settings. The Stereo pipeline type supports this.

> **Note:** The RGBD pipeline type may cause a segfault on OAK-FFC-3P when VIO is enabled. Use `i_pipeline_type: Stereo` instead.

---

## Step 4: Verify VIO Output

### Check Topics

```bash
# List VIO-related topics
ros2 topic list | grep -E "vio|odom|imu"

# Expected output:
#   /oak/vio/odometry      (nav_msgs/msg/Odometry)
#   /oak/imu/data          (sensor_msgs/msg/Imu)
```

### Monitor Odometry

```bash
# Watch odometry messages
ros2 topic echo /oak/vio/odometry

# Check message rate (should be close to VIO FPS)
ros2 topic hz /oak/vio/odometry
```

### Verify TF Tree

```bash
# Check that odom → oak_parent_frame transform is published
ros2 run tf2_ros tf2_echo odom oak_parent_frame
```

### Visualize in RViz2

1. Open RViz2: `rviz2`
2. Set **Fixed Frame** to `odom`
3. Add display: **Odometry** → topic: `/oak/vio/odometry`
4. Add display: **TF** to see the transform tree
5. Move the camera — you should see the pose updating in real-time

### Quick Health Check

When VIO is working correctly:
- Odometry rate matches your configured FPS (~60 Hz)
- Position stays near origin when stationary (no drift)
- Position tracks smoothly when moving (no jumps)
- Orientation quaternion is valid (norm ≈ 1.0)

```bash
# Check quaternion validity
ros2 topic echo /oak/vio/odometry --field pose.pose.orientation --once
# qw² + qx² + qy² + qz² should ≈ 1.0
```

---

## Running VIO with RTABMap SLAM

The VIO output can feed directly into RTABMap for full visual SLAM:

```bash
ros2 launch depthai_ros_driver rtabmap.launch.py \
  params_file:=$DEV_HOME/ros2_ws/src/depthai-ros/depthai_ros_driver/config/oak_ffc_3p_vio.yaml
```

Or create a dedicated RTABMap+VIO config. The key is that RTABMap subscribes to `/oak/vio/odometry` for its odometry source.

See `depthai_ros_driver/launch/rtabmap.launch.py` and `depthai_ros_driver/config/rtabmap.yaml` for the full RTABMap integration.

---

## Parameter Reference

### VIO ROS Parameters (in `vio:` section of YAML)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `i_width` | int | 640 | VIO input image width |
| `i_height` | int | 400 | VIO input image height |
| `i_fps` | double | 60.0 | VIO framerate |
| `i_board_socket_id` | int | 1 (CAM_B) | Left camera board socket |
| `i_imu_update_rate` | int | 400 | IMU update rate (Hz) |
| `i_override_imu_extrinsics` | bool | false | Enable custom T_imu_cam |
| `i_imu_extr_{x,y,z}` | float | 0.0 | IMU-to-camera translation |
| `i_imu_extr_{qx,qy,qz,qw}` | float | identity | IMU-to-camera rotation |
| `i_set_acc_bias` | bool | false | Enable custom accel bias |
| `i_acc_bias` | double[9] | zeros | Accelerometer bias (9-element) |
| `i_set_gyro_bias` | bool | false | Enable custom gyro bias |
| `i_gyro_bias` | double[12] | zeros | Gyroscope bias (12-element) |
| `i_set_acc_noise_std` | bool | false | Enable custom accel noise |
| `i_acc_noise_std` | double[3] | zeros | Accel noise std dev [x,y,z] |
| `i_set_gyro_noise_std` | bool | false | Enable custom gyro noise |
| `i_gyro_noise_std` | double[3] | zeros | Gyro noise std dev [x,y,z] |
| `i_publish_tf` | bool | true | Publish TF transforms |
| `i_frame_id` | string | "odom" | Parent TF frame |
| `i_child_frame_id` | string | "oak_parent_frame" | Child TF frame |
| `i_config_path` | string | (built-in default) | Path to Basalt JSON config |
| `i_covariance` | double[36] | zeros | Odometry covariance matrix |
| `i_max_q_size` | int | 2 | Output queue size |

### Pipeline Parameters (in `pipeline_gen:` section)

| Parameter | Value | Required |
|-----------|-------|----------|
| `i_enable_vio` | true | Enables VIO node creation |
| `i_enable_imu` | true | Enables IMU (required for VIO) |
| `i_pipeline_type` | RGBD, Stereo, Depth, etc. | Any stereo-capable pipeline |

---

## Troubleshooting

### VIO Not Starting

| Symptom | Cause | Fix |
|---------|-------|-----|
| No `/oak/vio/odometry` topic | VIO not enabled | Set `i_enable_vio: true` in `pipeline_gen` |
| "BasaltVIO not found" error | depthai-core built without Basalt | Rebuild with `-DDEPTHAI_BASALT_SUPPORT=ON` |
| Crash on startup | Missing vcpkg Basalt dependencies | Rebuild depthai-core from source (see Prerequisites) |
| Segfault with RGBD pipeline | RGBD + VIO not supported on OAK-FFC-3P | Use `i_pipeline_type: Stereo` instead |
| Intermittent segfault after "Driver ready!" | Upstream depthai-core bugs | Apply patches from [Required depthai-core Patches](#required-depthai-core-patches) |
| IMU topic exists but no VIO | IMU not linked to VIO | Ensure `i_enable_imu: true` is set |

### Poor VIO Performance

| Symptom | Cause | Fix |
|---------|-------|-----|
| Large drift when stationary | Bad IMU calibration | Re-run Basalt IMU calibration, verify g-vector norm ≈ 9.81 |
| Jumps/discontinuities in pose | Feature tracking loss | Reduce motion speed, improve lighting, lower exposure limit |
| Slow/delayed odometry | CPU overloaded | Reduce `i_fps`, increase `optical_flow_skip_frames` |
| VIO diverges on fast rotation | Gyro noise too high | Verify `gyro_noise_std` < 0.002, check calibration quality |
| Constant position but rotation works | Insufficient translation in scene | Move camera, ensure features at multiple depths |

### Calibration Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| VIO worse with Basalt calibration | Calibration recording quality | Re-calibrate: ensure dynamic motion, good AprilGrid coverage |
| Quaternion warnings in logs | Invalid IMU extrinsics | Check that `i_imu_extr_q{x,y,z,w}` quaternion norm ≈ 1.0 |
| Scale drift | Accelerometer bias inaccurate | Re-run IMU calibration with proper motion (move camera, not board) |

### Verifying Calibration Values

```bash
# Check quaternion norm (should be ~1.0)
python3 -c "
import math
qx, qy, qz, qw = 0.49978286617904008, -0.5027659780640653, -0.5007287704258576, 0.49670328813382788
print(f'Quaternion norm: {math.sqrt(qx**2 + qy**2 + qz**2 + qw**2):.6f}')
"
# Should print: Quaternion norm: 0.999983 (very close to 1.0)

# Check stereo baseline from calibration
python3 -c "
import json, math
with open('$DEV_HOME/basalt_calibration/stereo_imu_calibration_results/calibration.json') as f:
    cal = json.load(f)['value0']
t0 = cal['T_imu_cam'][0]
t1 = cal['T_imu_cam'][1]
dx = t0['px'] - t1['px']
dy = t0['py'] - t1['py']
dz = t0['pz'] - t1['pz']
baseline = math.sqrt(dx**2 + dy**2 + dz**2)
print(f'Stereo baseline: {baseline*1000:.1f} mm')
"
# Should be ~100mm for OAK-FFC-3P stereo pair
```

---

## Source Code Reference

Key files in `depthai-ros` for VIO:

| File | Purpose |
|------|---------|
| `depthai_ros_driver/config/vio.yaml` | Default VIO config |
| `depthai_ros_driver/config/custom/depthai_ros_driver_default_vio.json` | Basalt algorithm parameters |
| `depthai_ros_driver/launch/vio.launch.py` | VIO launch file |
| `depthai_ros_driver/src/param_handlers/vio_param_handler.cpp` | Parameter declaration + application |
| `depthai_ros_driver/src/dai_nodes/sensors/vio.cpp` | VIO node wrapper (creates BasaltVIO, publishes odometry) |
| `depthai_ros_driver/src/pipeline/base_types.cpp` | Pipeline integration (conditional VIO creation) |
| `depthai_examples/src/odom_publisher.cpp` | Standalone VIO example |
| `depthai_examples/src/slam.cpp` | VIO + RTABMap SLAM example |
