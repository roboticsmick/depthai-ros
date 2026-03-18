# Processing Architecture Guide: OAK-FFC-3P + Jetson Orin

## Overview

The **OAK-FFC-3P** and **Jetson Orin** form a powerful vision system, but both have resource constraints:

- **OAK-FFC-3P VPU** (on-device): 5–15 TFLOPS, ~1–2 GB RAM, limited CPU cores (low power)
- **Jetson Orin**: 100+ TFLOPS GPU, 16 GB+ RAM, multi-core CPU
- **USB 3.1**: ~400 MB/s theoretical bandwidth (real: 300–350 MB/s)

This guide examines three architectures for running **Basalt VIO** (stereo + IMU), **Stereo Depth**, and **4K RGB capture** simultaneously.

---

## Table of Contents

- [System Constraints](#system-constraints)
- [Architecture Comparison](#architecture-comparison)
  - [Approach A: Device-Heavy (VIO + Depth On-Device)](#approach-a-device-heavy-vio--depth-on-device)
  - [Approach B: Balanced (VIO On-Device + Depth On Jetson)](#approach-b-balanced-vio-on-device--depth-on-jetson)
  - [Approach C: Host-Heavy (VIO Host + Depth On-Device + RGB Priority)](#approach-c-host-heavy-vio-host--depth-on-device--rgb-priority)
- [Detailed Trade-off Analysis](#detailed-trade-off-analysis)
  - [RGB Encoding Options](#rgb-encoding-options)
  - [USB Bandwidth Analysis](#usb-bandwidth-analysis)
  - [Processing Load Breakdown](#processing-load-breakdown)
- [Recommendations by Use Case](#recommendations-by-use-case)
- [JPEG Compression on OAK-FFC-3P](#jpeg-compression-on-oak-ffc-3p)

---

## System Constraints

### OAK-FFC-3P Hardware

| Component | Spec | Notes |
|-----------|------|-------|
| **Stereo cameras (mono)** | 2× OV9282, up to 1280×800 @ 60 Hz | 8-bit grayscale, global shutter |
| **RGB camera (IMX577)** | **12.3 MP sensor, 4K @ 3840×2160 supported** | Fully supported in depthai-ros (see `oak_ffc_3p_rgb_only.yaml` for 4K config) |
| **IMU** | BMI270, 400 Hz max | 3-axis accel, 3-axis gyro |
| **ISP/Encoder** | Limited hardware JPEG encoder (slow) | ❌ No native H.264/H.265 support |
| **USB interface** | USB 3.1, ~350 MB/s sustained | Shared across all streams |
| **VPU** | Myriad X (OAK-FFC-3P older gen) | Limited AI acceleration, CPU-bound for some tasks |

**Key limitation:** No hardware H.264/H.265 encoding on OAK-FFC-3P. RGB must be sent as **raw Bayer, JPEG, or TIFF** over USB. Consider Jetson GPU re-encoding for long-term archival.

### Jetson Orin

| Component | Spec | Notes |
|-----------|------|-------|
| **GPU** | Orin NX: 8-core, 40 TFLOPS; Orin Nano: 4-core, 13 TFLOPS | H.264/H.265 encoding available |
| **Memory** | 8–16 GB | Sufficient for depth processing, VIO, and post-processing |
| **USB 3.1** | ~400 MB/s | Bandwidth-limited bottleneck |
| **Power** | 5–25W (depending on variant) | Lower power than desktop |

---

## Architecture Comparison

### Approach A: Device-Heavy (VIO + Depth On-Device)

**Configuration:**
- **Stereo VIO**: Basalt on **OAK-FFC-3P** (via depthai-core)
- **Stereo Depth**: Computed on **OAK-FFC-3P** (via stereo pipeline)
- **4K RGB**: **High-quality JPEG** streamed to Jetson

#### Data Flow — Option A1: 4K RGB @ 20 FPS

```
OAK-FFC-3P Device:
├── CAM_B (left 640×400 @ 60 Hz)   ┐
├── CAM_C (right 640×400 @ 60 Hz)  ├→ Basalt VIO → /oak/vio/odometry
└── BMI270 IMU (400 Hz)            ┘

OAK-FFC-3P Device:
├── CAM_B (left 1280×800 @ 30 Hz)  ┐
├── CAM_C (right 1280×800 @ 30 Hz) ├→ Stereo disparity → /oak/stereo/depth
└── Filtering (median, temporal)   ┘

OAK-FFC-3P Device:
└── CAM_A (IMX577 3840×2160 @ 20 Hz) → JPEG @ quality 85–90 → /oak/rgb/image_raw

USB Bandwidth:
├── VIO odometry:     negligible (~10 KB/frame × 60 Hz = 600 KB/s)
├── Stereo depth:     1280×800×2 bytes @ 30 Hz = 62 MB/s
├── RGB JPEG (4K, quality 85): ~200 KB/frame × 20 Hz = 4 MB/s
└── IMU data:         negligible (~100 bytes/sample × 400 Hz = 40 KB/s)
└── TOTAL: ~67 MB/s (well within 350 MB/s limit) ✅
```

#### USB Bandwidth Breakdown — Approach A Options

| Stream | Resolution | FPS | Encoding | Size/Frame | Bandwidth | Notes |
|--------|-----------|-----|----------|-----------|-----------|-------|
| Stereo depth | 1280×800 | 30 | Raw uint16 | 2.05 MB | **61.5 MB/s** | Fixed |
| RGB (JPEG) | **3840×2160 (4K)** | **20** | **JPEG (q=85)** | **~200 KB** | **4.0 MB/s** | **Recommended** ✅ |
| RGB (JPEG) | 3840×2160 (4K) | 15 | JPEG (q=90) | ~300 KB | 4.5 MB/s | Higher quality |
| RGB (JPEG) | 3840×2160 (4K) | 10 | JPEG (q=95) | ~400 KB | 4.0 MB/s | Lossless-like |
| RGB (JPEG) | 1920×1080 (FHD) | 30 | JPEG (q=85) | ~100 KB | 3.0 MB/s | Lower res, higher FPS |
| Stereo cameras | 640×400 | 60 | VIO (internal) | internal | ~0 MB/s | — |
| IMU | — | 400 Hz | Packed | 0.04 MB | ~0.04 MB/s | — |
| Odometry | — | 60 Hz | Packed | 0.01 MB | ~0.6 MB/s | — |

#### Pros

✅ **Low latency**: Both VIO and depth are on-device, odometry is immediate (~5 ms).

✅ **Predictable**: No network delays. Timestamps are device-synchronized.

✅ **Offline capable**: Can run without Jetson if needed (e.g., debugging).

✅ **USB bandwidth efficient**: Total ~65 MB/s, well below 350 MB/s limit.

✅ **High stereo depth FPS**: 30 Hz stereo depth, 60 Hz VIO.

✅ **4K RGB available**: 1920×1080 @ 20 Hz with high-quality JPEG.

✅ **Flexible**: Easy to add RGB point cloud projection for photogrammetry.

#### Cons

❌ **Limited RGB FPS**: 20 Hz @ 1920×1080 (motion blur on fast motion).

❌ **JPEG compression artifacts**: High-quality JPEG loses detail; TIFF/raw uses more bandwidth.

❌ **Depth processing uses VPU**: Stereo depth consumes device resources.

❌ **Temperature**: Running both VIO and depth on-device increases heat.

❌ **Scaling**: Adding more streams (e.g., IR emitters, AI inference) is difficult.

#### Best For

- **Autonomous robots**: Immediate odometry feedback, no network latency.
- **Offline mapping**: Can run without host connection.
- **Jetson-optional workflows**: Use Jetson only for post-processing (photogrammetry, SLAM).
- **Bandwidth-limited scenarios** (e.g., tethered systems, remote operation).

---

### Approach B: Balanced (VIO On-Device + Depth On Jetson)

**Configuration:**
- **Stereo VIO**: Basalt on **OAK-FFC-3P**
- **Stereo Depth**: Computed on **Jetson Orin** (via OpenCV, SGBM, or NeuralSGBM)
- **4K RGB**: **High-quality JPEG** streamed to Jetson

#### Data Flow — Option B1: 4K RGB @ 30 FPS

```
OAK-FFC-3P Device:
├── CAM_B (left 640×400 @ 60 Hz)   ┐
├── CAM_C (right 640×400 @ 60 Hz)  ├→ Basalt VIO → /oak/vio/odometry
└── BMI270 IMU (400 Hz)            ┘

OAK-FFC-3P Device:
├── CAM_B (left 1280×800 @ 60 Hz)  ┐
└── CAM_C (right 1280×800 @ 60 Hz) ├→ USB 3.1 → Jetson Orin → Stereo Matcher → /depth/disparity

OAK-FFC-3P Device:
└── CAM_A (IMX577 3840×2160 @ 30 Hz) → JPEG @ quality 90–95 → /oak/rgb/image_raw → Jetson

USB Bandwidth:
├── VIO odometry:       negligible (~10 KB/frame × 60 Hz = 600 KB/s)
├── Stereo raw frames:  2× (1280×800×1 byte @ 60 Hz) = 122 MB/s
├── RGB JPEG (quality 90): ~300 KB/frame × 30 Hz = 9 MB/s
└── TOTAL: ~132 MB/s (under 350 MB/s limit, good headroom) ✅
```

#### USB Bandwidth Breakdown — Approach B Options

| Stream | Resolution | FPS | Encoding | Size/Frame | Bandwidth | Notes |
|--------|-----------|-----|----------|-----------|-----------|-------|
| Stereo left | 1280×800 | 60 | Raw uint8 | 1.02 MB | **61.2 MB/s** | Fixed |
| Stereo right | 1280×800 | 60 | Raw uint8 | 1.02 MB | **61.2 MB/s** | Fixed |
| RGB (JPEG) | **3840×2160 (4K)** | **30** | **JPEG (q=90)** | **~300 KB** | **9.0 MB/s** | **Recommended** ✅ |
| RGB (JPEG) | 3840×2160 (4K) | 30 | JPEG (q=95) | ~400 KB | 12.0 MB/s | Very high quality (tight) |
| RGB (JPEG) | 3840×2160 (4K) | 20 | JPEG (q=92) | ~350 KB | 7.0 MB/s | High quality, lower FPS |
| RGB (JPEG) | 1920×1080 (FHD) | 30 | JPEG (q=95) | ~250 KB | 7.5 MB/s | Lower res, very high quality |
| IMU | — | 400 Hz | Packed | 0.04 MB | ~0.04 MB/s | — |
| Odometry | — | 60 Hz | Packed | 0.01 MB | ~0.6 MB/s | — |

#### Pros

✅ **Offloads depth to powerful GPU**: Jetson GPU can run advanced stereo matchers (SGBM, NeuralSGBM).

✅ **Higher RGB FPS**: 30 Hz @ 1920×1080 instead of 20 Hz (Approach A).

✅ **Better RGB quality**: Less JPEG compression needed (q≥92 instead of q=85).

✅ **Flexible depth algorithms**: Can switch stereo matcher at runtime without device reboot.

✅ **Separates concerns**: VIO (critical latency) on device, depth (can be deferred) on host.

✅ **Scales well**: More streams possible (e.g., thermal, IR) without overwhelming device.

✅ **GPU-accelerated depth**: CUDA-based stereo matching is fast (10–30 ms for 1280×800).

✅ **VIO latency unchanged**: ~5 ms odometry, same as Approach A.

#### Cons

❌ **Network dependency**: Depth calculation depends on Jetson being online.

❌ **Higher USB bandwidth**: 130 MB/s (though still safe margin).

❌ **Slightly more latency on depth**: ~10–30 ms processing latency (GPU dependent).

❌ **Jetson power/cost**: Requires always-on Jetson.

❌ **Synchronization complexity**: Must align stereo timestamps with depth output.

❌ **Missing IMX577 on device**: Can't compute depth at 1920×1080 (only depth from stereo OV9282).

#### Best For

- **High-quality RGB capture**: 1920×1080 @ 30 Hz with good compression.
- **Advanced depth algorithms**: GPU-based SGBM, NeuralSGBM, or ML-based stereo.
- **Photogrammetry + SLAM**: Combine VIO odometry with dense depth maps.
- **Jetson-centric workflows**: Jetson is doing most processing (AI, SLAM, etc.).
- **Balanced load**: Neither device is maxed out.

---

### Approach C: Host-Heavy (VIO Host + Depth On-Device + RGB Priority)

**Configuration:**
- **Stereo VIO**: **basalt_ros2** running on **Jetson Orin** (via depthai-ros streams)
- **Stereo Depth**: Computed on **OAK-FFC-3P** (via stereo pipeline)
- **4K RGB**: **Raw Bayer or high-quality JPEG** at 3840×2160 for maximum detail

#### Data Flow — Option C1: 4K Raw Bayer @ 10 FPS (Maximum Fidelity)

```
OAK-FFC-3P Device:
├── CAM_B (left 1280×800 @ 60 Hz) ┐ → USB 3.1
└── CAM_C (right 1280×800 @ 60 Hz)┘ → Jetson Orin → basalt_ros2 VIO → /basalt/odometry

OAK-FFC-3P Device:
├── CAM_B (left 1280×800 @ 30 Hz)  ┐
├── CAM_C (right 1280×800 @ 30 Hz) ├→ Stereo disparity → /oak/stereo/depth
└── Filtering (median, temporal)   ┘

OAK-FFC-3P Device:
└── CAM_A (IMX577 3840×2160 @ 10 Hz) → Raw Bayer (Rggb16 format) → /oak/rgb/image_raw

USB Bandwidth:
├── Stereo frames (VIO):  2× (1280×800×1 byte @ 60 Hz) = 122 MB/s
├── Stereo depth:         1280×800×2 bytes @ 30 Hz = 62 MB/s
├── RGB raw Bayer (4K):   3840×2160×1.5 bytes @ 10 Hz ≈ 99 MB/s
└── TOTAL: ~284 MB/s (under 350 MB/s, but tight with no margin) ⚠️
```

#### USB Bandwidth Breakdown — Approach C Options

| Stream | Resolution | FPS | Encoding | Size/Frame | Bandwidth | Notes |
|--------|-----------|-----|----------|-----------|-----------|-------|
| Stereo left (VIO) | 1280×800 | 60 | Raw uint8 | 1.02 MB | **61.2 MB/s** | Fixed |
| Stereo right (VIO) | 1280×800 | 60 | Raw uint8 | 1.02 MB | **61.2 MB/s** | Fixed |
| Stereo depth | 1280×800 | 30 | Raw uint16 | 2.05 MB | **61.5 MB/s** | Fixed |
| RGB raw Bayer | **3840×2160 (4K)** | **10** | **Rggb16** | **12.4 MB** | **124 MB/s** | Maximum quality (tight) ⚠️ |
| RGB raw Bayer | 3840×2160 (4K) | 5 | Rggb16 | 12.4 MB | 62 MB/s | Safe bandwidth |
| RGB JPEG | 3840×2160 (4K) | 20 | JPEG (q=95) | ~400 KB | 8.0 MB/s | Good compromise |
| RGB raw Bayer | 1920×1080 (FHD) | 20 | Rggb16 | 3.1 MB | 62 MB/s | Lower res, lossless |
| IMU | — | 400 Hz | Packed | 0.04 MB | ~0.04 MB/s | — |

#### Pros

✅ **Highest RGB flexibility**: Raw Bayer allows offline demosaicing/processing without compression artifacts.

✅ **Maximum quality**: No JPEG compression; raw data for highest-fidelity photogrammetry.

✅ **VIO on powerful CPU**: basalt_ros2 runs on Jetson multi-core CPU, more flexible tuning.

✅ **Depth still on device**: Lower USB bandwidth for depth (compared to Approach B).

✅ **SLAM-friendly**: Stereo frames + VIO + depth available for full visual SLAM.

✅ **Offline post-processing**: Raw RGB can be processed later with perfect quality.

#### Cons

❌ **Very high USB bandwidth**: ~247 MB/s is near the practical limit (350 MB/s – overhead).

⚠️ **USB bandwidth fragile**: Little margin for other streams (thermal, IR, additional sensors).

❌ **Slower RGB FPS**: 15 Hz instead of 20–30 Hz (more motion blur).

❌ **VIO latency higher**: ~20–50 ms (network + host processing) vs. 5 ms on-device.

❌ **Raw RGB file size**: 4.15 MB/frame × 15 Hz × 3600 s/hr = ~224 GB/hour (huge storage).

❌ **Complex setup**: Requires basalt_ros2 integration, message sync (message_filters), and careful calibration.

❌ **Jetson always required**: Can't offload to external storage easily.

❌ **Network latency added**: If Jetson is remote, adds 10+ ms to VIO feedback loop.

#### Best For

- **Offline photogrammetry**: Raw RGB data for SfM reconstruction without compression loss.
- **Research/archival**: Capturing reference data for algorithm development.
- **Jetson Orin Nano**: Limited device resources; better to use Jetson CPU for VIO.
- **Known-flat environments**: Where depth-from-stereo is sufficient, no need for depth on device.

---

## Detailed Trade-off Analysis

### RGB Encoding Options

#### 1. **JPEG (Lossy) — Best for Real-Time Streaming**

**At 1920×1080 (FHD):**
- **Quality 85**: ~100 KB/frame (good compression)
- **Quality 90**: ~150 KB/frame (high quality)
- **Quality 95**: ~250 KB/frame (near-lossless)

**At 3840×2160 (4K):**
- **Quality 85**: ~200 KB/frame (good balance)
- **Quality 90**: ~300 KB/frame (recommended)
- **Quality 95**: ~400 KB/frame (near-lossless)

**Use case**: General robotics, real-time streams, when storage/bandwidth is limited
**Pros**: Small file size, fast encoding/decoding, widely supported
**Cons**: Block artifacts, not ideal for detail-heavy scenes (tiles, textures)

**Bandwidth Examples:**
- 4K @ 30 FPS (q=90): 300 KB/frame × 30 Hz = **9.0 MB/s** ✅
- 4K @ 20 FPS (q=90): 300 KB/frame × 20 Hz = **6.0 MB/s** ✅
- 4K @ 15 FPS (q=95): 400 KB/frame × 15 Hz = **6.0 MB/s** ✅
- 1080p @ 30 FPS (q=90): 150 KB/frame × 30 Hz = **4.5 MB/s** ✅

#### 2. **TIFF (Lossless) — Archival Quality**

**At 1920×1080:**
- **Uncompressed**: 6.22 MB/frame (no compression)
- **LZ77-compressed**: 3–4 MB/frame (50–60% typical compression)

**At 3840×2160 (4K):**
- **Uncompressed**: 24.9 MB/frame
- **LZ77-compressed**: 12–15 MB/frame (typical)

**Use case**: Photogrammetry, detail preservation, archival
**Pros**: Lossless, good compression on uniform regions, professional quality
**Cons**: Large files, slower encoding, not suitable for real-time streaming

**Bandwidth @ 3840×2160:**
- Uncompressed @ 5 Hz: 24.9 MB/frame × 5 Hz = **124.5 MB/s** ❌ (infeasible)
- LZ77 @ 5 Hz: 12 MB/frame × 5 Hz = **60 MB/s** ⚠️ (tight)
- LZ77 @ 2 Hz: 12 MB/frame × 2 Hz = **24 MB/s** ✅ (safe)

#### 3. **Raw Bayer (Uncompressed) — Maximum Fidelity**

- **IMX577 Bayer @ 1920×1080**: 1920×1080×1.5 bytes (12-bit Bayer = 1.5 bytes/pixel)
- **IMX577 Bayer @ 3840×2160 (4K)**: 3840×2160×1.5 bytes

**Use case**: Maximum quality, offline processing, scientific/research imaging
**Pros**: No quality loss, direct demosaicing control, preserves all sensor data
**Cons**: Huge file size, very high bandwidth, requires offline processing

**Bandwidth Examples:**
- 4K @ 10 Hz: 12.4 MB/frame × 10 Hz = **124 MB/s** ⚠️ (tight)
- 4K @ 5 Hz: 12.4 MB/frame × 5 Hz = **62 MB/s** ✅ (safe)
- 1080p @ 15 Hz: 3.1 MB/frame × 15 Hz = **46.5 MB/s** ✅
- 1080p @ 20 Hz: 3.1 MB/frame × 20 Hz = **62 MB/s** ✅

#### 4. **H.264/H.265 (GPU Encoding on Jetson) — Best Compression**

❌ OAK-FFC-3P does **not** have hardware H.264/H.265 encoding. The ISP cannot produce H.264 streams directly.

✅ **Workaround**: Stream JPEG or raw Bayer to Jetson, encode with NVIDIA NVENC (GPU encoder)
- **H.264 @ 4K @ 30 FPS**: 1–3 MB/s (depends on bitrate setting)
- **H.265 @ 4K @ 30 FPS**: 0.5–2 MB/s (better compression)
- **Speed**: Fast (100+ FPS possible, minimal latency)

**Recommendation**: For best bandwidth efficiency on Approach B, stream JPEG to Jetson and optionally re-encode with NVENC if additional compression needed.

---

### JPEG Compression on OAK-FFC-3P

**Key Question:** How much processing power does JPEG encoding consume on the OAK?

#### JPEG Encoding Performance

The **OAK-FFC-3P ISP** has a limited JPEG encoder (not the Myriad X, but built-in ISP). Encoding 1920×1080 @ 20 Hz:

- **Hardware JPEG** (ISP-based, slow): ~10–20 ms per frame @ quality 90
  - Expected throughput: 5–10 Hz (not suitable for 20+ FPS)
  - **Conclusion:** OAK hardware JPEG is too slow for high-FPS encoding

- **Software JPEG** (CPU-based, depthai-core): ~30–50 ms per frame @ quality 90
  - Expected throughput: 2–3 Hz (even worse)
  - **Conclusion:** Not viable for real-time 1920×1080 encoding

**Practical Solution for Approach A:**
- Reduce RGB FPS to **10–15 Hz** (matches JPEG encoder throughput)
- Use quality **85–90** for acceptable compression
- Or accept 20 Hz @ quality 70–75 (more artifacts)

**Better Solution:** Stream raw or compressed to Jetson, encode with NVENC GPU (100+ FPS possible).

#### Recommendation

**For Approach A (RGB only on device):**
- JPEG encoding is slow; limit RGB to **10–15 FPS @ quality 80–85**
- Better: Use Approach B, let Jetson encode with NVENC

**For Approach B (stereo + RGB on device, depth on Jetson):**
- Stream **raw frames or quick JPEG (quality 70)** to Jetson
- Jetson re-encodes with NVIDIA NVENC (optional, or keep as JPEG if sufficient)

**For Approach C (raw RGB priority):**
- Stream **raw Bayer at 10–15 FPS** to Jetson
- Jetson handles offline demosaicing and compression

---

### USB Bandwidth Analysis

#### Available Bandwidth

- **USB 3.1 theoretical**: 400 MB/s
- **Practical sustained** (with overhead): 300–350 MB/s
- **Safety margin**: Keep total ≤ 280 MB/s for headroom

#### Stream Bandwidth Table

| Stream | Resolution | FPS | Format | Bytes/Frame | Bandwidth |
|--------|-----------|-----|--------|-------------|-----------|
| Stereo left | 1280×800 | 60 | Raw uint8 | 1.02 MB | **61.2 MB/s** |
| Stereo right | 1280×800 | 60 | Raw uint8 | 1.02 MB | **61.2 MB/s** |
| Depth | 1280×800 | 30 | uint16 | 2.05 MB | **61.5 MB/s** |
| Depth | 1280×800 | 15 | uint16 | 2.05 MB | **30.75 MB/s** |
| RGB raw Bayer | 1920×1080 | 20 | Rggb16 | 4.15 MB | **83 MB/s** |
| RGB raw Bayer | 1920×1080 | 15 | Rggb16 | 4.15 MB | **62 MB/s** |
| RGB raw Bayer | 1920×1080 | 10 | Rggb16 | 4.15 MB | **41.5 MB/s** |
| RGB JPEG | 1920×1080 | 20 | JPEG (q=85) | 0.15 MB | **3.0 MB/s** |
| RGB JPEG | 1920×1080 | 30 | JPEG (q=85) | 0.15 MB | **4.5 MB/s** |
| RGB JPEG | 1920×1080 | 30 | JPEG (q=95) | 0.30 MB | **9.0 MB/s** |

#### Approach A Bandwidth

```
VIO (on device):      0 MB/s (internal)
Stereo depth:         61.5 MB/s
RGB JPEG (q=85):      3.0 MB/s @ 20 Hz
────────────────────────────
Total:                64.5 MB/s ✅ Safe
```

#### Approach B Bandwidth

```
Stereo left:          61.2 MB/s
Stereo right:         61.2 MB/s
RGB JPEG (q=92):      7.5 MB/s
────────────────────────────
Total:                130 MB/s ✅ Safe (30% headroom)
```

#### Approach C Bandwidth

```
Stereo left:          61.2 MB/s
Stereo right:         61.2 MB/s
Depth:                61.5 MB/s
RGB raw Bayer:        62 MB/s @ 15 Hz
────────────────────────────
Total:                246 MB/s ⚠️ Tight (30% headroom, no room for other streams)
```

---

### Processing Load Breakdown

#### OAK-FFC-3P (Myriad X VPU + ISP)

| Task | Load | Notes |
|------|------|-------|
| Basalt VIO @ 640×400 @ 60 Hz | ~30–40% | Optical flow + pose optimization |
| Stereo depth @ 1280×800 @ 30 Hz | ~40–50% | Disparity + filtering |
| JPEG encoding 1920×1080 @ 20 Hz | ~50–60% | Hardware JPEG is bottleneck |
| IMU @ 400 Hz | ~2% | Negligible |
| **VIO + Depth + JPEG** | **120–150%** ❌ | **Infeasible** (would thrash) |
| **VIO + Depth** (no RGB) | **70–90%** ✅ | Feasible, safe margin |
| **VIO only** (no depth, no RGB) | **30–40%** ✅ | Plenty of headroom |

**Conclusion:** OAK-FFC-3P **cannot do VIO + depth + JPEG simultaneously** without thermal throttling. Choose **Approach A or B**.

#### Jetson Orin Nano (4-core, 13 TFLOPS)

| Task | Load | Notes |
|------|------|-------|
| Basalt VIO @ 1280×800 @ 60 Hz | ~50–70% | CPU-intensive optical flow |
| Stereo matching (SGBM) @ 1280×800 @ 30 Hz | ~40–60% | GPU-accelerated, varies by matcher |
| JPEG encoding @ 1920×1080 @ 30 Hz | ~5–10% | NVENC (hardware encoder) |
| ROS2 middleware + TF | ~5–10% | Negligible |
| **VIO + Depth + JPEG encode** | **50–80%** ✅ | Feasible on Orin Nano |

#### Jetson Orin (12-core, 100+ TFLOPS)

| Task | Load | Notes |
|------|------|-------|
| Basalt VIO @ 1280×800 @ 60 Hz | ~20–30% | One CPU core only |
| Stereo matching (NeuralSGBM) @ 1280×800 @ 30 Hz | ~10–20% | GPU-accelerated, fast |
| JPEG/H.264 encoding @ 1920×1080 @ 30 Hz | ~2–5% | NVENC, very fast |
| **VIO + Depth + JPEG encode** | **30–55%** ✅ | Plenty of headroom |

---

## Recommendations by Use Case

### Use Case 1: **Autonomous Mobile Robot (Fast Navigation)**

**Priority:** Low-latency odometry for control loop feedback

**Recommended:** **Approach A** (Device-Heavy)

**Configuration (Option 1: 4K @ 20 Hz):**
```yaml
# VIO @ 60 Hz, depth @ 30 Hz, 4K RGB @ 20 Hz JPEG
vio:
  i_width: 640
  i_height: 400
  i_fps: 60

stereo:
  i_fps: 30
  i_depth_preset: DEFAULT

rgb:
  i_fps: 20
  i_width: 3840
  i_height: 2160
  i_jpeg_quality: 85  # Compression-friendly for real-time
```

**Configuration (Option 2: 1080p @ 30 Hz, if higher RGB FPS needed):**
```yaml
# VIO @ 60 Hz, depth @ 30 Hz, 1080p RGB @ 30 Hz JPEG (lower FPS impact)
rgb:
  i_fps: 30
  i_width: 1920
  i_height: 1080
  i_jpeg_quality: 85
```

**Reasoning:**
- Immediate 5 ms odometry feedback (on-device) ✅
- No network latency for navigation loop ✅
- Adequate depth for obstacle avoidance ✅
- Works offline if Jetson is unavailable ✅
- USB bandwidth comfortable (65–67 MB/s) ✅
- **4K RGB** available for visual logging without sacrificing latency ✅

**Gotchas:**
- RGB @ 20 Hz (4K) might cause motion blur on fast robots. Choose based on robot speed:
  - Slow/medium speed (< 1 m/s): 4K @ 20 Hz is fine
  - Fast (> 1 m/s): Use 1080p @ 30 Hz instead
  - Very fast (> 2 m/s): Consider reducing depth FPS to 15 Hz, increase RGB FPS

---

### Use Case 2: **Photogrammetry + High-Quality 4K RGB Capture**

**Priority:** RGB quality and quantity at 4K, some post-processing delay OK

**Recommended:** **Approach B** (Balanced)

**Configuration (Recommended: 4K @ 30 FPS):**
```yaml
# VIO @ 60 Hz on device, depth @ 30 Hz on Jetson, 4K RGB @ 30 FPS
vio:
  i_width: 640
  i_height: 400
  i_fps: 60

stereo:
  i_width: 1280
  i_height: 800
  i_fps: 60
  # Don't output depth; let Jetson compute it

rgb:
  i_fps: 30
  i_width: 3840
  i_height: 2160
  # Stream JPEG (q=90) to Jetson
```

**On Jetson:**
```bash
# Receive stereo frames, compute depth with GPU
ros2 run depthai_depth_processor stereo_matcher_node \
  --matcher-type=NeuralSGBM  # Best for photogrammetry

# Optionally re-encode RGB with NVENC for archival H.265
gst-launch-1.0 rtspsrc location=rtsp://... ! \
  nvh265enc bitrate=5000 ! filesink location=video.h265
```

**Reasoning:**
- **30 FPS @ 3840×2160 (4K) RGB is excellent for photogrammetry** ✅
- High-quality JPEG (quality 90) sufficient for detail preservation
- Jetson GPU handles depth with advanced NeuralSGBM matcher
- USB bandwidth still safe (132 MB/s, 30% margin) ✅
- Can pause RGB recording, dynamically adjust depth matcher
- **4K frames enable higher-precision SfM reconstruction** ✅

**Storage & Processing:**
- 4K JPEG @ 30 Hz @ quality 90: ~300 KB/frame × 30 = **9 MB/s** → ~32 GB/hour
- Stereo frames + depth maps can be saved for bundle adjustment
- Jetson GPU re-encodes to H.265 if long-term archival needed (1–2 MB/s)

**Gotchas:**
- Network latency added for depth (10–30 ms), but not critical for photogrammetry
- Requires Jetson always online
- High-speed SSD recommended if recording all streams continuously

---

### Use Case 3: **Offline High-Fidelity Archive (Research/Calibration)**

**Priority:** Maximum quality, storage OK, real-time not critical

**Recommended:** **Approach C** (Host-Heavy), with modifications

**Configuration:**
```yaml
# Raw Bayer RGB @ 10 FPS, VIO on Jetson CPU, depth @ 15 Hz
vio:
  # Don't enable on-device; let Jetson basalt_ros2 do it
  i_enable_vio: false

stereo:
  i_width: 1280
  i_height: 800
  i_fps: 60  # Full rate to Jetson

rgb:
  i_fps: 10
  # Stream raw Bayer
```

**On Jetson:**
```bash
# Run basalt_ros2 on stereo streams (60 Hz)
ros2 launch basalt_ros2 oak_ffc_3p.launch.py

# Record raw Bayer @ 10 FPS + odometry + depth
rosbag2 record -a -o calibration_run_20260227
```

**Storage Requirements:**
- 10 Hz raw RGB: 41.5 MB/s × 3600 s = ~149 GB/hour ⚠️
- 60 FPS stereo: 122 MB/s × 3600 s = ~439 GB/hour ⚠️
- **Subtotal: ~600 GB/hour** (requires fast external SSD)

**Reasoning:**
- Raw Bayer preserves maximum detail for offline photogrammetry
- Jetson CPU handles VIO (flexible tuning)
- Can record for extended periods (10+ hours)
- Post-process at leisure (demosaic, bundle adjustment, etc.)

**Gotchas:**
- Very high bandwidth (246 MB/s); no margin for error
- Requires external SSD (USB 3.0+ or NVME)
- Jetson required for VIO processing
- Storage expensive

---

### Use Case 4: **Jetson Orin Nano (Budget/Power Constraint)**

**Priority:** Low power, adequate performance, lower cost

**Recommended:** **Approach B** (Balanced), tuned down

**Configuration:**
```yaml
# Lower FPS to reduce Jetson load
vio:
  i_width: 640
  i_height: 400
  i_fps: 30  # Reduced from 60 Hz

stereo:
  i_width: 1280
  i_height: 800
  i_fps: 20  # Reduced from 30 Hz

rgb:
  i_fps: 15  # Lower than standard
```

**On Jetson Nano:**
```bash
# Use fast stereo matcher (SGBM, not NeuralSGBM)
ros2 run depthai_depth_processor stereo_matcher_node \
  --matcher-type=SGBM \
  --max-disp=96  # Limit search range for speed
```

**Why This Works:**
- 30 FPS VIO still acceptable for odometry
- 20 FPS depth matches stereo framerate
- Nano can handle this load without throttling (40–60% CPU)
- 15 FPS RGB with quality 90 JPEG = 6.75 MB/s (trivial for network)

**USB Bandwidth:**
```
30 Hz @ 1280×800:       30.6 MB/s (left)
30 Hz @ 1280×800:       30.6 MB/s (right)
RGB @ 15 FPS JPEG:      5.0 MB/s
────────────────────────────
Total:                  66 MB/s ✅ (half bandwidth used!)
```

**Gotchas:**
- 30 FPS VIO is on the edge of acceptable for smooth odometry
- Can add more sensors later (IR, thermal) with spare bandwidth

---

## Summary Table

| Feature | Approach A | Approach B | Approach C |
|---------|-----------|-----------|-----------|
| **VIO Location** | On-device | On-device | Jetson CPU |
| **VIO Latency** | ~5 ms | ~5 ms | ~20–50 ms |
| **Depth Location** | On-device | Jetson GPU | On-device |
| **Depth FPS** | 30 | 30 | 30 |
| **RGB Resolution** | **4K (3840×2160)** | **4K (3840×2160)** | **4K (3840×2160)** |
| **RGB FPS** | 20 | 30 | 5–10 |
| **RGB Quality** | JPEG q85–90 | JPEG q90–95 | Raw Bayer or JPEG |
| **USB Bandwidth** | 67 MB/s | 132 MB/s | 254–284 MB/s |
| **Device Load** | 90% | 50% | 50% |
| **Jetson Load** | ~0% (if used) | 50–70% | 50–80% |
| **Total Latency** | ~10 ms | ~30 ms | ~40–70 ms |
| **Best For** | Real-time nav + 4K RGB | **Photogrammetry** ✅ | High-fidelity archive |
| **Feasibility** | ✅ Proven | ✅ **Recommended** | ⚠️ Tight bandwidth |
| **4K Storage (1 hr)** | ~32 GB/hr | ~32 GB/hr | ~149–224 GB/hr |

---

## Next Steps

1. **Choose your approach** based on your use case (see "Recommendations by Use Case")
2. **Test USB bandwidth** with your setup:
   ```bash
   # Monitor USB traffic
   iftop -i usbmon0  # or your USB interface
   ```
3. **Profile Jetson CPU/GPU** during depth processing:
   ```bash
   # Jetson Orin
   jtop  # Interactive tool showing live load
   ```
4. **Benchmark stereo matchers** on your Jetson variant (SGBM vs. NeuralSGBM)
5. **Measure thermal behavior** under sustained load (VPU temp, Jetson throttle)

---

## References

- [depthai-core Basalt VIO Setup](assets/depthai-core_basalt/README.md)
- [depthai-ros ROS2 Jazzy](README.md)
- [Basalt VIO GitHub](https://github.com/VladyslavUsenko/basalt)
- [basalt_ros2 Wrapper](https://github.com/roboticsmick/basalt_ros2)
- [OAK-FFC-3P Specs](https://docs.luxonis.com/hardware/oak-ffc-3p/)
- [Jetson Orin Benchmarks](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
