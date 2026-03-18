#include "depthai/basalt/BasaltVIO.hpp"

#include <chrono>
#include <limits>
#include <thread>

#include "../utility/PimplImpl.hpp"
#include "basalt/calibration/calibration.hpp"
#include "basalt/serialization/headers_serialization.h"
#include "basalt/spline/se3_spline.h"
#include "basalt/utils/vio_config.h"
#include "basalt/vi_estimator/vio_estimator.h"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/VioHealthData.hpp"
#include "pipeline/datatype/TransformData.hpp"
#include "tbb/concurrent_queue.h"
#include "tbb/global_control.h"
namespace dai {

namespace node {

class BasaltVIO::Impl {
   public:
    Impl() = default;
    std::shared_ptr<tbb::concurrent_bounded_queue<basalt::OpticalFlowInput::Ptr>> imageDataQueue;
    std::shared_ptr<tbb::concurrent_bounded_queue<basalt::ImuData<double>::Ptr>> imuDataQueue;
    std::shared_ptr<tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr>> outStateQueue;
    std::shared_ptr<tbb::concurrent_bounded_queue<basalt::VioVisualizationData::Ptr>> outVisQueue;
    std::shared_ptr<tbb::detail::d1::global_control> tbbGlobalControl;
    std::shared_ptr<basalt::Calibration<double>> calib;

    basalt::OpticalFlowBase::Ptr optFlowPtr;
    basalt::VioEstimatorBase::Ptr vio;
    basalt::OpticalFlowInput::Ptr lastImgData;

    // Monotonicity guards — Basalt asserts strictly increasing timestamps.
    // The OAK-FFC-3P (separate camera modules) occasionally produces out-of-order
    // or duplicate timestamps, especially in the first few frames.  We filter
    // at the source so the assertions never trigger.
    // Each field is only accessed from one callback thread, so no extra locking needed.
    int64_t lastFrameTNs{std::numeric_limits<int64_t>::min()};
    int64_t lastImuTNs{std::numeric_limits<int64_t>::min()};
    /**
     * VIO configuration file.
     */
    basalt::VioConfig vioConfig;

    std::vector<int64_t> vioTNSec;
    std::shared_ptr<basalt::PoseState<double>::SE3> localTransform;
};

BasaltVIO::BasaltVIO() {}

BasaltVIO::~BasaltVIO() = default;

void BasaltVIO::buildInternal() {
    sync->out.link(inSync);
    inSync.addCallback(std::bind(&BasaltVIO::stereoCB, this, std::placeholders::_1));
    imu.addCallback(std::bind(&BasaltVIO::imuCB, this, std::placeholders::_1));

    basalt::PoseState<double>::SE3 initTrans(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0, 0, 0));
    Eigen::Matrix<double, 3, 3> R;
    R << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::Quaterniond q(R);
    basalt::PoseState<double>::SE3 initialRotation(q, Eigen::Vector3d(0, 0, 0));
    // to output pose in FLU world coordinates
    pimpl->localTransform = std::make_shared<basalt::PoseState<double>::SE3>(initTrans * initialRotation.inverse());
    setDefaultVIOConfig();
}

void BasaltVIO::setLocalTransform(const std::shared_ptr<TransformData>& transform) {
    auto trans = transform->getTranslation();
    auto quat = transform->getQuaternion();
    pimpl->localTransform =
        std::make_shared<basalt::PoseState<double>::SE3>(Eigen::Quaterniond(quat.qw, quat.qx, quat.qy, quat.qz), Eigen::Vector3d(trans.x, trans.y, trans.z));
}
void BasaltVIO::run() {
    basalt::PoseVelBiasState<double>::Ptr data;
    Eigen::Matrix<double, 3, 3> R;
    R << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
    Eigen::Quaterniond q(R);
    basalt::PoseState<double>::SE3 opticalTransform(q, Eigen::Vector3d(0, 0, 0));

    while(isRunning()) {
        if(!initialized.load(std::memory_order_acquire)) {
            // Yield the CPU while waiting for initialization so that startup
            // threads (ROS2 executor, TBB scheduler, camera callbacks) are not
            // starved.  The first VIO state is delayed by at most 1 ms, which
            // is negligible compared to VIO cycle time (~50 ms at 20 Hz).
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        pimpl->outStateQueue->pop(data);

        if(!data.get()) continue;
        basalt::PoseState<double>::SE3 pose = (*pimpl->localTransform * data->T_w_i * pimpl->calib->T_i_c[0]);

        // pose is in RDF orientation, convert to FLU
        auto finalPose = pose * opticalTransform.inverse();
        auto trans = finalPose.translation();
        auto rot = finalPose.unit_quaternion();
        auto out = std::make_shared<TransformData>(trans.x(), trans.y(), trans.z(), rot.x(), rot.y(), rot.z(), rot.w());

        // ── Build VioHealthData ────────────────────────────────────────────
        auto healthOut = std::make_shared<VioHealthData>();
        healthOut->ts = out->ts;
        healthOut->tsDevice = out->tsDevice;

        // Tier 1: velocity in FLU world frame.
        // vel_w_i is in the VIO internal world frame; localTransform rotation
        // maps it into the same FLU world frame used by the pose output.
        Eigen::Vector3d vel_flu = pimpl->localTransform->rotationMatrix() * data->vel_w_i;
        healthOut->velX = vel_flu.x();
        healthOut->velY = vel_flu.y();
        healthOut->velZ = vel_flu.z();

        // Tier 1: IMU biases (in IMU sensor frame, not transformed).
        healthOut->gyroBiasX  = data->bias_gyro.x();
        healthOut->gyroBiasY  = data->bias_gyro.y();
        healthOut->gyroBiasZ  = data->bias_gyro.z();
        healthOut->accelBiasX = data->bias_accel.x();
        healthOut->accelBiasY = data->bias_accel.y();
        healthOut->accelBiasZ = data->bias_accel.z();

        // Tier 2: optical-flow tracking quality (non-blocking).
        basalt::VioVisualizationData::Ptr visData;
        if(pimpl->outVisQueue && pimpl->outVisQueue->try_pop(visData) && visData) {
            if(visData->opt_flow_res && !visData->opt_flow_res->keypoints.empty()) {
                healthOut->numTrackedFeatures = static_cast<int32_t>(visData->opt_flow_res->keypoints[0].size());

                if(!visData->opt_flow_res->keypoint_responses.empty()) {
                    float sumResp = 0.0f;
                    int32_t count = 0;
                    for(const auto& kv : visData->opt_flow_res->keypoint_responses[0]) {
                        sumResp += kv.second;
                        ++count;
                    }
                    healthOut->meanFeatureResponse = count > 0 ? sumResp / static_cast<float>(count) : 0.0f;
                }
            }
            healthOut->numLandmarks = static_cast<int32_t>(visData->points.size());
        }
        // ──────────────────────────────────────────────────────────────────

        transform.send(out);
        health.send(healthOut);

        // Note: Passthrough of leftImg is disabled to avoid thread-safety issues.
        // The image data is safely accessible through VioHealthData output.
        // Enabling passthrough caused double-free crashes due to concurrent
        // stereoCB and run() threads both managing the shared_ptr refcount.
    }
}

void BasaltVIO::stereoCB(std::shared_ptr<ADatatype> in) {
    auto group = std::dynamic_pointer_cast<MessageGroup>(in);
    if(group == nullptr) return;
    if(!initialized.load(std::memory_order_acquire)) {
        std::vector<std::shared_ptr<ImgFrame>> imgFrames;
        for(auto& msg : *group) {
            imgFrames.emplace_back(std::dynamic_pointer_cast<ImgFrame>(msg.second));
        }

        initialize(imgFrames);
    }
    int i = 0;
    basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput(2));
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
            // Defensive: avoid buffer overrun if frame data length disagrees with width*height
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
    // Basalt asserts strictly increasing frame timestamps.
    // OAK-FFC-3P cameras can produce duplicate or backwards timestamps at startup.
    // Drop any frame that does not strictly advance the timestamp.
    if(data->t_ns <= pimpl->lastFrameTNs) {
        return;
    }
    pimpl->lastFrameTNs = data->t_ns;

    pimpl->lastImgData = data;
    if(pimpl->imageDataQueue) {
        pimpl->imageDataQueue->push(data);
    }
};

void BasaltVIO::imuCB(std::shared_ptr<ADatatype> imuData) {
    auto imuPackets = std::dynamic_pointer_cast<IMUData>(imuData);

    for(auto& imuPacket : imuPackets->packets) {
        basalt::ImuData<double>::Ptr data;
        data = std::make_shared<basalt::ImuData<double>>();
        auto t = imuPacket.acceleroMeter.getTimestamp();
        int64_t t_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(t).time_since_epoch().count();

        // Basalt asserts IMU timestamps are strictly increasing relative to the
        // current VIO state.  The OAK device can deliver negative or duplicate
        // timestamps during clock-sync warm-up.  Drop any non-advancing packet.
        if(t_ns <= pimpl->lastImuTNs) continue;
        pimpl->lastImuTNs = t_ns;

        data->t_ns = t_ns;
        data->accel = Eigen::Vector3d(imuPacket.acceleroMeter.x, imuPacket.acceleroMeter.y, imuPacket.acceleroMeter.z);
        data->gyro = Eigen::Vector3d(imuPacket.gyroscope.x, imuPacket.gyroscope.y, imuPacket.gyroscope.z);
        if(pimpl->imuDataQueue) pimpl->imuDataQueue->push(data);
    }
};

void BasaltVIO::stop() {
    // Signal Basalt's internal threads to shut down by sending sentinel nullptrs.
    // The VIO estimator pushes nullptr to outStateQueue on exit, which unblocks
    // run()'s blocking pop().
    if(pimpl->imageDataQueue) pimpl->imageDataQueue->push(nullptr);
    if(pimpl->imuDataQueue) pimpl->imuDataQueue->push(nullptr);

    // Drain the vis queue so Basalt's shutdown nullptr push doesn't block.
    // (sqrt_keypoint_vio.cpp line 352 pushes nullptr to out_vis_queue at exit;
    //  if the queue is at capacity it blocks → Basalt can't push state nullptr
    //  → outStateQueue::pop() never returns → run() never exits → deadlock.)
    if(pimpl->outVisQueue) {
        basalt::VioVisualizationData::Ptr dummy;
        while(pimpl->outVisQueue->try_pop(dummy)) {}
    }

    // ThreadedNode::stop() sets running=false and closes all input queues so the
    // busy-loop in run() can exit.  Crucially, wait() is commented out there, so
    // the thread is NOT joined — only ~JoiningThread() joins it.
    ThreadedHostNode::stop();

    // CRITICAL: join the run() thread HERE, before our destructor starts tearing
    // down pimpl and the other BasaltVIO members.  ~JoiningThread() would join
    // later (inside ~ThreadedNode()), but by that point pimpl has already been
    // destroyed.  The running thread would then access freed memory, corrupting
    // the vtable and triggering "pure virtual method called".
    wait();
}

void BasaltVIO::setImuExtrinsics(const std::shared_ptr<TransformData>& imuExtr) {
    imuExtrinsics = imuExtr;
}

void BasaltVIO::setAccelBias(const std::vector<double>& accelBias) {
    if(accelBias.size() != 9) {
        throw std::invalid_argument("Accelerometer bias vector must have 9 elements.");
    }
    this->accelBias = accelBias;
}

void BasaltVIO::setAccelNoiseStd(const std::vector<double>& accelNoiseStd) {
    if(accelNoiseStd.size() != 3) {
        throw std::invalid_argument("Accelerometer noise vector must have 3 elements.");
    }
    this->accelNoiseStd = accelNoiseStd;
    ;
}

void BasaltVIO::setGyroNoiseStd(const std::vector<double>& gyroNoiseStd) {
    if(gyroNoiseStd.size() != 3) {
        throw std::invalid_argument("Gyroscope noise vector must have 3 elements.");
    }
    this->gyroNoiseStd = gyroNoiseStd;
}

void BasaltVIO::setGyroBias(const std::vector<double>& gyroBias) {
    if(gyroBias.size() != 12) {
        throw std::invalid_argument("Gyroscope bias vector must have 12 elements.");
    }

    this->gyroBias = gyroBias;
}

void BasaltVIO::initialize(std::vector<std::shared_ptr<ImgFrame>> frames) {
    if(threadNum > 0) {
        pimpl->tbbGlobalControl = std::make_shared<tbb::global_control>(tbb::global_control::max_allowed_parallelism, threadNum);
    }

    auto pipeline = getParentPipeline();
    using Scalar = double;
    pimpl->calib = std::make_shared<basalt::Calibration<Scalar>>();
    pimpl->calib->imu_update_rate = imuUpdateRate;

    auto calibHandler = pipeline.getDefaultDevice()->readCalibration();

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
    if(accelNoiseStd.has_value()) {
        pimpl->calib->accel_noise_std = Eigen::Vector3d(accelNoiseStd.value()[0], accelNoiseStd.value()[1], accelNoiseStd.value()[2]).cwiseSqrt();
    }
    if(gyroNoiseStd.has_value()) {
        pimpl->calib->gyro_noise_std = Eigen::Vector3d(gyroNoiseStd.value()[0], gyroNoiseStd.value()[1], gyroNoiseStd.value()[2]).cwiseSqrt();
    }

    int camIdx = 0;
    basalt::Calibration<Scalar>::SE3 T_i_c0_override;
    CameraBoardSocket firstCamSocket{};
    for(const auto& frame : frames) {
        Eigen::Vector2i resolution;
        resolution << frame->getWidth(), frame->getHeight();
        pimpl->calib->resolution.push_back(resolution);
        auto camID = static_cast<CameraBoardSocket>(frame->getInstanceNum());
        // imu extrinsics
        if(imuExtrinsics.has_value()) {
            if(camIdx == 0) {
                // First camera: use override directly (T_imu_cam[0] from Basalt calibration)
                Eigen::Vector3d trans(
                    imuExtrinsics.value()->getTranslation().x, imuExtrinsics.value()->getTranslation().y, imuExtrinsics.value()->getTranslation().z);
                Eigen::Quaterniond q(imuExtrinsics.value()->getQuaternion().qw,
                                     imuExtrinsics.value()->getQuaternion().qx,
                                     imuExtrinsics.value()->getQuaternion().qy,
                                     imuExtrinsics.value()->getQuaternion().qz);
                T_i_c0_override = basalt::Calibration<Scalar>::SE3(q, trans);
                firstCamSocket = camID;
                pimpl->calib->T_i_c.push_back(T_i_c0_override);
            } else {
                // Subsequent cameras: T_i_cn = T_i_c0 * T_c0_cn
                // getCameraExtrinsics(src=cn, dst=c0) gives the transform from cn to c0
                auto stereoExtr = calibHandler.getCameraExtrinsics(camID, firstCamSocket, useSpecTranslation);
                Eigen::Matrix<Scalar, 3, 3> R;
                R << double(stereoExtr[0][0]), double(stereoExtr[0][1]), double(stereoExtr[0][2]),
                    double(stereoExtr[1][0]), double(stereoExtr[1][1]), double(stereoExtr[1][2]),
                    double(stereoExtr[2][0]), double(stereoExtr[2][1]), double(stereoExtr[2][2]);
                Eigen::Quaterniond q(R);
                // Translation in EEPROM is in centimeters, convert to meters
                Eigen::Vector3d trans(double(stereoExtr[0][3]) * 0.01, double(stereoExtr[1][3]) * 0.01, double(stereoExtr[2][3]) * 0.01);
                basalt::Calibration<Scalar>::SE3 T_c0_cn(q, trans);
                pimpl->calib->T_i_c.push_back(T_i_c0_override * T_c0_cn);
            }
        } else {
            std::vector<std::vector<float>> imuExtr = calibHandler.getCameraToImuExtrinsics(camID, useSpecTranslation);

            Eigen::Matrix<Scalar, 3, 3> R;
            R << double(imuExtr[0][0]), double(imuExtr[0][1]), double(imuExtr[0][2]), double(imuExtr[1][0]), double(imuExtr[1][1]), double(imuExtr[1][2]),
                double(imuExtr[2][0]), double(imuExtr[2][1]), double(imuExtr[2][2]);
            Eigen::Quaterniond q(R);

            Eigen::Vector3d trans(double(imuExtr[0][3]) * 0.01, double(imuExtr[1][3]) * 0.01, double(imuExtr[2][3]) * 0.01);
            basalt::Calibration<Scalar>::SE3 T_i_c(q, trans);
            pimpl->calib->T_i_c.push_back(T_i_c);
        }

        // camera intrinsics
        auto intrinsics = calibHandler.getCameraIntrinsics(camID, frame->getWidth(), frame->getHeight());
        auto model = calibHandler.getDistortionModel(camID);
        auto distCoeffs = calibHandler.getDistortionCoefficients(camID);
        basalt::GenericCamera<Scalar> camera;
        if(model == CameraModel::Perspective) {
            basalt::PinholeRadtan8Camera<Scalar>::VecN params;
            // fx, fy, cx, cy
            double fx = double(intrinsics[0][0]);
            double fy = double(intrinsics[1][1]);
            double cx = double(intrinsics[0][2]);
            double cy = double(intrinsics[1][2]);
            double k1 = double(distCoeffs[0]);
            double k2 = double(distCoeffs[1]);
            double p1 = double(distCoeffs[2]);
            double p2 = double(distCoeffs[3]);
            double k3 = double(distCoeffs[4]);
            double k4 = double(distCoeffs[5]);
            double k5 = double(distCoeffs[6]);
            double k6 = double(distCoeffs[7]);
            params << fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6;
            basalt::PinholeRadtan8Camera<Scalar> pinhole(params);
            camera.variant = pinhole;
        } else if(model == CameraModel::Fisheye) {
            // fx, fy, cx, cy
            double fx = double(intrinsics[0][0]);
            double fy = double(intrinsics[1][1]);
            double cx = double(intrinsics[0][2]);
            double cy = double(intrinsics[1][2]);
            double k1 = double(distCoeffs[0]);
            double k2 = double(distCoeffs[1]);
            double k3 = double(distCoeffs[2]);
            double k4 = double(distCoeffs[3]);
            basalt::KannalaBrandtCamera4<Scalar>::VecN params;
            params << fx, fy, cx, cy, k1, k2, k3, k4;
            basalt::KannalaBrandtCamera4<Scalar> kannala(params);
            camera.variant = kannala;
        } else {
            throw std::runtime_error("Unknown distortion model");
        }
        pimpl->calib->intrinsics.push_back(camera);
        camIdx++;
    }
    if(!configPath.empty()) {
        pimpl->vioConfig.load(configPath);
    }

    pimpl->optFlowPtr = basalt::OpticalFlowFactory::getOpticalFlow(pimpl->vioConfig, *pimpl->calib);
    pimpl->optFlowPtr->show_gui = false;
    pimpl->optFlowPtr->start();
    pimpl->imageDataQueue = pimpl->optFlowPtr->input_img_queue;
    pimpl->vio = basalt::VioEstimatorFactory::getVioEstimator(pimpl->vioConfig, *pimpl->calib, basalt::constants::g, true, true);
    pimpl->vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    pimpl->imuDataQueue = pimpl->vio->imu_data_queue;
    pimpl->optFlowPtr->output_queue = pimpl->vio->vision_data_queue;
    pimpl->outStateQueue = std::make_shared<tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr>>();
    pimpl->vio->out_state_queue = pimpl->outStateQueue;
    // NOTE: We intentionally leave vio->out_vis_queue as nullptr.
    //
    // Basalt's OpticalFlowInput::show_uimat defaults to UIMAT::ALL, so whenever
    // out_vis_queue is non-null Basalt computes and stores every UI Jacobian and
    // Hessian matrix on every frame.  These heavy-weight allocations are designed
    // for pangolin GUI display.  Running them headless triggers heap corruption
    // ("corrupted double-linked list") within minutes of operation.
    //
    // Tier-2 health metrics (numTrackedFeatures, numLandmarks) therefore remain
    // at their sentinel values (-1) until a lighter mechanism is added later.
    pimpl->vio->opt_flow_depth_guess_queue = pimpl->optFlowPtr->input_depth_queue;
    pimpl->vio->opt_flow_state_queue = pimpl->optFlowPtr->input_state_queue;
    pimpl->vio->opt_flow_lm_bundle_queue = pimpl->optFlowPtr->input_lm_bundle_queue;
    initialized.store(true, std::memory_order_release);
}
void BasaltVIO::runSyncOnHost(bool runOnHost) {
    sync->setRunOnHost(runOnHost);
}
void BasaltVIO::setDefaultVIOConfig() {
    pimpl->vioConfig.optical_flow_type = "frame_to_frame";
    pimpl->vioConfig.optical_flow_detection_grid_size = 50;
    pimpl->vioConfig.optical_flow_detection_num_points_cell = 1;
    pimpl->vioConfig.optical_flow_detection_min_threshold = 5;
    pimpl->vioConfig.optical_flow_detection_max_threshold = 40;
    pimpl->vioConfig.optical_flow_detection_nonoverlap = true;
    pimpl->vioConfig.optical_flow_max_recovered_dist2 = 0.04;
    pimpl->vioConfig.optical_flow_pattern = 51;
    pimpl->vioConfig.optical_flow_max_iterations = 5;
    pimpl->vioConfig.optical_flow_epipolar_error = 0.005;
    pimpl->vioConfig.optical_flow_levels = 3;
    pimpl->vioConfig.optical_flow_skip_frames = 1;
    pimpl->vioConfig.optical_flow_matching_guess_type = basalt::MatchingGuessType::REPROJ_AVG_DEPTH;
    pimpl->vioConfig.optical_flow_matching_default_depth = 2.0;
    pimpl->vioConfig.optical_flow_image_safe_radius = 472.0;
    pimpl->vioConfig.optical_flow_recall_enable = false;
    pimpl->vioConfig.optical_flow_recall_all_cams = false;
    pimpl->vioConfig.optical_flow_recall_num_points_cell = true;
    pimpl->vioConfig.optical_flow_recall_over_tracking = false;
    pimpl->vioConfig.optical_flow_recall_update_patch_viewpoint = false;
    pimpl->vioConfig.optical_flow_recall_max_patch_dist = 3;
    pimpl->vioConfig.optical_flow_recall_max_patch_norms = {1.74, 0.96, 0.99, 0.44};
    pimpl->vioConfig.vio_linearization_type = basalt::LinearizationType::ABS_QR;
    pimpl->vioConfig.vio_sqrt_marg = true;
    pimpl->vioConfig.vio_max_states = 3;
    pimpl->vioConfig.vio_max_kfs = 7;
    pimpl->vioConfig.vio_min_frames_after_kf = 5;
    pimpl->vioConfig.vio_new_kf_keypoints_thresh = 0.7;
    pimpl->vioConfig.vio_debug = false;
    pimpl->vioConfig.vio_extended_logging = false;
    pimpl->vioConfig.vio_obs_std_dev = 0.5;
    pimpl->vioConfig.vio_obs_huber_thresh = 1.0;
    pimpl->vioConfig.vio_min_triangulation_dist = 0.05;
    pimpl->vioConfig.vio_max_iterations = 7;
    pimpl->vioConfig.vio_enforce_realtime = false;
    pimpl->vioConfig.vio_use_lm = true;
    pimpl->vioConfig.vio_lm_lambda_initial = 1e-4;
    pimpl->vioConfig.vio_lm_lambda_min = 1e-6;
    pimpl->vioConfig.vio_lm_lambda_max = 1e2;
    pimpl->vioConfig.vio_scale_jacobian = false;
    pimpl->vioConfig.vio_init_pose_weight = 1e8;
    pimpl->vioConfig.vio_init_ba_weight = 1e1;
    pimpl->vioConfig.vio_init_bg_weight = 1e2;
    pimpl->vioConfig.vio_marg_lost_landmarks = true;
    pimpl->vioConfig.vio_fix_long_term_keyframes = false;
    pimpl->vioConfig.vio_kf_marg_feature_ratio = 0.1;
    pimpl->vioConfig.vio_kf_marg_criteria = basalt::KeyframeMargCriteria::KF_MARG_DEFAULT;
    pimpl->vioConfig.mapper_obs_std_dev = 0.25;
    pimpl->vioConfig.mapper_obs_huber_thresh = 1.5;
    pimpl->vioConfig.mapper_detection_num_points = 800;
    pimpl->vioConfig.mapper_num_frames_to_match = 30;
    pimpl->vioConfig.mapper_frames_to_match_threshold = 0.04;
    pimpl->vioConfig.mapper_min_matches = 20;
    pimpl->vioConfig.mapper_ransac_threshold = 5e-5;
    pimpl->vioConfig.mapper_min_track_length = 5;
    pimpl->vioConfig.mapper_max_hamming_distance = 70;
    pimpl->vioConfig.mapper_second_best_test_ratio = 1.2;
    pimpl->vioConfig.mapper_bow_num_bits = 16;
    pimpl->vioConfig.mapper_min_triangulation_dist = 0.07;
    pimpl->vioConfig.mapper_no_factor_weights = false;
    pimpl->vioConfig.mapper_use_factors = true;
    pimpl->vioConfig.mapper_use_lm = true;
    pimpl->vioConfig.mapper_lm_lambda_min = 1e-32;
    pimpl->vioConfig.mapper_lm_lambda_max = 1e3;
}
}  // namespace node
}  // namespace dai
