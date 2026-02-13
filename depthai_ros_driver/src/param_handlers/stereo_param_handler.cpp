#include "depthai_ros_driver/param_handlers/stereo_param_handler.hpp"

#include "depthai/common/CameraFeatures.hpp"
#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
StereoParamHandler::StereoParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat)
    : BaseParamHandler(node, name, deviceName, rsCompat) {
    depthPresetMap = {{"FAST_ACCURACY", dai::node::StereoDepth::PresetMode::FAST_ACCURACY},
                      {"DEFAULT", dai::node::StereoDepth::PresetMode::DEFAULT},
                      {"FACE", dai::node::StereoDepth::PresetMode::FACE},
                      {"HIGH_DETAIL", dai::node::StereoDepth::PresetMode::HIGH_DETAIL},
                      {"ROBOTICS", dai::node::StereoDepth::PresetMode::ROBOTICS}};

    disparityWidthMap = {
        {"DISPARITY_64", dai::StereoDepthConfig::CostMatching::DisparityWidth::DISPARITY_64},
        {"DISPARITY_96", dai::StereoDepthConfig::CostMatching::DisparityWidth::DISPARITY_96},
    };

    decimationModeMap = {{"PIXEL_SKIPPING", dai::StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode::PIXEL_SKIPPING},
                         {"NON_ZERO_MEDIAN", dai::StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode::NON_ZERO_MEDIAN},
                         {"NON_ZERO_MEAN", dai::StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode::NON_ZERO_MEAN}};

    temporalPersistencyMap = {
        {"PERSISTENCY_OFF", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::PERSISTENCY_OFF},
        {"VALID_8_OUT_OF_8", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_8_OUT_OF_8},
        {"VALID_2_IN_LAST_3", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_2_IN_LAST_3},
        {"VALID_2_IN_LAST_4", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_2_IN_LAST_4},
        {"VALID_2_OUT_OF_8", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_2_OUT_OF_8},
        {"VALID_1_IN_LAST_2", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_1_IN_LAST_2},
        {"VALID_1_IN_LAST_5", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_1_IN_LAST_5},
        {"VALID_1_IN_LAST_8", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_1_IN_LAST_8},
        {"PERSISTENCY_INDEFINITELY", dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::PERSISTENCY_INDEFINITELY},
    };

    medianFilterMap = {{"MEDIAN_OFF", dai::StereoDepthConfig::MedianFilter::MEDIAN_OFF},
                       {"KERNEL_3x3", dai::StereoDepthConfig::MedianFilter::KERNEL_3x3},
                       {"KERNEL_5x5", dai::StereoDepthConfig::MedianFilter::KERNEL_5x5}};
}

StereoParamHandler::~StereoParamHandler() = default;

void StereoParamHandler::updateSocketsFromParams(dai::CameraBoardSocket& left, dai::CameraBoardSocket& right, dai::CameraBoardSocket& align) {
    int newLeftS = declareAndLogParam<int>("i_left_socket_id", static_cast<int>(left));
    int newRightS = declareAndLogParam<int>("i_right_socket_id", static_cast<int>(right));
    alignSocket = static_cast<dai::CameraBoardSocket>(declareAndLogParam<int>("i_board_socket_id", static_cast<int>(align)));
    if(newLeftS != static_cast<int>(left) || newRightS != static_cast<int>(right)) {
        RCLCPP_WARN(getROSNode()->get_logger(), "Left or right socket changed, updating stereo node");
        RCLCPP_WARN(getROSNode()->get_logger(), "Old left socket: %d, new left socket: %d", static_cast<int>(left), newLeftS);
        RCLCPP_WARN(getROSNode()->get_logger(), "Old right socket: %d, new right socket: %d", static_cast<int>(right), newRightS);
    }
    left = static_cast<dai::CameraBoardSocket>(newLeftS);
    right = static_cast<dai::CameraBoardSocket>(newRightS);
}

void StereoParamHandler::declareParams(std::shared_ptr<dai::node::StereoDepth> stereo) {
    declareAndLogParam<int>(ParamNames::MAX_Q_SIZE, 30);
    bool lowBandwidth = declareAndLogParam<bool>(ParamNames::LOW_BANDWIDTH, false);
    declareAndLogParam<int>(ParamNames::LOW_BANDWIDTH_QUALITY, 50);
    declareAndLogParam<int>(ParamNames::LOW_BANDWIDTH_PROFILE, 4);
    declareAndLogParam<int>(ParamNames::LOW_BANDWIDTH_FRAME_FREQ, 30);
    declareAndLogParam<int>(ParamNames::LOW_BANDWIDTH_BITRATE, 0);
    declareAndLogParam<std::string>(ParamNames::LOW_BANDWIDTH_FFMPEG_ENCODER, "libx264");
    declareAndLogParam<bool>("i_output_disparity", false);
    declareAndLogParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP, false);
    declareAndLogParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG, false);
    declareAndLogParam<bool>(ParamNames::PUBLISH_TOPIC, true);
    declareAndLogParam<bool>(ParamNames::ADD_EXPOSURE_OFFSET, false);
    declareAndLogParam<int>(ParamNames::EXPOSURE_OFFSET, 0);
    declareAndLogParam<bool>(ParamNames::ENABLE_LAZY_PUBLISHER, true);
    declareAndLogParam<bool>(ParamNames::REVERSE_STEREO_SOCKET_ORDER, false);
    declareAndLogParam<bool>(ParamNames::PUBLISH_COMPRESSED, false);
    declareAndLogParam<float>(ParamNames::FPS, 30);
    declareAndLogParam<std::string>(ParamNames::CALIBRATION_FILE, "");

    declareAndLogParam<bool>("i_left_rect_publish_topic", false);
    declareAndLogParam<bool>("i_left_rect_low_bandwidth", false);
    declareAndLogParam<int>("i_left_rect_low_bandwidth_profile", 4);
    declareAndLogParam<int>("i_left_rect_low_bandwidth_frame_freq", 30);
    declareAndLogParam<int>("i_left_rect_low_bandwidth_bitrate", 0);
    declareAndLogParam<int>("i_left_rect_low_bandwidth_quality", 50);
    declareAndLogParam<std::string>("i_left_rect_low_bandwidth_ffmpeg_encoder", "libx264");
    declareAndLogParam<bool>("i_left_rect_add_exposure_offset", false);
    declareAndLogParam<int>("i_left_rect_exposure_offset", 0);
    declareAndLogParam<bool>("i_left_rect_enable_feature_tracker", false);
    declareAndLogParam<bool>("i_left_rect_synced", false);
    declareAndLogParam<bool>("i_left_rect_publish_compressed", false);

    declareAndLogParam<bool>("i_right_rect_publish_topic", false);
    declareAndLogParam<bool>("i_right_rect_low_bandwidth", false);
    declareAndLogParam<int>("i_right_rect_low_bandwidth_quality", 50);
    declareAndLogParam<int>("i_right_rect_low_bandwidth_profile", 4);
    declareAndLogParam<int>("i_right_rect_low_bandwidth_frame_freq", 30);
    declareAndLogParam<int>("i_right_rect_low_bandwidth_bitrate", 0);
    declareAndLogParam<std::string>("i_right_rect_low_bandwidth_ffmpeg_encoder", "libx264");
    declareAndLogParam<bool>("i_right_rect_enable_feature_tracker", false);
    declareAndLogParam<bool>("i_right_rect_add_exposure_offset", false);
    declareAndLogParam<int>("i_right_rect_exposure_offset", 0);
    declareAndLogParam<bool>("i_right_rect_synced", false);
    declareAndLogParam<bool>("i_right_rect_publish_compressed", false);

    declareAndLogParam<bool>("i_enable_left_spatial_nn", false);
    declareAndLogParam<bool>("i_enable_right_spatial_nn", false);
    declareAndLogParam<bool>("i_enable_left_rgbd", false);
    declareAndLogParam<bool>("i_enable_right_rgbd", false);
    declareAndLogParam<bool>(ParamNames::SYNCED, false);
    declareAndLogParam<bool>("i_run_align_on_host", true);
    declareAndLogParam<bool>(ParamNames::ALIGNED, true);

    stereo->setLeftRightCheck(declareAndLogParam<bool>("i_lr_check", true));
    int width = 640;
    int height = 400;
    std::string socketName;
    socketName = getSocketName(alignSocket);
    declareAndLogParam<std::string>("i_socket_name", socketName);

    if(declareAndLogParam<bool>("i_set_input_size", false)) {
        stereo->setInputResolution(declareAndLogParam<int>("i_input_width", width), declareAndLogParam<int>("i_input_height", height));
    }
    auto depthPreset = depthPresetMap.at(declareAndLogParam<std::string>("i_depth_preset", "FAST_ACCURACY"));
    stereo->setDefaultProfilePreset(depthPreset);
    width = declareAndLogParam<int>(ParamNames::WIDTH, width);
    height = declareAndLogParam<int>(ParamNames::HEIGHT, height);
    if(declareAndLogParam<bool>("i_enable_distortion_correction", true)) {
        stereo->enableDistortionCorrection(true);
    }
    if(declareAndLogParam<bool>("i_set_disparity_to_depth_use_spec_translation", false)) {
        stereo->setDisparityToDepthUseSpecTranslation(true);
    }
    //
    stereo->initialConfig->setBilateralFilterSigma(declareAndLogParam<int>("i_bilateral_sigma", 0));
    stereo->initialConfig->setLeftRightCheckThreshold(declareAndLogParam<int>("i_lrc_threshold", 10));
    bool useHostFilters = declareAndLogParam<bool>("i_use_host_filters", false);
    if(!useHostFilters) {
        stereo->initialConfig->setMedianFilter(
            utils::getValFromMap(declareAndLogParam<std::string>("i_median_filter", "MEDIAN_OFF"), medianFilterMap));
    } else {
        declareAndLogParam<std::string>("i_median_filter", "MEDIAN_OFF");
    }
    stereo->initialConfig->setConfidenceThreshold(declareAndLogParam<int>("i_stereo_conf_threshold", 15));
    if(declareAndLogParam<bool>("i_subpixel", true) && !lowBandwidth) {
        stereo->initialConfig->setSubpixel(true);
        stereo->initialConfig->setSubpixelFractionalBits(declareAndLogParam<int>("i_subpixel_fractional_bits", 3));
    } else {
        stereo->initialConfig->setSubpixel(false);
        if(lowBandwidth) {
            RCLCPP_INFO(getROSNode()->get_logger(), "Subpixel disabled due to low bandwidth mode");
        }
    }
    stereo->setRectifyEdgeFillColor(declareAndLogParam<int>("i_rectify_edge_fill_color", 0));
    if(declareAndLogParam<bool>("i_enable_alpha_scaling", false)) {
        stereo->setAlphaScaling(declareAndLogParam<float>("i_alpha_scaling", 0.0));
    }
    auto config = stereo->initialConfig;
    config->costMatching.disparityWidth = utils::getValFromMap(declareAndLogParam<std::string>("i_disparity_width", "DISPARITY_96"), disparityWidthMap);
    stereo->setExtendedDisparity(declareAndLogParam<bool>("i_extended_disp", false));
    config->costMatching.enableCompanding = declareAndLogParam<bool>("i_enable_companding", false);
    // Declare filter params (used by both device-side PostProcessing and host-side ImageFilters)
    bool enableTemporal = declareAndLogParam<bool>("i_enable_temporal_filter", false);
    float temporalAlpha = declareAndLogParam<float>("i_temporal_filter_alpha", 0.4);
    int temporalDelta = declareAndLogParam<int>("i_temporal_filter_delta", 20);
    auto temporalPersistency = declareAndLogParam<std::string>("i_temporal_filter_persistency", "VALID_2_IN_LAST_4");

    bool enableSpeckle = declareAndLogParam<bool>("i_enable_speckle_filter", false);
    int speckleRange = declareAndLogParam<int>("i_speckle_filter_speckle_range", 50);
    int speckleDiffThresh = declareAndLogParam<int>("i_speckle_filter_difference_threshold", 2);

    bool enableSpatial = declareAndLogParam<bool>("i_enable_spatial_filter", false);
    int spatialHoleRadius = declareAndLogParam<int>("i_spatial_filter_hole_filling_radius", 2);
    float spatialAlpha = declareAndLogParam<float>("i_spatial_filter_alpha", 0.5);
    int spatialDelta = declareAndLogParam<int>("i_spatial_filter_delta", 20);
    int spatialIters = declareAndLogParam<int>("i_spatial_filter_iterations", 1);

    if(!useHostFilters) {
        // Apply filters on-device via StereoDepthConfig::PostProcessing
        if(enableTemporal) {
            config->postProcessing.temporalFilter.enable = true;
            config->postProcessing.temporalFilter.alpha = temporalAlpha;
            config->postProcessing.temporalFilter.delta = temporalDelta;
            config->postProcessing.temporalFilter.persistencyMode = utils::getValFromMap(temporalPersistency, temporalPersistencyMap);
        }
        if(enableSpeckle) {
            config->postProcessing.speckleFilter.enable = true;
            config->postProcessing.speckleFilter.speckleRange = speckleRange;
            config->postProcessing.speckleFilter.differenceThreshold = speckleDiffThresh;
        }
        if(enableSpatial) {
            config->postProcessing.spatialFilter.enable = true;
            config->postProcessing.spatialFilter.holeFillingRadius = spatialHoleRadius;
            config->postProcessing.spatialFilter.alpha = spatialAlpha;
            config->postProcessing.spatialFilter.delta = spatialDelta;
            config->postProcessing.spatialFilter.numIterations = spatialIters;
        }
    } else {
        // Disable device-side PostProcessing filters (the preset may have enabled them).
        // These will run on the host via ImageFilters instead, avoiding double filtering.
        config->postProcessing.spatialFilter.enable = false;
        config->postProcessing.temporalFilter.enable = false;
        config->postProcessing.speckleFilter.enable = false;
        config->postProcessing.median = dai::StereoDepthConfig::MedianFilter::MEDIAN_OFF;
    }

    if(declareAndLogParam<bool>("i_enable_disparity_shift", false)) {
        config->algorithmControl.disparityShift = declareAndLogParam<int>("i_disparity_shift", 0);
    }
    if(declareAndLogParam<bool>("i_enable_threshold_filter", false)) {
        config->postProcessing.thresholdFilter.minRange = declareAndLogParam<int>("i_threshold_filter_min_range", 400);
        config->postProcessing.thresholdFilter.maxRange = declareAndLogParam<int>("i_threshold_filter_max_range", 15000);
    }
    if(declareAndLogParam<bool>("i_enable_brightness_filter", false)) {
        config->postProcessing.brightnessFilter.minBrightness = declareAndLogParam<int>("i_brightness_filter_min_brightness", 0);
        config->postProcessing.brightnessFilter.maxBrightness = declareAndLogParam<int>("i_brightness_filter_max_brightness", 256);
    }
    if(declareAndLogParam<bool>("i_enable_decimation_filter", false)) {
        config->postProcessing.decimationFilter.decimationMode =
            utils::getValFromMap(declareAndLogParam<std::string>("i_decimation_filter_decimation_mode", "PIXEL_SKIPPING"), decimationModeMap);
        config->postProcessing.decimationFilter.decimationFactor = declareAndLogParam<int>("i_decimation_filter_decimation_factor", 1);
        int decimatedWidth = width / config->postProcessing.decimationFilter.decimationFactor;
        int decimatedHeight = height / config->postProcessing.decimationFilter.decimationFactor;
        RCLCPP_INFO(getROSNode()->get_logger(),
                    "Decimation filter enabled with decimation factor %d. Previous width and height: %d x %d, after decimation: %d x %d",
                    config->postProcessing.decimationFilter.decimationFactor,
                    width,
                    height,
                    decimatedWidth,
                    decimatedHeight);
        stereo->setOutputSize(decimatedWidth, decimatedHeight);
    }
    declareAndLogParam("i_width", width, true);
    declareAndLogParam("i_height", height, true);
    stereo->initialConfig = config;
}
void StereoParamHandler::configureImageFilters(std::shared_ptr<dai::ImageFiltersConfig> config) {
    std::vector<dai::FilterParams> params;

    // Speckle filter
    dai::SpeckleFilterParams speckle;
    speckle.enable = getParam<bool>("i_enable_speckle_filter");
    speckle.speckleRange = getParam<int>("i_speckle_filter_speckle_range");
    speckle.differenceThreshold = getParam<int>("i_speckle_filter_difference_threshold");
    params.push_back(speckle);

    // Temporal filter
    dai::TemporalFilterParams temporal;
    temporal.enable = getParam<bool>("i_enable_temporal_filter");
    temporal.alpha = getParam<float>("i_temporal_filter_alpha");
    temporal.delta = getParam<int>("i_temporal_filter_delta");
    temporal.persistencyMode = utils::getValFromMap(getParam<std::string>("i_temporal_filter_persistency"), temporalPersistencyMap);
    params.push_back(temporal);

    // Spatial filter
    dai::SpatialFilterParams spatial;
    spatial.enable = getParam<bool>("i_enable_spatial_filter");
    spatial.holeFillingRadius = getParam<int>("i_spatial_filter_hole_filling_radius");
    spatial.alpha = getParam<float>("i_spatial_filter_alpha");
    spatial.delta = getParam<int>("i_spatial_filter_delta");
    spatial.numIterations = getParam<int>("i_spatial_filter_iterations");
    params.push_back(spatial);

    // Median filter
    auto median = utils::getValFromMap(getParam<std::string>("i_median_filter"), medianFilterMap);
    params.push_back(static_cast<dai::MedianFilterParams>(median));

    config->filterIndices = {};
    config->filterParams = params;
}

}  // namespace param_handlers
}  // namespace depthai_ros_driver
