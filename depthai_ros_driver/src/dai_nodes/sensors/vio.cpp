#include "depthai_ros_driver/dai_nodes/sensors/vio.hpp"

#include <cmath>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/pipeline/datatype/VioHealthData.hpp"
#include "depthai_bridge/TransformDataConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/imu.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/stereo.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/param_handlers/vio_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/node.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace depthai_ros_driver {
namespace dai_nodes {
Vio::Vio(const std::string& daiNodeName,
         std::shared_ptr<rclcpp::Node> node,
         std::shared_ptr<dai::Pipeline> pipeline,
         std::shared_ptr<dai::Device> device,
         bool rsCompat,
         SensorWrapper& left,
         SensorWrapper& right,
         Imu& imu)
    : BaseNode(daiNodeName, node, pipeline, device->getDeviceName(), rsCompat) {
    using namespace param_handlers;
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    vioNode = pipeline->create<dai::node::BasaltVIO>();
    ph = std::make_unique<VioParamHandler>(node, daiNodeName, device->getDeviceName(), rsCompat);
    ph->declareParams(vioNode);
    frameId = ph->getParam<std::string>("i_frame_id");
    childFrameId = ph->getParam<std::string>("i_child_frame_id");
    auto width = ph->getParam<int>(ParamNames::WIDTH);
    auto height = ph->getParam<int>(ParamNames::HEIGHT);
    auto fps = ph->getParam<double>(ParamNames::FPS);
    socket = ph->getSocketID();
    imu.link(vioNode->imu);

    left.getUnderlyingNode()->requestOutput(std::make_pair(width, height), std::nullopt, dai::ImgResizeMode::CROP, fps)->link(vioNode->left);
    right.getUnderlyingNode()->requestOutput(std::make_pair(width, height), std::nullopt, dai::ImgResizeMode::CROP, fps)->link(vioNode->right);
    vioNode->setImuUpdateRate(ph->getParam<int>("i_imu_update_rate"));
    publishTf = ph->getParam<bool>("i_publish_tf");
    if(publishTf) {
        tfBr = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    }

    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
}
Vio::Vio(const std::string& daiNodeName,
         std::shared_ptr<rclcpp::Node> node,
         std::shared_ptr<dai::Pipeline> pipeline,
         std::shared_ptr<dai::Device> device,
         bool rsCompat,
         Stereo& stereo,
         Imu& imu)
    : BaseNode(daiNodeName, node, pipeline, device->getDeviceName(), rsCompat) {
    using namespace param_handlers;
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    vioNode = pipeline->create<dai::node::BasaltVIO>();
    ph = std::make_unique<VioParamHandler>(node, daiNodeName, device->getDeviceName(), rsCompat);
    ph->declareParams(vioNode);
    frameId = ph->getParam<std::string>("i_frame_id");
    childFrameId = ph->getParam<std::string>("i_child_frame_id");
    imu.link(vioNode->imu);
    auto width = ph->getParam<int>(ParamNames::WIDTH);
    auto height = ph->getParam<int>(ParamNames::HEIGHT);
    auto fps = ph->getParam<double>(ParamNames::FPS);
    socket = ph->getSocketID();
    stereo.getLeftSensor()->getUnderlyingNode()->requestOutput(std::make_pair(width, height), std::nullopt, dai::ImgResizeMode::CROP, fps)->link(vioNode->left);
    stereo.getRightSensor()
        ->getUnderlyingNode()
        ->requestOutput(std::make_pair(width, height), std::nullopt, dai::ImgResizeMode::CROP, fps)
        ->link(vioNode->right);
    vioNode->setImuUpdateRate(ph->getParam<int>("i_imu_update_rate"));
    publishTf = ph->getParam<bool>("i_publish_tf");
    if(publishTf) {
        tfBr = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    }

    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
}
Vio::~Vio() = default;

void Vio::setNames() {}

void Vio::setInOut(std::shared_ptr<dai::Pipeline> /* pipeline */) {}

void Vio::setupQueues(std::shared_ptr<dai::Device> /* device */) {
    using ParamNames = param_handlers::ParamNames;
    const int qSize = ph->getParam<int>(ParamNames::MAX_Q_SIZE);

    // ── Odometry (pose) queue — unchanged behaviour ───────────────────────
    transQ = vioNode->transform.createOutputQueue(qSize, false);
    auto tfPrefix = frameId;
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions();
    odomConv = std::make_unique<depthai_bridge::TransformDataConverter>(tfPrefix, childFrameId, ph->getParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP));
    odomConv->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG));
    odomConv->setCovariance(ph->getParam<std::vector<double>>("i_covariance"));

    odomPub = getROSNode()->create_publisher<nav_msgs::msg::Odometry>("~/" + getName() + "/odometry", qSize, options);
    transQ->addCallback(std::bind(&Vio::transCB, this, std::placeholders::_1, std::placeholders::_2));

    // ── Health queue — velocity, biases, tracking quality ─────────────────
    healthQ = vioNode->health.createOutputQueue(qSize, false);

    velPub = getROSNode()->create_publisher<geometry_msgs::msg::TwistStamped>(
        "~/" + getName() + "/velocity", qSize, options);
    healthPub = getROSNode()->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
        "~/" + getName() + "/health", qSize, options);

    healthQ->addCallback(std::bind(&Vio::healthCB, this, std::placeholders::_1, std::placeholders::_2));
}

void Vio::closeQueues() {
    transQ->close();
    healthQ->close();
}

void Vio::transCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto transData = std::dynamic_pointer_cast<dai::TransformData>(data);
    std::deque<nav_msgs::msg::Odometry> deq;
    odomConv->toRosMsg(transData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        currMsg.header.stamp = getROSNode()->get_clock()->now();
        odomPub->publish(currMsg);
        if(publishTf) {
            geometry_msgs::msg::TransformStamped transformMsg;
            transformMsg.header.stamp = currMsg.header.stamp;
            transformMsg.header.frame_id = frameId;
            transformMsg.child_frame_id = childFrameId;
            transformMsg.transform.translation.x = currMsg.pose.pose.position.x;
            transformMsg.transform.translation.y = currMsg.pose.pose.position.y;
            transformMsg.transform.translation.z = currMsg.pose.pose.position.z;
            transformMsg.transform.rotation = currMsg.pose.pose.orientation;
            tfBr->sendTransform(transformMsg);
        }
        deq.pop_front();
    }
}

void Vio::healthCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto h = std::dynamic_pointer_cast<dai::VioHealthData>(data);
    if(!h) return;

    const auto stamp = getROSNode()->get_clock()->now();

    // ── /oak/vio/velocity  (geometry_msgs/TwistStamped) ──────────────────
    // Linear velocity of the camera body in the FLU world frame (m/s).
    // Feed this directly into robot_localization or your EKF as a velocity
    // observation with the odom frame_id.
    geometry_msgs::msg::TwistStamped velMsg;
    velMsg.header.stamp = stamp;
    velMsg.header.frame_id = frameId;
    velMsg.twist.linear.x = h->velX;
    velMsg.twist.linear.y = h->velY;
    velMsg.twist.linear.z = h->velZ;
    velPub->publish(velMsg);

    // ── /oak/vio/health  (diagnostic_msgs/DiagnosticStatus) ──────────────
    // All metrics in one place, viewable in Foxglove's Diagnostics panel.
    // Level: OK (0) = healthy, WARN (1) = degraded, ERROR (2) = poor.
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = getName() + "/vio_health";

    auto kv = [](const std::string& key, const std::string& val) {
        diagnostic_msgs::msg::KeyValue p;
        p.key = key;
        p.value = val;
        return p;
    };

    // Velocity magnitude
    const double speed = std::sqrt(h->velX * h->velX + h->velY * h->velY + h->velZ * h->velZ);
    status.values.push_back(kv("vel_x_m_s",  std::to_string(h->velX)));
    status.values.push_back(kv("vel_y_m_s",  std::to_string(h->velY)));
    status.values.push_back(kv("vel_z_m_s",  std::to_string(h->velZ)));
    status.values.push_back(kv("speed_m_s",  std::to_string(speed)));

    // IMU biases
    const double accelNorm = std::sqrt(h->accelBiasX * h->accelBiasX + h->accelBiasY * h->accelBiasY + h->accelBiasZ * h->accelBiasZ);
    const double gyroNorm  = std::sqrt(h->gyroBiasX  * h->gyroBiasX  + h->gyroBiasY  * h->gyroBiasY  + h->gyroBiasZ  * h->gyroBiasZ);
    status.values.push_back(kv("accel_bias_x_m_s2", std::to_string(h->accelBiasX)));
    status.values.push_back(kv("accel_bias_y_m_s2", std::to_string(h->accelBiasY)));
    status.values.push_back(kv("accel_bias_z_m_s2", std::to_string(h->accelBiasZ)));
    status.values.push_back(kv("accel_bias_norm",   std::to_string(accelNorm)));
    status.values.push_back(kv("gyro_bias_x_rad_s", std::to_string(h->gyroBiasX)));
    status.values.push_back(kv("gyro_bias_y_rad_s", std::to_string(h->gyroBiasY)));
    status.values.push_back(kv("gyro_bias_z_rad_s", std::to_string(h->gyroBiasZ)));
    status.values.push_back(kv("gyro_bias_norm",    std::to_string(gyroNorm)));

    // Tracking quality
    status.values.push_back(kv("tracked_features",      std::to_string(h->numTrackedFeatures)));
    status.values.push_back(kv("active_landmarks",      std::to_string(h->numLandmarks)));
    status.values.push_back(kv("mean_feature_response", std::to_string(h->meanFeatureResponse)));

    // Set severity level based on feature count.
    // < 10 features is effectively failed tracking; < 20 is degraded.
    if(h->numTrackedFeatures < 0) {
        // vis data not yet available (first few frames)
        status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "initialising";
    } else if(h->numTrackedFeatures < 10) {
        status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "tracking lost — too few features";
    } else if(h->numTrackedFeatures < 20) {
        status.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "degraded — low feature count";
    } else {
        status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "nominal";
    }

    healthPub->publish(status);
}

void Vio::link(dai::Node::Input& in, int /*linkType*/) {
    vioNode->transform.link(in);
}

dai::Node::Input& Vio::getInput(int linkType) {
    if(linkType == static_cast<int>(link_types::VioLinkType::left)) {
        return vioNode->left;
    } else if(linkType == static_cast<int>(link_types::VioLinkType::right)) {
        return vioNode->right;
    } else if(linkType == static_cast<int>(link_types::VioLinkType::imu)) {
        return vioNode->imu;
    } else {
        RCLCPP_ERROR(getLogger(), "Wrong link type: %d", linkType);
        throw std::runtime_error("Wrong link type specified!");
    }
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
