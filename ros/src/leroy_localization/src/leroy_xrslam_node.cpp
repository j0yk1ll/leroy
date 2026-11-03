// TODO(AUTONOMY) #009 [metric visual-inertial localization]: Calibrate actual
// camera-to-IMU rotation/translation, acquisition-time phase, rolling
// shutter characteristics, IMU axes/noise and native XRSLAM mode on Leroy.
// Camera-only monocular yaw cannot establish metric scale or necessarily
// provide triangulation; plan clearance-checked translational parallax or
// an independently qualified scale source when initialization needs it.
// TODO(AUTONOMY) #010 [pose authority]: Validate native pose frame direction,
// world scale, covariance, drift, relocalization and loss-of-tracking
// behavior before allowing /odom or odom->base_link/map->odom publication.
// Acceptance: cold boot, low-texture/lighting changes, lost tracking,
// timestamp jumps and reinitialization leave metric pose invalid until
// independently measured accuracy and continuity are demonstrated.

#include <XRSLAM.h>
#include "leroy_xrslam/sensor_mode.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cstring>
#include <cstdlib>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <optional>
#include <utility>
#include <cmath>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
double seconds(const builtin_interfaces::msg::Time & stamp) {
  return rclcpp::Time(stamp).seconds();
}
bool is_finite_value(double x) { return std::isfinite(x); }
bool valid_quat(const geometry_msgs::msg::Quaternion & q) {
  const double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  return is_finite_value(n) && std::abs(n-1.0)<1e-3;
}
YAML::Node vec3(const geometry_msgs::msg::Vector3 & p) {
  YAML::Node n(YAML::NodeType::Sequence);
  n.push_back(p.x);n.push_back(p.y);n.push_back(p.z);return n;
}
YAML::Node quat4(const geometry_msgs::msg::Quaternion & q) {
  YAML::Node n(YAML::NodeType::Sequence);
  n.push_back(q.x);n.push_back(q.y);n.push_back(q.z);n.push_back(q.w);return n;
}
geometry_msgs::msg::Quaternion inverse_quat(const geometry_msgs::msg::Quaternion & q) {
  geometry_msgs::msg::Quaternion o=q;o.x=-q.x;o.y=-q.y;o.z=-q.z;return o;
}
} // namespace

class LeroyXrslam final : public rclcpp::Node {
public:
  LeroyXrslam()
  : Node("leroy_xrslam"),
    tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
    tf_broadcaster_(*this) {
    slam_yaml_=declare_parameter<std::string>("slam_yaml","");
    template_yaml_=declare_parameter<std::string>("device_template_yaml","");
    image_topic_=declare_parameter<std::string>("image_topic","");
    info_topic_=declare_parameter<std::string>("camera_info_topic","");
    imu_topic_=declare_parameter<std::string>("imu_topic","/imu/data_raw");
    depth_topic_=declare_parameter<std::string>("depth_topic","/depthart/depth");
    depth_wait_ms_=declare_parameter<int>("depth_wait_ms",1000);
    min_depth_valid_fraction_=declare_parameter<double>("min_depth_valid_fraction",0.01);
    if(depth_wait_ms_<50 || depth_wait_ms_>10000 ||
       min_depth_valid_fraction_<0.0 || min_depth_valid_fraction_>1.0)
      throw std::runtime_error("Invalid RGB-D synchronization parameters");
    camera_frame_=declare_parameter<std::string>("camera_optical_frame","");
    imu_frame_=declare_parameter<std::string>("imu_frame","imu_link");
    base_frame_=declare_parameter<std::string>("base_frame","base_link");
    odom_frame_=declare_parameter<std::string>("odom_frame","odom");
    visual_world_frame_=declare_parameter<std::string>("visual_world_frame","xrslam_visual_world");
    visual_body_frame_=declare_parameter<std::string>("visual_body_frame","xrslam_visual_body");
    visual_sparse_map_limit_=declare_parameter<int>("visual_sparse_map_max_points",5000);
    if(visual_sparse_map_limit_<1 || visual_sparse_map_limit_>50000)
      throw std::runtime_error("visual_sparse_map_max_points must be between 1 and 50000");
    if(visual_world_frame_.empty() || visual_body_frame_.empty() ||
       visual_world_frame_==visual_body_frame_ ||
       visual_world_frame_==odom_frame_ || visual_world_frame_==base_frame_ ||
       visual_body_frame_==odom_frame_ || visual_body_frame_==base_frame_ ||
       visual_body_frame_==camera_frame_)
      throw std::runtime_error("visual TF frames must be distinct from robot navigation frames");
    pose_validated_=declare_parameter<bool>("pose_convention_validated",false);
    enable_map_tf_=declare_parameter<bool>("enable_map_tf",false);
    map_frame_=declare_parameter<std::string>("map_frame","map");
    if(visual_world_frame_==map_frame_ || visual_body_frame_==map_frame_)
      throw std::runtime_error("visual TF frames cannot be the map frame");
    map_id_=declare_parameter<std::string>("map_id","");
    if(enable_map_tf_ && (map_id_.empty() || map_frame_.empty() ||
       map_frame_==odom_frame_ || map_frame_==base_frame_))
      throw std::runtime_error("enable_map_tf needs map_id and a distinct map_frame");
    if(enable_map_tf_) {
      const char *env_map_id=std::getenv("XRSLAM_MAP_ID");
      if(!env_map_id || map_id_!=env_map_id)
        throw std::runtime_error("map_id must equal XRSLAM_MAP_ID for this process");
    }
    offset_=declare_parameter<double>("camera_imu_offset_sec",std::numeric_limits<double>::quiet_NaN());
    max_jump_=declare_parameter<double>("max_pose_step_m",1.0);
    max_angular_jump_=declare_parameter<double>("max_pose_step_rad",1.0);
    if(slam_yaml_.empty()||template_yaml_.empty()||image_topic_.empty()||
       info_topic_.empty()||camera_frame_.empty()) {
      throw std::runtime_error(
        "Configure SLAM YAML, device template, image/CameraInfo topics and camera frame");
    }
    const YAML::Node slam=YAML::LoadFile(slam_yaml_);
    const std::string mode=(slam["sensor"] && slam["sensor"]["mode"]) ?
        slam["sensor"]["mode"].as<std::string>() : "";
    if(!leroy_xrslam::supported_sensor_mode(mode))
      throw std::runtime_error(
        "XRSLAM adapter supports sensor.mode=monocular, monocular_imu or rgbd only");
    use_imu_=leroy_xrslam::sensor_mode_requires_imu(mode);
    use_depth_=leroy_xrslam::sensor_mode_requires_depth(mode);
    if(use_imu_ && !is_finite_value(offset_))
      throw std::runtime_error(
        "monocular_imu requires a measured camera_imu_offset_sec");
    if(!use_imu_) {
      // The monocular case has no temporal camera/IMU calibration to apply.
      // Do not silently ignore an explicit nonzero offset or allow
      // unscaled visual-only motion to be published as metric robot odometry.
      if(is_finite_value(offset_) && offset_!=0.0)
        throw std::runtime_error(
          "monocular mode requires camera_imu_offset_sec unset or zero");
      if(pose_validated_ || enable_map_tf_)
        throw std::runtime_error(
          "monocular mode is scale-ambiguous: metric pose/odom and map TF must remain disabled");
      offset_=0.0; // Device YAML requires a numeric field; unused in monocular.
    }
    if(use_depth_ && depth_topic_.empty())
      throw std::runtime_error("RGB-D mode requires depth_topic");
    RCLCPP_INFO(get_logger(),"XRSLAM mode=%s; IMU=%s; pose output gated=%s",
      mode.c_str(),use_imu_?"enabled":"disabled",pose_validated_?"no":"yes");
    auto qos=rclcpp::SensorDataQoS();
    auto latched=rclcpp::QoS(1).reliable().transient_local();
    status_pub_=create_publisher<std_msgs::msg::String>("/xrslam/tracking_status",latched);
    valid_pub_=create_publisher<std_msgs::msg::Bool>("/xrslam/localization_valid",latched);
    map_valid_pub_=create_publisher<std_msgs::msg::Bool>(
      "/xrslam/map_localization_valid",latched);
    map_status_pub_=create_publisher<std_msgs::msg::String>(
      "/xrslam/map_localization_status",latched);
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("/odom",rclcpp::QoS(10));
    pose_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("/xrslam/pose",rclcpp::QoS(10));
    feature_pub_=create_publisher<sensor_msgs::msg::Image>("/xrslam/tracked_features/image",qos);
    landmarks_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/xrslam/landmarks",qos);
    visual_landmarks_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/xrslam/visual_landmarks",qos);
    visual_sparse_map_pub_=create_publisher<sensor_msgs::msg::PointCloud2>(
      "/xrslam/visual_sparse_map",qos);
    visual_landmarks_status_pub_=create_publisher<std_msgs::msg::String>(
      "/xrslam/visual_landmarks_status",latched);
    visual_landmarks_diagnostics_pub_=create_publisher<std_msgs::msg::String>(
      "/xrslam/visual_landmarks_diagnostics",rclcpp::QoS(10).reliable());
    visual_pose_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("/xrslam/visual_pose",rclcpp::QoS(10));
    visual_path_pub_=create_publisher<nav_msgs::msg::Path>("/xrslam/visual_trajectory",rclcpp::QoS(10));
    if(use_imu_) {
      imu_sub_=create_subscription<sensor_msgs::msg::Imu>(imu_topic_,qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr p){ imu(*p); });
    }
    info_sub_=create_subscription<sensor_msgs::msg::CameraInfo>(info_topic_,qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr p){ info(*p); });
    image_sub_=create_subscription<sensor_msgs::msg::Image>(image_topic_,qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr p){ image(p); });
    if(use_depth_) {
      depth_sub_=create_subscription<sensor_msgs::msg::Image>(depth_topic_,qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr p){ depth(p); });
      depth_diagnostics_pub_=create_publisher<std_msgs::msg::String>(
        "/xrslam/depth_sync_status",rclcpp::QoS(10).reliable());
    }
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{ poll(); });
    landmark_diagnostics_timer_=create_wall_timer(
      std::chrono::seconds(2),[this]{ publish_landmark_diagnostics(); });
    if(use_depth_)depth_diagnostics_timer_=create_wall_timer(
      std::chrono::seconds(2),[this]{ publish_depth_diagnostics(); });
    announce("waiting_calibration",false);
    map_announce(enable_map_tf_ ? "waiting_calibration" : "disabled",false);
  }
  ~LeroyXrslam() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(initialized_) XRSLAMDestroy();
    if(!generated_yaml_.empty()) {
      std::error_code ec; std::filesystem::remove(generated_yaml_,ec);
    }
  }
private:
  void announce(const std::string & status,bool valid) {
    if(status==last_status_ && valid==last_valid_) return;
    last_status_=status;last_valid_=valid;
    std_msgs::msg::String s;s.data=status;status_pub_->publish(s);
    std_msgs::msg::Bool b;b.data=valid;valid_pub_->publish(b);
  }
  void landmark_announce(const std::string &status) {
    if(status==last_landmark_status_)return;
    last_landmark_status_=status;
    std_msgs::msg::String msg;msg.data=status;
    visual_landmarks_status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(),"XRSLAM visual landmarks: %s",status.c_str());
  }
  void reset_visual_sparse_map(const char * reason) {
    visual_sparse_map_.clear();
    visual_sparse_map_seq_=0;
    last_visual_sparse_map_publish_=rclcpp::Time(0,0,RCL_ROS_TIME);
    // Explicitly clear RViz's previous snapshot on tracking-session loss.
    sensor_msgs::msg::PointCloud2 empty_cloud;
    empty_cloud.header.stamp=get_clock()->now();
    empty_cloud.header.frame_id=visual_world_frame_;
    sensor_msgs::PointCloud2Modifier modifier(empty_cloud);
    modifier.setPointCloud2FieldsByString(1,"xyz");
    modifier.resize(0);
    visual_sparse_map_pub_->publish(empty_cloud);
    RCLCPP_WARN(get_logger(),"Visual sparse map cleared: %s",reason);
  }
  void publish_landmark_diagnostics() {
    const double age_s=(last_cloud_publish_wall_.nanoseconds()>0)?
      (get_clock()->now()-last_cloud_publish_wall_).seconds():-1.0;
    std_msgs::msg::String msg;
    msg.data="state="+(last_landmark_status_.empty()?std::string("unknown"):last_landmark_status_)+
      " cloud_publishes="+std::to_string(visual_cloud_publish_count_)+
      " last_native_points="+std::to_string(last_native_landmark_count_)+
      " last_finite_points="+std::to_string(last_finite_landmark_count_)+
      " publish_age_s="+std::to_string(age_s)+
      " subscribers="+std::to_string(visual_landmarks_pub_->get_subscription_count())+
      " sparse_map_points="+std::to_string(visual_sparse_map_.size());
    visual_landmarks_diagnostics_pub_->publish(msg);
  }
  void map_announce(const std::string &status,bool valid) {
    if(map_status_==status && map_valid_==valid)return;
    map_status_=status;map_valid_=valid;
    std_msgs::msg::String s;s.data=status;
    map_status_pub_->publish(s);
    std_msgs::msg::Bool b;b.data=valid;
    map_valid_pub_->publish(b);
  }

  bool read_verified_map_alignment(XRSLAMVerifiedMapAlignment &alignment) {
    alignment={};
    if(!enable_map_tf_)return false;
    if(!XRSLAMGetVerifiedMapAlignment(&alignment)) {
      map_announce("xrslam_map_api_unavailable",false);return false;
    }
    if(map_id_!=std::string(alignment.map_id)) {
      map_announce("map_identity_mismatch",false);return false;
    }
    if(!alignment.valid) {
      map_announce("waiting_verified_map_alignment",false);return false;
    }
    if(!is_finite_value(alignment.verified_timestamp) ||
       !is_finite_value(alignment.yaw_rad) || std::abs(alignment.yaw_rad)>6.283186) {
      map_announce("invalid_map_alignment",false);return false;
    }
    for(double v:alignment.translation)
      if(!is_finite_value(v)){map_announce("invalid_map_alignment",false);return false;}
    return true;
  }

  bool init_locked() {
    if(initialized_||!info_)return initialized_;
    geometry_msgs::msg::TransformStamped body_imu,body_cam;
    try {
      if(use_imu_)
        body_imu=tf_buffer_.lookupTransform(base_frame_,imu_frame_,tf2::TimePointZero);
      body_cam=tf_buffer_.lookupTransform(base_frame_,camera_frame_,tf2::TimePointZero);
    }catch(const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Waiting for URDF static TF: %s",e.what());
      return false;
    }
    if((use_imu_ && !valid_quat(body_imu.transform.rotation)) ||
       !valid_quat(body_cam.transform.rotation))
      throw std::runtime_error("Invalid static TF quaternion");
    YAML::Node dev=YAML::LoadFile(template_yaml_);
    const YAML::Node slam=YAML::LoadFile(slam_yaml_);
    if(!slam["sensor"] || !slam["sensor"]["mode"] ||
       leroy_xrslam::sensor_mode_requires_imu(
         slam["sensor"]["mode"].as<std::string>())!=use_imu_ ||
       !leroy_xrslam::supported_sensor_mode(
         slam["sensor"]["mode"].as<std::string>()))
      throw std::runtime_error("XRSLAM sensor mode changed after adapter construction");
    // XRSLAM cam0.q_bc/p_bc encode the camera's orientation/origin in body.
    // Confirm via a nonidentity test before enabling pose output.
    dev["cam0"]["extrinsic"]["q_bc"]=quat4(body_cam.transform.rotation);
    dev["cam0"]["extrinsic"]["p_bc"]=vec3(body_cam.transform.translation);
    if(use_imu_) {
      dev["imu"]["extrinsic"]["q_bi"]=quat4(body_imu.transform.rotation);
      dev["imu"]["extrinsic"]["p_bi"]=vec3(body_imu.transform.translation);
    }
    const auto & c=*info_;
    // Rectified monocular pinhole: P defines actual image geometry.
    if(c.roi.do_rectify || c.p[0]<=0 || c.p[5]<=0 ||
       std::abs(c.p[3])>1e-8 || std::abs(c.p[7])>1e-8)
      throw std::runtime_error("Expected rectified monocular CameraInfo P with no stereo baseline");
    YAML::Node k(YAML::NodeType::Sequence);
    for(double v:{c.p[0],c.p[5],c.p[2],c.p[6]})k.push_back(v);
    dev["cam0"]["intrinsics"]=k;
    YAML::Node resolution(YAML::NodeType::Sequence);
    resolution.push_back(c.width);resolution.push_back(c.height);
    dev["cam0"]["resolution"]=resolution;
    dev["cam0"]["camera_model"]="pinhole";
    dev["cam0"]["camera_distortion_flag"]=0;
    YAML::Node zeros(YAML::NodeType::Sequence);
    for(int i=0;i<4;++i)zeros.push_back(0.0);
    dev["cam0"]["distortion"]=zeros;
    dev["cam0"]["time_offset"]=offset_;
    auto tmp=std::filesystem::temp_directory_path()/
      ("leroy_xrslam_"+std::to_string(reinterpret_cast<std::uintptr_t>(this))+".yaml");
    generated_yaml_=tmp.string();
    {
      std::ofstream out(generated_yaml_);
      if(!out) throw std::runtime_error("Could not write XRSLAM runtime device configuration");
      out<<dev;
    }
    void * config=nullptr;
    if(!XRSLAMCreate(slam_yaml_.c_str(),generated_yaml_.c_str(),"","leroy_xrslam",&config))
      throw std::runtime_error("XRSLAMCreate failed");
    initialized_=true;
    if((XRSLAMRequiresIMU(config)!=0)!=use_imu_)
      throw std::runtime_error("XRSLAM IMU requirement disagrees with sensor.mode");
    announce("initializing",false);
    return true;
  }
  void info(const sensor_msgs::msg::CameraInfo & c) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(initialized_)return;
    if(c.header.frame_id!=camera_frame_ || c.width==0 || c.height==0 ||
       !is_finite_value(c.p[0])||!is_finite_value(c.p[5]))return;
    info_=c;
  }
  void imu(const sensor_msgs::msg::Imu & m) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!use_imu_)return;
    const double t=seconds(m.header.stamp);
    if(m.header.frame_id!=imu_frame_||t<=0||
       !is_finite_value(m.angular_velocity.x)||!is_finite_value(m.angular_velocity.y)||!is_finite_value(m.angular_velocity.z)||
       !is_finite_value(m.linear_acceleration.x)||!is_finite_value(m.linear_acceleration.y)||!is_finite_value(m.linear_acceleration.z))return;
    if(last_imu_time_>0 && t<=last_imu_time_) {
      if(t<last_imu_time_-0.1) fail("imu_clock_discontinuity");
      return;
    }
    if(last_imu_time_>0 && t-last_imu_time_>0.3 && initialized_) fail("imu_gap");
    last_imu_time_=t;
    if(!initialized_||failed_)return;
    XRSLAMGyroscope gyro{};
    gyro.timestamp=t;gyro.data[0]=m.angular_velocity.x;
    gyro.data[1]=m.angular_velocity.y;gyro.data[2]=m.angular_velocity.z;
    XRSLAMAcceleration accel{};
    accel.timestamp=t;accel.data[0]=m.linear_acceleration.x;
    accel.data[1]=m.linear_acceleration.y;accel.data[2]=m.linear_acceleration.z;
    // Both channels, exactly one ROS IMU stamp.
    XRSLAMPushSensorData(XRSLAM_SENSOR_GYROSCOPE,&gyro);
    XRSLAMPushSensorData(XRSLAM_SENSOR_ACCELERATION,&accel);
    // No camera frame should precede the first IMU sample that was actually
    // handed to XRSLAM. Samples received before XRSLAMCreate are not forwarded.
    if(first_forwarded_imu_time_<0)first_forwarded_imu_time_=t;
    drain();
  }
  void publish_depth_diagnostics() {
    std_msgs::msg::String out;
    out.data="paired="+std::to_string(depth_pairs_)+
      " dropped_rgb="+std::to_string(depth_dropped_rgb_)+
      " late_depth="+std::to_string(depth_late_)+
      " invalid_depth="+std::to_string(depth_invalid_)+
      " pending_rgb="+std::to_string(pending_.size())+
      " pending_depth="+std::to_string(pending_depth_.size());
    depth_diagnostics_pub_->publish(out);
  }
  bool valid_depth(const sensor_msgs::msg::Image &m) const {
    if(m.header.frame_id!=camera_frame_ || m.encoding!="32FC1" ||
       m.is_bigendian!=0 || m.width==0 || m.height==0 ||
       m.step<m.width*sizeof(float) ||
       m.data.size()<static_cast<size_t>(m.step)*m.height ||
       !info_ || m.width!=info_->width || m.height!=info_->height)return false;
    size_t sampled=0,valid=0;
    for(size_t y=0;y<m.height;y+=8)for(size_t x=0;x<m.width;x+=8) {
      float z;
      std::memcpy(&z,m.data.data()+static_cast<size_t>(y)*m.step+x*sizeof(float),sizeof(z));
      ++sampled;
      if(std::isfinite(z)&&z>0.05f&&z<50.0f)++valid;
    }
    return sampled>0 && static_cast<double>(valid)/sampled>=min_depth_valid_fraction_;
  }
  void depth(sensor_msgs::msg::Image::ConstSharedPtr m) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(failed_ || !initialized_ || !use_depth_)return;
    const auto stamp=rclcpp::Time(m->header.stamp).nanoseconds();
    if(stamp<=0 || !valid_depth(*m)) {++depth_invalid_;return;}
    if(stamp<=last_image_time_ns_) {++depth_late_;return;}
    for(const auto &d:pending_depth_)
      if(rclcpp::Time(d->header.stamp).nanoseconds()==stamp)return;
    pending_depth_.push_back(m);
    while(pending_depth_.size()>8)pending_depth_.pop_front();
    drain();
  }
  void image(sensor_msgs::msg::Image::ConstSharedPtr m) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(failed_)return;
    const double t=seconds(m->header.stamp);
    if(m->header.frame_id!=camera_frame_ || t<=0)return;
    // Reject pre-initialization camera frames instead of filling the
    // runtime overflow-protected queue while TF becomes available.
    // VIO additionally needs a first forwarded IMU pair; pure monocular
    // deliberately does not subscribe to, await or push IMU samples.
    if(!leroy_xrslam::can_queue_camera(
         initialized_, use_imu_, t, offset_, first_forwarded_imu_time_))return;
    if(!pending_.empty() && t<=seconds(pending_.back()->header.stamp))return;
    if(t<=last_image_time_)return;
    if(pending_.size()>=8){
      if(!use_depth_){fail("camera_queue_overflow");return;}
      pending_.pop_front();++depth_dropped_rgb_;
    }
    pending_.push_back(m);
    if(use_depth_)pending_arrivals_.push_back(std::chrono::steady_clock::now());
    while(pending_arrivals_.size()>pending_.size())pending_arrivals_.pop_front();
    drain();
  }
  void poll() {
    std::lock_guard<std::mutex> lock(mutex_);
    if(failed_)return;
    if(!initialized_) {
      try{ if(!init_locked())return; }catch(const std::exception & e){
        RCLCPP_ERROR(get_logger(),"Calibration/initialization failure: %s",e.what());
        fail("configuration_error");return;
      }
    }
    drain();
    read_pose();
  }
  void drain() {
    if(!initialized_||failed_)return;
    while(!pending_.empty()) {
      const auto m=pending_.front();
      const double t=seconds(m->header.stamp);
      sensor_msgs::msg::Image::ConstSharedPtr matched_depth;
      if(use_depth_) {
        const auto stamp=rclcpp::Time(m->header.stamp).nanoseconds();
        while(!pending_depth_.empty() &&
          rclcpp::Time(pending_depth_.front()->header.stamp).nanoseconds()<stamp)
          {pending_depth_.pop_front();++depth_late_;}
        auto match=std::find_if(pending_depth_.begin(),pending_depth_.end(),
          [stamp](const auto &d){return rclcpp::Time(d->header.stamp).nanoseconds()==stamp;});
        if(match==pending_depth_.end()) {
          if(!pending_arrivals_.empty() &&
             std::chrono::steady_clock::now()-pending_arrivals_.front()>
               std::chrono::milliseconds(depth_wait_ms_)) {
            pending_.pop_front();pending_arrivals_.pop_front();++depth_dropped_rgb_;
            continue;
          }
          break;
        }
        matched_depth=*match;
        pending_depth_.erase(match);
      }
      if(leroy_xrslam::await_imu_for_camera(
           use_imu_, last_imu_time_, t, offset_))break;
      if(t<=last_image_time_){pending_.pop_front();continue;}
      if(m->width!=info_->width||m->height!=info_->height) {fail("image_geometry_changed");return;}
      // Preserve RGB/BGR conventions expected by XRSLAMManager::PushImage.
      int channels=0;
      if(m->encoding=="mono8") channels=1;
      else if(m->encoding=="bgr8"||m->encoding=="rgb8")channels=3;
      else if(m->encoding=="bgra8"||m->encoding=="rgba8")channels=4;
      else {fail("unsupported_image_encoding");return;}
      if(m->step<m->width*static_cast<unsigned>(channels) ||
         m->data.size()<static_cast<size_t>(m->step)*m->height) {
        fail("malformed_image");return;
      }
      std::vector<unsigned char> converted;
      const unsigned char* data=m->data.data();
      if(m->encoding=="rgb8"||m->encoding=="rgba8") {
        converted=m->data;
        for(size_t row=0;row<m->height;++row)
          for(size_t col=0;col<m->width;++col) {
            auto * px=converted.data()+row*m->step+col*channels;
            std::swap(px[0],px[2]);
          }
        data=converted.data();
      }
      XRSLAMImage camera{};
      camera.data=const_cast<unsigned char*>(data);
      camera.timeStamp=t;camera.camera_id=0;
      camera.channel=channels;camera.stride=static_cast<int>(m->step);
      if(use_depth_) {
        XRSLAMDepthImage dep{};
        dep.data_f32=reinterpret_cast<float*>(const_cast<unsigned char*>(matched_depth->data.data()));
        dep.timeStamp=t;
        dep.width=static_cast<int>(matched_depth->width);
        dep.height=static_cast<int>(matched_depth->height);
        dep.stride=static_cast<int>(matched_depth->step);
        dep.format=XRSLAM_DEPTH_FLOAT32;
        dep.source=XRSLAM_DEPTH_MONOCULAR_METRIC;
        XRSLAMPushSensorData(XRSLAM_SENSOR_DEPTH_CAMERA,&dep);
        ++depth_pairs_;
      }
      XRSLAMPushSensorData(XRSLAM_SENSOR_CAMERA,&camera);
      XRSLAMRunOneFrame();
      publish_visualization(*m);
      ++submitted_camera_frames_;
      // A stationary scene can remain in 'initializing' even when images
      // are arriving. Log actual XRSLAM submissions, not just DDS endpoints.
      if(submitted_camera_frames_==1 || submitted_camera_frames_%300==0)
        RCLCPP_INFO(get_logger(),"Submitted %llu camera frames to XRSLAM (%s)",
          static_cast<unsigned long long>(submitted_camera_frames_),
          use_imu_?"monocular_imu":"monocular");
      last_image_time_=t;
      pending_.pop_front();
      if(use_depth_ && !pending_arrivals_.empty())pending_arrivals_.pop_front();
      last_image_time_ns_=rclcpp::Time(m->header.stamp).nanoseconds();
    }
  }
  // Export XRSLAM-owned observations only; any visual-world pose/TF stays
  // isolated from the metric navigation frame tree.
  void publish_visualization(const sensor_msgs::msg::Image &source) {
    XRSLAMState state=XRSLAM_STATE_INITIALIZING;
    XRSLAMGetResult(XRSLAM_RESULT_STATE,&state);
    if(state==XRSLAM_STATE_TRACKING_FAIL)return;

    // Feature tracking precedes successful monocular map initialization.
    // Expose XRSLAM's real front-end tracks while initializing, without
    // implying valid 3D pose or changing the metric-pose authorization gate.
    if(feature_pub_->get_subscription_count()>0) {
      XRSLAMFeatures features;
      XRSLAMGetResult(XRSLAM_RESULT_FEATURES,&features);
      if(!features.pos.empty()) {
      sensor_msgs::msg::Image output;
      output.header=source.header;
      output.height=source.height;output.width=source.width;
      output.encoding="rgb8";output.is_bigendian=0;
      output.step=source.width*3;
      output.data.resize(static_cast<size_t>(output.step)*output.height);
      const unsigned channels=(source.encoding=="mono8")?1:
          (source.encoding=="bgr8"||source.encoding=="rgb8")?3:4;
      const bool bgr=(source.encoding=="bgr8"||source.encoding=="bgra8");
      for(size_t y=0;y<source.height;++y)
        for(size_t x=0;x<source.width;++x) {
          const auto *p=source.data.data()+y*source.step+x*channels;
          auto *d=output.data.data()+y*output.step+x*3;
          d[0]=(channels==1)?p[0]:p[bgr?2:0];
          d[1]=(channels==1)?p[0]:p[1];
          d[2]=(channels==1)?p[0]:p[bgr?0:2];
        }
      for(const auto &point:features.pos) {
        if(!std::isfinite(point.x)||!std::isfinite(point.y))continue;
        const int cx=static_cast<int>(std::round(point.x));
        const int cy=static_cast<int>(std::round(point.y));
        if(cx<2||cy<2||cx>=static_cast<int>(source.width)-2||
           cy>=static_cast<int>(source.height)-2)continue;
        for(int step=-2;step<=2;++step) {
          for(const auto [x,y] : {std::pair<int,int>{cx+step,cy},
                                  std::pair<int,int>{cx,cy+step}}) {
            auto *px=output.data.data()+y*output.step+x*3;
            px[0]=255;px[1]=210;px[2]=0;
          }
        }
      }
      feature_pub_->publish(output);
      }
    }

    // Visual-only reconstruction is unscaled and must never be advertised
    // as metric odom. Keep the previously validated odom cloud unchanged.
    const bool publish_visual = visual_landmarks_pub_->get_subscription_count()>0;
    const bool publish_metric = pose_validated_ && landmarks_pub_->get_subscription_count()>0;
    if(state!=XRSLAM_STATE_TRACKING_SUCCESS) {
      if(visual_map_was_tracking_) {
        reset_visual_sparse_map("native tracking exited TRACKING_SUCCESS");
        visual_map_was_tracking_=false;
      }
      landmark_announce(state==XRSLAM_STATE_INITIALIZING?
        "waiting_for_native_tracking":"native_tracking_failed");
      return;
    }
    visual_map_was_tracking_=true;
    const bool publish_sparse=visual_sparse_map_pub_->get_subscription_count()>0;
    if(!publish_visual && !publish_metric && !publish_sparse) {
      landmark_announce("no_pointcloud_subscribers");
      return;
    }
    XRSLAMLandmarks landmarks{};
    XRSLAMGetResult(XRSLAM_RESULT_LANDMARKS,&landmarks);
    const size_t count=(landmarks.num_landmarks>0 && landmarks.landmarks)?
        static_cast<size_t>(landmarks.num_landmarks):0;
    last_native_landmark_count_=count;
    size_t finite=0;
    std::vector<std::array<float,3>> current_points;
    current_points.reserve(count);
    for(size_t i=0;i<count;++i) {
      const auto &p=landmarks.landmarks[i];
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))
        continue;
      ++finite;
      current_points.push_back({static_cast<float>(p.x),
                                static_cast<float>(p.y),
                                static_cast<float>(p.z)});
      // Reuse the native identity and only retain triangulated geometry.
      if(p.triangulated && p.track_id!=0) {
        auto &entry=visual_sparse_map_[p.track_id];
        entry.x=static_cast<float>(p.x);
        entry.y=static_cast<float>(p.y);
        entry.z=static_cast<float>(p.z);
        entry.seq=++visual_sparse_map_seq_;
      }
    }
    delete[] landmarks.landmarks;
    last_finite_landmark_count_=finite;
    // Bound memory deterministically by evicting least recently observed
    // track IDs; this is a session cache, not a globally corrected map.
    while(visual_sparse_map_.size()>static_cast<size_t>(visual_sparse_map_limit_)) {
      auto oldest=std::min_element(visual_sparse_map_.begin(),
        visual_sparse_map_.end(),[](const auto &a,const auto &b){
          return a.second.seq<b.second.seq;
        });
      visual_sparse_map_.erase(oldest);
    }
    if(count==0)landmark_announce("native_snapshot_empty");
    else if(finite==0)landmark_announce("native_snapshot_all_nonfinite");
    else landmark_announce("publishing_finite_points");

    auto make_cloud=[&](size_t n) {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header=source.header;
      cloud.header.frame_id=visual_world_frame_;
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2FieldsByString(1,"xyz");
      modifier.resize(n);
      return cloud;
    };
    if(finite>0 && (publish_visual || publish_metric)) {
      auto cloud=make_cloud(current_points.size());
      sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
      for(const auto &p:current_points) {
        *x=p[0];*y=p[1];*z=p[2];++x;++y;++z;
      }
      if(publish_visual) {
        visual_landmarks_pub_->publish(cloud);
        ++visual_cloud_publish_count_;
        last_cloud_publish_wall_=get_clock()->now();
      }
      if(publish_metric) {
        cloud.header.frame_id=odom_frame_;
        landmarks_pub_->publish(cloud);
      }
    }
    // Publish cached landmarks at a bounded cadence, including while the
    // latest sliding window has zero tracks. Never publish synthetic points.
    if(publish_sparse && !visual_sparse_map_.empty() &&
       (get_clock()->now()-last_visual_sparse_map_publish_).seconds()>=0.5) {
      auto cloud=make_cloud(visual_sparse_map_.size());
      sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
      for(const auto &[id,p]:visual_sparse_map_) {
        (void)id;
        *x=p.x;*y=p.y;*z=p.z;++x;++y;++z;
      }
      visual_sparse_map_pub_->publish(cloud);
      last_visual_sparse_map_publish_=get_clock()->now();
    }
  }

  void read_pose() {
    if(!initialized_||failed_)return;
    XRSLAMState state=XRSLAM_STATE_INITIALIZING;
    XRSLAMGetResult(XRSLAM_RESULT_STATE,&state);
    if(state==XRSLAM_STATE_TRACKING_FAIL){fail("tracking_lost");return;}
    if(state!=XRSLAM_STATE_TRACKING_SUCCESS){
      if(last_pose_time_>0){fail("tracking_session_reset");return;}
      announce("initializing",false);
      map_announce(enable_map_tf_?"waiting_tracking":"disabled",false);
      return;
    }
    XRSLAMVerifiedMapAlignment map_alignment{};
    const bool map_ready=read_verified_map_alignment(map_alignment);
    if(last_pose_time_>0 && enable_map_tf_ &&
       prior_map_epoch_!=0 && map_alignment.tracking_session_epoch!=prior_map_epoch_) {
      fail("map_tracking_epoch_changed");return;
    }
    XRSLAMPose p{};
    XRSLAMGetResult(XRSLAM_RESULT_BODY_POSE,&p);
    const double qnorm=p.quaternion[0]*p.quaternion[0]+p.quaternion[1]*p.quaternion[1]+
      p.quaternion[2]*p.quaternion[2]+p.quaternion[3]*p.quaternion[3];
    if(p.timestamp<=0||!is_finite_value(p.timestamp)||qnorm<0.99||qnorm>1.01||
       !is_finite_value(p.translation[0])||!is_finite_value(p.translation[1])||!is_finite_value(p.translation[2]))return;
    if(p.timestamp<=last_pose_time_)return;
    // Publish a separate session-local *unscaled* visual reconstruction.
    // No TF is added between xrslam_visual_world and the robot/nav graph.
    // XRSLAM body pose convention remains unverified for metric authority.
    if(p.timestamp>last_visual_pose_time_) {
      geometry_msgs::msg::PoseStamped visual_pose;
      visual_pose.header.stamp=rclcpp::Time(
        static_cast<int64_t>(std::llround(p.timestamp*1e9)),RCL_ROS_TIME);
      visual_pose.header.frame_id=visual_world_frame_;
      visual_pose.pose.position.x=p.translation[0];
      visual_pose.pose.position.y=p.translation[1];
      visual_pose.pose.position.z=p.translation[2];
      visual_pose.pose.orientation.x=p.quaternion[0];
      visual_pose.pose.orientation.y=p.quaternion[1];
      visual_pose.pose.orientation.z=p.quaternion[2];
      visual_pose.pose.orientation.w=p.quaternion[3];
      visual_pose_pub_->publish(visual_pose);
      // Standalone TF tree: never attach the unscaled body to base_link or odom.
      geometry_msgs::msg::TransformStamped visual_tf;
      visual_tf.header=visual_pose.header;
      visual_tf.child_frame_id=visual_body_frame_;
      visual_tf.transform.translation.x=visual_pose.pose.position.x;
      visual_tf.transform.translation.y=visual_pose.pose.position.y;
      visual_tf.transform.translation.z=visual_pose.pose.position.z;
      visual_tf.transform.rotation=visual_pose.pose.orientation;
      tf_broadcaster_.sendTransform(visual_tf);
      if(visual_path_.header.frame_id!=visual_world_frame_) {
        visual_path_.header.frame_id=visual_world_frame_;
        visual_path_.poses.clear();
      }
      visual_path_.header.stamp=visual_pose.header.stamp;
      visual_path_.poses.push_back(visual_pose);
      if(visual_path_.poses.size()>1000)
        visual_path_.poses.erase(visual_path_.poses.begin());
      if(visual_path_pub_->get_subscription_count()>0)
        visual_path_pub_->publish(visual_path_);
      last_visual_pose_time_=p.timestamp;
    }
    if(!pose_validated_) {
      announce("pose_convention_unverified",false);
      map_announce("pose_convention_unverified",false);
      return;
    }
    const rclcpp::Time stamp(static_cast<int64_t>(std::llround(p.timestamp*1e9)),RCL_ROS_TIME);
    geometry_msgs::msg::Pose pose;
    pose.position.x=p.translation[0];pose.position.y=p.translation[1];pose.position.z=p.translation[2];
    pose.orientation.x=p.quaternion[0];pose.orientation.y=p.quaternion[1];
    pose.orientation.z=p.quaternion[2];pose.orientation.w=p.quaternion[3];
    tf2::Quaternion q(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
    q.normalize();
    if(last_pose_time_>0) {
      const double dt=p.timestamp-last_pose_time_;
      const double dx=pose.position.x-last_pose_.position.x;
      const double dy=pose.position.y-last_pose_.position.y;
      const double dz=pose.position.z-last_pose_.position.z;
      tf2::Quaternion prev(last_pose_.orientation.x,last_pose_.orientation.y,
                            last_pose_.orientation.z,last_pose_.orientation.w);
      prev.normalize();
      if(dt>0.5||std::sqrt(dx*dx+dy*dy+dz*dz)>max_jump_||
         prev.angleShortestPath(q)>max_angular_jump_) {
        fail("pose_discontinuity");return;
      }
    }
    nav_msgs::msg::Odometry odom;
    odom.header.stamp=stamp;odom.header.frame_id=odom_frame_;odom.child_frame_id=base_frame_;
    odom.pose.pose=pose;
    // The C API exposes no covariance. Do not claim estimator certainty.
    for(int i=0;i<36;++i){odom.pose.covariance[i]=0;odom.twist.covariance[i]=0;}
    for(int i=0;i<6;++i){odom.pose.covariance[i*6+i]=100;odom.twist.covariance[i*6+i]=100;}
    if(last_pose_time_>0) {
      const double dt=p.timestamp-last_pose_time_;
      tf2::Vector3 displacement(pose.position.x-last_pose_.position.x,
                                pose.position.y-last_pose_.position.y,
                                pose.position.z-last_pose_.position.z);
      tf2::Quaternion prev(last_pose_.orientation.x,last_pose_.orientation.y,
                            last_pose_.orientation.z,last_pose_.orientation.w);
      prev.normalize();
      const tf2::Vector3 local=quatRotate(prev.inverse(),displacement)/dt;
      odom.twist.twist.linear.x=local.x();odom.twist.twist.linear.y=local.y();odom.twist.twist.linear.z=local.z();
      auto delta=prev.inverse()*q;delta.normalize();
      const double angle=delta.getAngleShortestPath();
      const auto axis=delta.getAxis();
      odom.twist.twist.angular.x=axis.x()*angle/dt;
      odom.twist.twist.angular.y=axis.y()*angle/dt;
      odom.twist.twist.angular.z=axis.z()*angle/dt;
    }
    // First velocity is unknown: no zero-as-measurement output.
    if(last_pose_time_<=0){last_pose_time_=p.timestamp;last_pose_=pose;announce("tracking_velocity_warmup",false);return;}
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header=odom.header;stamped.pose=pose;
    pose_pub_->publish(stamped);
    odom_pub_->publish(odom);
    geometry_msgs::msg::TransformStamped tf;
    tf.header=odom.header;tf.child_frame_id=base_frame_;
    tf.transform.translation.x=pose.position.x;
    tf.transform.translation.y=pose.position.y;
    tf.transform.translation.z=pose.position.z;
    tf.transform.rotation=pose.orientation;
    tf_broadcaster_.sendTransform(tf);
    last_pose_time_=p.timestamp;last_pose_=pose;
    announce("tracking",true);
    if(map_ready) {
      // Compare the status for the same source session again immediately
      // before publishing the corresponding dynamic transform.
      XRSLAMVerifiedMapAlignment second{};
      const bool confirmed=read_verified_map_alignment(second);
      if(confirmed &&
         second.tracking_session_epoch==map_alignment.tracking_session_epoch &&
         second.reference_session_id==map_alignment.reference_session_id &&
         second.verified_timestamp<=p.timestamp+1e-6) {
        // T_map_odom = T_map_vio * T_vio_base * inverse(T_odom_base).
        // Here the adapter's T_odom_base is the same local body pose,
        // but retain the composition to document and test frame direction.
        tf2::Quaternion vio_q(pose.orientation.x,pose.orientation.y,
                              pose.orientation.z,pose.orientation.w);
        vio_q.normalize();
        const tf2::Vector3 vio_xyz(pose.position.x,pose.position.y,pose.position.z);
        const tf2::Transform T_vio_base(vio_q,vio_xyz);
        const tf2::Transform T_odom_base(vio_q,vio_xyz);
        tf2::Quaternion map_q(tf2::Vector3(0,0,1),second.yaw_rad);
        const tf2::Transform T_map_vio(
          map_q,tf2::Vector3(second.translation[0],second.translation[1],
                            second.translation[2]));
        const tf2::Transform T_map_odom =
          T_map_vio*T_vio_base*T_odom_base.inverse();
        geometry_msgs::msg::TransformStamped global_tf;
        global_tf.header.stamp=stamp;
        global_tf.header.frame_id=map_frame_;
        global_tf.child_frame_id=odom_frame_;
        global_tf.transform=tf2::toMsg(T_map_odom);
        tf_broadcaster_.sendTransform(global_tf);
        prior_map_epoch_=second.tracking_session_epoch;
        map_announce("verified:"+map_id_,true);
      }else{
        map_announce("map_alignment_changed_or_not_current",false);
      }
    }else if(enable_map_tf_){
      map_announce("waiting_verified_map_alignment",false);
    }
  }
  void fail(const std::string & reason) {
    if(failed_)return;
    failed_=true;pending_.clear();
    reset_visual_sparse_map(reason.c_str());
    visual_map_was_tracking_=false;
    announce(reason,false);
    map_announce(reason,false);
    RCLCPP_ERROR(get_logger(),"Odometry inhibited: %s (restart requires supervised validation)",reason.c_str());
  }

  std::mutex mutex_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr feature_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr visual_landmarks_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr visual_sparse_map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr visual_landmarks_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr visual_landmarks_diagnostics_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr depth_diagnostics_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr visual_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr visual_path_pub_;
  nav_msgs::msg::Path visual_path_;
  struct VisualMapPoint {float x=0,y=0,z=0; std::uint64_t seq=0;};
  std::map<std::uint64_t,VisualMapPoint> visual_sparse_map_;
  std::uint64_t visual_sparse_map_seq_=0;
  rclcpp::Time last_visual_sparse_map_publish_{0,0,RCL_ROS_TIME};
  int visual_sparse_map_limit_=5000;
  bool visual_map_was_tracking_=false;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr map_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr landmark_diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr depth_diagnostics_timer_;
  std::optional<sensor_msgs::msg::CameraInfo> info_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> pending_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> pending_depth_;
  std::deque<std::chrono::steady_clock::time_point> pending_arrivals_;
  std::uint64_t depth_pairs_=0,depth_dropped_rgb_=0,depth_late_=0,depth_invalid_=0;
  std::int64_t last_image_time_ns_=0;
  int depth_wait_ms_=1000;
  double min_depth_valid_fraction_=0.01;
  std::string slam_yaml_,template_yaml_,image_topic_,info_topic_,imu_topic_,depth_topic_;
  std::string camera_frame_,imu_frame_,base_frame_,odom_frame_,generated_yaml_;
  std::string map_frame_,map_id_,map_status_,visual_world_frame_,visual_body_frame_;
  std::string last_status_,last_landmark_status_;
  bool last_valid_=false,initialized_=false,failed_=false,pose_validated_=false;
  bool enable_map_tf_=false,map_valid_=false,use_imu_=true,use_depth_=false;
  std::uint64_t prior_map_epoch_=0;
  double offset_=0,max_jump_=1,max_angular_jump_=1;
  double last_imu_time_=-1,last_image_time_=-1,last_pose_time_=-1;
  double last_visual_pose_time_=-1;
  double first_forwarded_imu_time_=-1;
  std::uint64_t submitted_camera_frames_=0;
  std::uint64_t visual_cloud_publish_count_=0;
  size_t last_native_landmark_count_=0,last_finite_landmark_count_=0;
  rclcpp::Time last_cloud_publish_wall_{0,0,RCL_ROS_TIME};
  geometry_msgs::msg::Pose last_pose_;
};

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<LeroyXrslam>();
    rclcpp::spin(node);
  }catch(const std::exception & e) {
    std::fprintf(stderr,"leroy_xrslam fatal: %s\n",e.what());
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
