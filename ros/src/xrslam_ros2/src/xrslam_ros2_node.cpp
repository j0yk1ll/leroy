#include <xrslam_ros2/device_config.hpp>

// The XRSLAM C API header uses std::vector in one C++-only structure.
#include <cstddef>
#include <vector>
#include <XRSLAM.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <unistd.h>

using namespace std::chrono_literals;

namespace {
struct PoseSample {
    double timestamp = 0;
    tf2::Vector3 position;
    tf2::Quaternion orientation;
};

bool finite(double v) { return std::isfinite(v); }

xrslam_ros2::RigidExtrinsic extrinsic_from_tf(
    const geometry_msgs::msg::TransformStamped &tf) {
    xrslam_ros2::RigidExtrinsic e;
    e.translation = {tf.transform.translation.x,
                     tf.transform.translation.y,
                     tf.transform.translation.z};
    e.quaternion_xyzw = {tf.transform.rotation.x,
                         tf.transform.rotation.y,
                         tf.transform.rotation.z,
                         tf.transform.rotation.w};
    return e;
}

std::array<double,3> variance_parameter(rclcpp::Node &node, const std::string &name,
                                       const std::vector<double> &fallback) {
    const auto a = node.declare_parameter<std::vector<double>>(name, fallback);
    if (a.size() != 3)
        throw std::invalid_argument(name + " must have exactly three positive variances");
    return {a[0], a[1], a[2]};
}

// Images are already rectified by image_proc. Convert only the pixel channel
// order; never undistort a second time. XRSLAM copies the buffer on push.
bool input_image(const sensor_msgs::msg::Image &msg,
                 std::vector<std::uint8_t> &pixels,
                 int &channels, std::size_t &stride) {
    if (msg.width == 0 || msg.height == 0) return false;
    const auto enc = msg.encoding;
    channels = (enc == "mono8" || enc == "8UC1") ? 1 :
               (enc == "rgb8" || enc == "bgr8") ? 3 :
               (enc == "rgba8" || enc == "bgra8") ? 4 : 0;
    if (!channels) return false;
    const auto row = std::size_t(msg.width) * std::size_t(channels);
    if (msg.step < row || msg.data.size() < std::size_t(msg.step) * msg.height)
        return false;
    stride = row;
    pixels.resize(row * msg.height);
    for (std::size_t y = 0; y < msg.height; ++y) {
        const auto *src = msg.data.data() + y * msg.step;
        auto *dst = pixels.data() + y * row;
        std::memcpy(dst, src, row);
    }
    if (enc == "rgb8" || enc == "rgba8") {
        for (std::size_t i = 0; i < pixels.size(); i += channels)
            std::swap(pixels[i], pixels[i+2]); // XRSLAM wants BGR/BGRA
    }
    return true;
}
} // namespace

class XRSLAMRos2Node final : public rclcpp::Node {
public:
    XRSLAMRos2Node() : Node("xrslam_ros2") {
        using sensor_msgs::msg::Image;
        using sensor_msgs::msg::Imu;
        using sensor_msgs::msg::CameraInfo;
        image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_rect");
        imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data_raw");
        camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_optical_frame");
        imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
        slam_config_path_ = declare_parameter<std::string>("slam_config_path", "");
        max_linear_speed_ = declare_parameter<double>("maximum_plausible_linear_speed_mps", 4.0);
        max_angular_speed_ = declare_parameter<double>("maximum_plausible_angular_speed_rps", 8.0);
        stale_timeout_s_ = declare_parameter<double>("stale_camera_timeout_s", 0.7);
        pixel_variance_ = declare_parameter<double>("pixel_variance", 0.5);
        time_offset_ = declare_parameter<double>("camera_time_offset_s", 0.0);
        pose_stddev_xy_ = declare_parameter<double>("illustrative_pose_stddev_xy_m", 0.20);
        pose_stddev_z_ = declare_parameter<double>("illustrative_pose_stddev_z_m", 0.35);
        pose_stddev_yaw_ = declare_parameter<double>("illustrative_pose_stddev_yaw_rad", 0.25);
        twist_stddev_ = declare_parameter<double>("illustrative_twist_stddev", 0.5);
        config_.gyro_variance = variance_parameter(*this, "imu_cov_g_diag", {2.87913024e-8,2.87913024e-8,2.87913024e-8});
        config_.accel_variance = variance_parameter(*this, "imu_cov_a_diag", {4e-6,4e-6,4e-6});
        config_.gyro_bias_variance = variance_parameter(*this, "imu_cov_bg_diag", {3.76088449e-10,3.76088449e-10,3.76088449e-10});
        config_.accel_bias_variance = variance_parameter(*this, "imu_cov_ba_diag", {9e-6,9e-6,9e-6});
        if (slam_config_path_.empty() || !std::filesystem::exists(slam_config_path_))
            throw std::invalid_argument("slam_config_path must be an existing XRSLAM YAML file");
        if (base_frame_.empty() || odom_frame_.empty() || camera_frame_.empty() ||
            imu_frame_.empty() || base_frame_ == odom_frame_ ||
            max_linear_speed_ <= 0 || max_angular_speed_ <= 0 || stale_timeout_s_ <= 0 ||
            pose_stddev_xy_ <= 0 || pose_stddev_z_ <= 0 || pose_stddev_yaw_ <= 0 ||
            twist_stddev_ <= 0)
            throw std::invalid_argument("invalid ROS frames or odometry guard parameters");
        config_.pixel_variance = pixel_variance_;
        config_.camera_time_offset_s = time_offset_;

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        // Dedicated TF listener thread allows waiting for /tf_static without
        // blocking the camera+IMU subscription executor.
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10).reliable());
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/xrslam/pose", 10);
        diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/xrslam/tracking_status", 10);
        // Leroy IMU publishes BEST_EFFORT: a RELIABLE subscription would not match.
        auto sensor_qos = rclcpp::SensorDataQoS();
        imu_sub_ = create_subscription<Imu>(imu_topic_, sensor_qos,
            [this](Imu::ConstSharedPtr msg) { on_imu(std::move(msg)); });
        image_sub_ = create_subscription<Image>(image_topic_, sensor_qos,
            [this](Image::ConstSharedPtr msg) { on_image(std::move(msg)); });
        camera_info_sub_ = create_subscription<CameraInfo>(camera_info_topic_, sensor_qos,
            [this](CameraInfo::ConstSharedPtr msg) { on_camera_info(std::move(msg)); });
        timer_ = create_wall_timer(100ms, [this] { tick(); });
        RCLCPP_WARN(get_logger(),
            "0124a integration: bridge odometry+TF must be DISABLED. "
            "IMU noise/covariances are provisional until calibrated. "
            "No map->odom or persistent localization is published.");
    }
    ~XRSLAMRos2Node() override {
        if (created_) XRSLAMDestroy();
    }

private:
    void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        if (created_ || fault_) return;
        if (msg->header.frame_id != camera_frame_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "CameraInfo frame '%s' != configured optical frame '%s'",
                msg->header.frame_id.c_str(), camera_frame_.c_str());
            return;
        }
        camera_info_ = std::move(msg);
        attempt_init();
    }
    void on_imu(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        if (fault_) return;
        if (msg->header.frame_id != imu_frame_) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                "IMU frame '%s' != '%s': sample dropped", msg->header.frame_id.c_str(), imu_frame_.c_str());
            return;
        }
        const double t = rclcpp::Time(msg->header.stamp).seconds();
        if (!finite(t) || t <= 0.0 || t <= last_received_imu_t_) return;
        const auto &a = msg->linear_acceleration;
        const auto &w = msg->angular_velocity;
        if (!finite(a.x) || !finite(a.y) || !finite(a.z) ||
            !finite(w.x) || !finite(w.y) || !finite(w.z)) return;
        last_received_imu_t_ = t;
        imus_.push_back(std::move(msg));
        while (imus_.size() > 1000) imus_.pop_front();
        if (created_) drain_images();
    }
    void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (fault_) return;
        if (msg->header.frame_id != camera_frame_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Image frame '%s' != '%s'; no VIO input", msg->header.frame_id.c_str(), camera_frame_.c_str());
            return;
        }
        const double t = rclcpp::Time(msg->header.stamp).seconds();
        if (!finite(t) || t <= 0 || t <= last_received_image_t_) return;
        last_received_image_t_ = t;
        last_image_wall_ = std::chrono::steady_clock::now();
        saw_image_ = true;
        images_.push_back(std::move(msg));
        while (images_.size() > 30) {
            images_.pop_front();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "VIO image queue overflow; dropping oldest frame");
        }
        if (!created_) attempt_init();
        if (created_) drain_images();
    }

    void attempt_init() {
        if (created_ || fault_ || !camera_info_) return;
        const auto &ci = *camera_info_;
        if (!ci.width || !ci.height || !finite(ci.p[0]) || !finite(ci.p[5]) ||
            ci.p[0] <= 0 || ci.p[5] <= 0 ||
            !finite(ci.p[2]) || !finite(ci.p[6]) ||
            std::abs(ci.p[3]) > 1e-9 || std::abs(ci.p[7]) > 1e-9) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                "Rectified CameraInfo has invalid pinhole P matrix");
            return;
        }
        geometry_msgs::msg::TransformStamped camera_tf, imu_tf;
        try {
            // lookupTransform(target, source): ^base T_camera and ^base T_imu.
            camera_tf = tf_buffer_->lookupTransform(base_frame_, camera_frame_, tf2::TimePointZero);
            imu_tf = tf_buffer_->lookupTransform(base_frame_, imu_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Waiting for static sensor TF: %s", e.what());
            return;
        }
        config_.width = int(ci.width);
        config_.height = int(ci.height);
        config_.fx = ci.p[0]; config_.fy = ci.p[5];
        config_.cx = ci.p[2]; config_.cy = ci.p[6];
        config_.base_from_camera = extrinsic_from_tf(camera_tf);
        config_.base_from_imu = extrinsic_from_tf(imu_tf);
        bool create_attempted = false;
        try {
            const auto yaml = xrslam_ros2::render_device_config(config_);
            // An ephemeral 0600 file only adapts the existing XRSLAM C API;
            // the authoritative values are CameraInfo and /tf_static.
            std::string templ = "/tmp/xrslam_ros2_device_XXXXXX";
            std::vector<char> filename(templ.begin(),templ.end());
            filename.push_back('\0');
            const int fd = ::mkstemp(filename.data());
            if (fd < 0) throw std::runtime_error("mkstemp failed");
            const std::string path(filename.data());
            std::size_t written = 0;
            bool ok = true;
            while (written < yaml.size()) {
                const ssize_t n = ::write(fd,yaml.data()+written,yaml.size()-written);
                if (n <= 0) {ok = false;break;}
                written += static_cast<std::size_t>(n);
            }
            if (::close(fd) != 0) ok = false;
            if (!ok) {::unlink(path.c_str());throw std::runtime_error("write temporary calibration failed");}
            void *opaque_config = nullptr;
            int result=0;
            try {
                create_attempted = true;
                result=XRSLAMCreate(slam_config_path_.c_str(),path.c_str(),"","XRSLAM ROS2 Leroy",&opaque_config);
            } catch (...) {
                ::unlink(path.c_str());
                throw;
            }
            ::unlink(path.c_str());
            if (result != 1 || !opaque_config)
                throw std::runtime_error("XRSLAMCreate failed");
            if (!XRSLAMRequiresIMU(opaque_config))
                throw std::runtime_error("slam_config must enable monocular_imu");
            created_ = true;
            state_ = "initializing";
            RCLCPP_INFO(get_logger(),
                "XRSLAM VIO started from TF+CameraInfo, resolution=%dx%d; gyro=%s, image=%s",
                config_.width,config_.height,imu_topic_.c_str(),image_topic_.c_str());
        } catch (const std::exception &e) {
            if (create_attempted) {
                // XRSLAMCreate may throw after constructing its worker threads.
                // Tear them down deterministically even on partial failure.
                try { XRSLAMDestroy(); } catch (...) {}
            }
            latch_fault(std::string("initialization failed: ")+e.what());
        }
    }

    void drain_images() {
        if (!created_ || fault_ || imus_.empty()) return;
        while (!images_.empty()) {
            const auto &image = images_.front();
            const double t = rclcpp::Time(image->header.stamp).seconds();
            if (t > last_received_imu_t_) return;  // await next IMU; never extrapolate to future
            if (last_sent_imu_t_ < 0 && rclcpp::Time(imus_.front()->header.stamp).seconds() > t) {
                images_.pop_front(); // no earlier IMU can ever arrive
                continue;
            }
            // Consume all IMU observations up to the image timestamp in time order.
            while (!imus_.empty() && rclcpp::Time(imus_.front()->header.stamp).seconds() <= t) {
                const auto sample=imus_.front();
                const double ti = rclcpp::Time(sample->header.stamp).seconds();
                imus_.pop_front();
                if (ti <= last_sent_imu_t_) continue;
                XRSLAMGyroscope gyro = {{sample->angular_velocity.x,
                    sample->angular_velocity.y,sample->angular_velocity.z},ti};
                XRSLAMAcceleration accel = {{sample->linear_acceleration.x,
                    sample->linear_acceleration.y,sample->linear_acceleration.z},ti};
                XRSLAMPushSensorData(XRSLAM_SENSOR_GYROSCOPE,&gyro);
                XRSLAMPushSensorData(XRSLAM_SENSOR_ACCELERATION,&accel);
                last_sent_imu_t_=ti;
            }
            if (last_sent_imu_t_ < 0) {images_.pop_front();continue;}
            process_image(*image);
            images_.pop_front();
            if (fault_) return;
        }
    }

    void process_image(const sensor_msgs::msg::Image &image) {
        if (int(image.width)!=config_.width || int(image.height)!=config_.height) {
            latch_fault("image resolution changed after XRSLAM initialization");
            return;
        }
        std::vector<std::uint8_t> converted;
        int channels = 0;
        std::size_t stride=0;
        if (!input_image(image,converted,channels,stride)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                "Expected mono8, rgb8, bgr8, rgba8, or bgra8 rectified image");
            return;
        }
        XRSLAMImage xi{};
        xi.camera_id=0; xi.channel=channels;
        xi.timeStamp=rclcpp::Time(image.header.stamp).seconds();
        xi.ext=nullptr; xi.data=converted.data(); xi.stride=static_cast<int>(stride);
        try {
            XRSLAMPushSensorData(XRSLAM_SENSOR_CAMERA,&xi);
            XRSLAMRunOneFrame();
            XRSLAMState state=XRSLAM_STATE_INITIALIZING;
            XRSLAMGetResult(XRSLAM_RESULT_STATE,&state);
            if (state != XRSLAM_STATE_TRACKING_SUCCESS) {
                if (ever_tracking_) latch_fault("tracking lost: odometry continuity cannot be assumed");
                else state_="initializing";
                return;
            }
            XRSLAMPose pose{};
            XRSLAMGetResult(XRSLAM_RESULT_BODY_POSE,&pose);
            PoseSample current;
            current.timestamp=pose.timestamp;
            current.position=tf2::Vector3(pose.translation[0],pose.translation[1],pose.translation[2]);
            current.orientation=tf2::Quaternion(pose.quaternion[0],pose.quaternion[1],
                                                pose.quaternion[2],pose.quaternion[3]);
            if (!finite(current.timestamp) || current.timestamp<=0 ||
                !finite(current.position.x()) || !finite(current.position.y()) ||
                !finite(current.position.z()) ||
                !finite(current.orientation.length2()) ||
                std::abs(current.orientation.length2()-1.0)>1e-2) {
                latch_fault("nonfinite or invalid XRSLAM body pose"); return;
            }
            current.orientation.normalize();
            if (previous_ && current.timestamp <= previous_->timestamp + 1e-7)
                return; // asynchronous backend has not advanced its output
            ever_tracking_=true;
            state_="tracking";
            if (previous_) {
                const double dt=current.timestamp-previous_->timestamp;
                if (dt<=0 || dt>stale_timeout_s_) {
                    latch_fault("odometry time gap: cannot ensure continuity");return;
                }
                const tf2::Vector3 delta=current.position-previous_->position;
                const double linear_speed=delta.length()/dt;
                tf2::Quaternion dq=previous_->orientation.inverse()*current.orientation;
                dq.normalize();
                if (dq.w()<0) dq=tf2::Quaternion(-dq.x(),-dq.y(),-dq.z(),-dq.w());
                const double angle=dq.getAngle();
                if (linear_speed>max_linear_speed_ || angle/dt>max_angular_speed_) {
                    latch_fault("implausible pose jump (possible VIO session reset)");return;
                }
                const auto stamp = rclcpp::Time(static_cast<int64_t>(std::llround(current.timestamp*1e9)));
                nav_msgs::msg::Odometry odom;
                odom.header.stamp=stamp; odom.header.frame_id=odom_frame_;
                odom.child_frame_id=base_frame_;
                odom.pose.pose.position.x=current.position.x();
                odom.pose.pose.position.y=current.position.y();
                odom.pose.pose.position.z=current.position.z();
                odom.pose.pose.orientation.x=current.orientation.x();
                odom.pose.pose.orientation.y=current.orientation.y();
                odom.pose.pose.orientation.z=current.orientation.z();
                odom.pose.pose.orientation.w=current.orientation.w();
                // Explicit provisional heuristics, NOT XRSLAM estimator covariances.
                odom.pose.covariance[0]=pose_stddev_xy_*pose_stddev_xy_;
                odom.pose.covariance[7]=odom.pose.covariance[0];
                odom.pose.covariance[14]=pose_stddev_z_*pose_stddev_z_;
                odom.pose.covariance[21]=odom.pose.covariance[28]=1e3;
                odom.pose.covariance[35]=pose_stddev_yaw_*pose_stddev_yaw_;
                for (int i=0;i<6;++i) odom.twist.covariance[i*6+i]=twist_stddev_*twist_stddev_;
                const tf2::Vector3 vbase=tf2::quatRotate(current.orientation.inverse(),delta)/dt;
                odom.twist.twist.linear.x=vbase.x();
                odom.twist.twist.linear.y=vbase.y();
                odom.twist.twist.linear.z=vbase.z();
                const tf2::Vector3 omega=dq.getAxis()*(angle/dt);
                odom.twist.twist.angular.x=omega.x();
                odom.twist.twist.angular.y=omega.y();
                odom.twist.twist.angular.z=omega.z();
                odom_pub_->publish(odom);
                geometry_msgs::msg::TransformStamped tf;
                tf.header=odom.header; tf.child_frame_id=base_frame_;
                tf.transform.translation.x=current.position.x();
                tf.transform.translation.y=current.position.y();
                tf.transform.translation.z=current.position.z();
                tf.transform.rotation=odom.pose.pose.orientation;
                tf_broadcaster_->sendTransform(tf);
                geometry_msgs::msg::PoseStamped p;
                p.header=odom.header; p.pose=odom.pose.pose;
                pose_pub_->publish(p);
            }
            previous_=current;
        } catch (const std::exception &e) {
            latch_fault(std::string("XRSLAM processing exception: ")+e.what());
        }
    }

    void latch_fault(const std::string &reason) {
        fault_=true;
        state_="fault";
        RCLCPP_ERROR(get_logger(),"XRSLAM odometry HALTED: %s. Stop navigation; restart after inspection.",reason.c_str());
    }
    void tick() {
        if (!fault_ && ever_tracking_ && saw_image_ &&
            std::chrono::duration<double>(std::chrono::steady_clock::now()-last_image_wall_).count()>stale_timeout_s_)
            latch_fault("camera stopped or stalled");
        if (!created_ && !fault_) attempt_init();
        diagnostic_msgs::msg::DiagnosticArray msg;
        msg.header.stamp=now();
        diagnostic_msgs::msg::DiagnosticStatus st;
        st.name="XRSLAM VIO";
        st.hardware_id="leroy/front_camera+imu";
        st.level = fault_ ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
            (state_=="tracking" ? diagnostic_msgs::msg::DiagnosticStatus::OK :
             diagnostic_msgs::msg::DiagnosticStatus::WARN);
        st.message=state_;
        msg.status.push_back(std::move(st));
        diag_pub_->publish(msg);
    }

    std::string image_topic_,imu_topic_,camera_info_topic_,odom_topic_;
    std::string base_frame_,odom_frame_,camera_frame_,imu_frame_,slam_config_path_;
    double max_linear_speed_=4,max_angular_speed_=8,stale_timeout_s_=0.7;
    double pixel_variance_=0.5,time_offset_=0.0;
    double pose_stddev_xy_=0.2,pose_stddev_z_=0.35,pose_stddev_yaw_=0.25,twist_stddev_=0.5;
    xrslam_ros2::SensorConfig config_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
    std::deque<sensor_msgs::msg::Image::ConstSharedPtr> images_;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imus_;
    double last_received_image_t_=-1,last_received_imu_t_=-1,last_sent_imu_t_=-1;
    std::optional<PoseSample> previous_;
    bool created_=false, ever_tracking_=false, saw_image_=false, fault_=false;
    std::chrono::steady_clock::time_point last_image_wall_{};
    std::string state_="waiting_for_calibration";
};

int main(int argc,char **argv) {
    rclcpp::init(argc,argv);
    try {
        auto node=std::make_shared<XRSLAMRos2Node>();
        rclcpp::spin(node);
    } catch(const std::exception &e) {
        std::fprintf(stderr,"[XRSLAM ROS2] FATAL: %s\n",e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
