#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>

#include <thread>
#include <atomic>
#include <chrono>
#include "sensor_fusion_mt/safe_queue.hpp"
#include <map>
#include <mutex>
#include <limits>
#include <cmath>


using namespace std::chrono_literals;

struct ImuData
{
    double ax, ay, az; // Accelerometer data
    rclcpp::Time timestamp;
};

struct EncoderData
{
    int32_t ticks=0;
    rclcpp::Time timestamp;
};

struct CameraData
{
    rclcpp::Time timestamp;
};



class SensorFusionNode : public rclcpp::Node
{
public:
    SensorFusionNode() : Node("sensor_fusion_node"), running_(true)
    {

        RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node has been started....");

        imu_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        encoder_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        camera_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions imu_sub_options;
        imu_sub_options.callback_group = imu_cb_group_;
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&SensorFusionNode::imuCallback, this, std::placeholders::_1),
            imu_sub_options
        );
        
        rclcpp::SubscriptionOptions encoder_sub_options;
        encoder_sub_options.callback_group = encoder_cb_group_;
        encoder_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/encoder/ticks",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&SensorFusionNode::encoderCallback, this, std::placeholders::_1),
            encoder_sub_options
        );

        rclcpp::SubscriptionOptions camera_sub_options;
        camera_sub_options.callback_group = camera_cb_group_;
        camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&SensorFusionNode::cameraCallback, this, std::placeholders::_1),
            camera_sub_options
        );


        // Start fusion thread
        fusion_thread_ = std::thread(&SensorFusionNode::fusionLoop, this);


        // imu_thread_ = std::thread(&SensorFusionNode::imuLoop, this);
        // encoder_thread_ = std::thread(&SensorFusionNode::encoderLoop, this);
        // camera_thread_ = std::thread(&SensorFusionNode::cameraLoop, this);

        // fusion_thread_ = std::thread(&SensorFusionNode::fusionLoop, this);

    }

    ~SensorFusionNode()
    {
        running_ = false;
        // if (imu_thread_.joinable()) imu_thread_.join();
        // if (encoder_thread_.joinable()) encoder_thread_.join();
        // if (camera_thread_.joinable()) camera_thread_.join();
        if (fusion_thread_.joinable()) fusion_thread_.join();
        RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node has been stopped.");
    }


    std::vector<rclcpp::CallbackGroup::SharedPtr> getCallbackGroups() {
        return {imu_cb_group_, encoder_cb_group_, camera_cb_group_};
    }

private:

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encoder_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;

    rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
    rclcpp::CallbackGroup::SharedPtr encoder_cb_group_;
    rclcpp::CallbackGroup::SharedPtr camera_cb_group_;



    std::atomic<bool> running_;
    // std::thread imu_thread_;
    // std::thread encoder_thread_;
    // std::thread camera_thread_;
    std::thread fusion_thread_;


    SafeQueue<ImuData> imu_queue_;
    SafeQueue<EncoderData> encoder_queue_;
    SafeQueue<CameraData> camera_queue_;

    std::map<int64_t, ImuData> imu_buffer_;
    std::map<int64_t, EncoderData> encoder_buffer_;
    std::map<int64_t, CameraData> camera_buffer_;

    mutable std::mutex imu_mutex_;
    mutable std::mutex encoder_mutex_;
    mutable std::mutex camera_mutex_;



    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        ImuData data;
        data.ax = msg->linear_acceleration.x;
        data.ay = msg->linear_acceleration.y;
        data.az = msg->linear_acceleration.z;
        data.timestamp = msg->header.stamp;
        int64_t stamp_ns = data.timestamp.nanoseconds();    
        imu_queue_.insert_and_cleanup(imu_buffer_, imu_mutex_, data, stamp_ns); 
        
    }

    void encoderCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        EncoderData data;  
        data.ticks = msg->data;
        data.timestamp = this->now();
        int64_t stamp_ns = data.timestamp.nanoseconds();
        encoder_queue_.insert_and_cleanup(encoder_buffer_, encoder_mutex_, data, stamp_ns);

        

    }


    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        CameraData data;  
        data.timestamp = msg->header.stamp;
        int64_t stamp_ns = data.timestamp.nanoseconds();
        camera_queue_.insert_and_cleanup(camera_buffer_, camera_mutex_, data, stamp_ns); 
        
    }



    void fusionLoop()
    {
        const int64_t MAX_DIFF_NS = 10 * 1000 * 1000; // 10 ms
        rclcpp::Rate rate(60); // 60 Hz
        while(rclcpp::ok() && running_)
        {
            CameraData cam;
            {
                std::lock_guard<std::mutex> lock(camera_mutex_);
                if (camera_buffer_.empty()) {
                rate.sleep();
                continue;
                }
                cam = camera_buffer_.rbegin()->second;
            }

            int64_t cam_ts = cam.timestamp.nanoseconds();

            ImuData imu;
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                if (!imu_queue_.find_nearest(imu_buffer_, cam_ts, imu, MAX_DIFF_NS)) {
                    rate.sleep();
                    continue;
                }
            }

            EncoderData enc;
            {
                std::lock_guard<std::mutex> lock(encoder_mutex_);
                if (!encoder_queue_.find_nearest(encoder_buffer_, cam_ts, enc, MAX_DIFF_NS)) {
                    rate.sleep();
                    continue;
                }
            }

            // at this point, we have cam, imu, and enc data synchronized
            double imu_dt_ms = double(imu.timestamp.nanoseconds() - cam_ts) / 1e6;
            double enc_dt_ms = double(enc.timestamp.nanoseconds() - cam_ts) / 1e6;


            RCLCPP_INFO(this->get_logger(),
                "FUSED: cam_ts=%ld imu_dt=%.3fms enc_dt=%.3fms imu(ax,ay,az)=%.3f,%.3f,%.3f enc_ticks=%d",
                cam_ts,
                imu_dt_ms,
                enc_dt_ms,
                imu.ax, imu.ay, imu.az,
                enc.ticks
            );

            rate.sleep();
        }
    }



};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorFusionNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    for (const auto& cb_group : node->getCallbackGroups()) {
        executor.add_callback_group(cb_group, node->get_node_base_interface());
    }
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}