#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <atomic>
#include <chrono>
#include "sensor_fusion_mt/safe_queue.hpp"
#include <map>


using namespace std::chrono_literals;

struct ImuData
{
    double ax, ay, az; // Accelerometer data
    rclcpp::Time timestamp;
};

struct EncoderData
{
    int ticks;
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
        imu_thread_ = std::thread(&SensorFusionNode::imuLoop, this);
        encoder_thread_ = std::thread(&SensorFusionNode::encoderLoop, this);
        camera_thread_ = std::thread(&SensorFusionNode::cameraLoop, this);

        fusion_thread_ = std::thread(&SensorFusionNode::fusionLoop, this);

    }

    ~SensorFusionNode()
    {
        running_ = false;
        if (imu_thread_.joinable()) imu_thread_.join();
        if (encoder_thread_.joinable()) encoder_thread_.join();
        if (camera_thread_.joinable()) camera_thread_.join();
        if (fusion_thread_.joinable()) fusion_thread_.join();
        RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node has been stopped.");
    }

private:
    std::atomic<bool> running_;
    std::thread imu_thread_;
    std::thread encoder_thread_;
    std::thread camera_thread_;
    std::thread fusion_thread_;


    SafeQueue<ImuData> imu_queue_;
    SafeQueue<EncoderData> encoder_queue_;
    SafeQueue<CameraData> camera_queue_;

    std::map<int64_t, ImuData> imu_buffer_;
    std::map<int64_t, EncoderData> encoder_buffer_;
    std::map<int64_t, CameraData> camera_buffer_;

    std::mutex imu_mutex_;
    std::mutex encoder_mutex_;
    std::mutex camera_mutex_;



    void imuLoop()
    {
        rclcpp::Rate rate(100); // 100 Hz
        while (rclcpp::ok() && running_)
        {

            ImuData data;
            data.ax = 0.1; data.ay = 0.2; data.az = 9.81;
            data.timestamp = this->now();
            int64_t stamp_ns = data.timestamp.nanoseconds();
            imu_queue_.insert_and_cleanup(imu_buffer_, imu_mutex_, data, stamp_ns);
            rate.sleep();   
        }       
        
    }

    void encoderLoop()
    {
        rclcpp::Rate rate(50); // 50 Hz
        int tick = 0;
        while (rclcpp::ok() && running_)
        {
            EncoderData data;
            data.ticks = tick++;
            data.timestamp = this->now();
            int64_t stamp_ns = data.timestamp.nanoseconds();
            encoder_queue_.insert_and_cleanup(encoder_buffer_, encoder_mutex_, data, stamp_ns);
            rate.sleep();
        }    
        

    }


    void cameraLoop()
    {
        rclcpp::Rate rate(30); // 30 Hz
        while (rclcpp::ok() && running_)
        {
            CameraData data;
            data.timestamp = this->now();
            int64_t stamp_ns = data.timestamp.nanoseconds();
            camera_queue_.insert_and_cleanup(camera_buffer_, camera_mutex_, data, stamp_ns);
            rate.sleep();
         
        }    
        
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


                RCLCPP_INFO(this->get_logger(),
                "\n=== FUSED ===\n"
                "Camera: %ld\n"
                "IMU   : ax=%.2f ay=%.2f az=%.2f (dt=%.2f ms)\n"
                "Enc   : ticks=%d (dt=%.2f ms)\n",
                cam_ts,
                imu.ax, imu.ay, imu.az,
                (imu.timestamp.nanoseconds() - cam_ts) / 1e6,
                enc.ticks,
                (enc.timestamp.nanoseconds() - cam_ts) / 1e6
            );

            rate.sleep();
        }
    }



};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}