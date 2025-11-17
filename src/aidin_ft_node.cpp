#include <chrono>
#include <memory>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

extern "C" {
#include <soem/soem.h>
}

using namespace std::chrono_literals;

class AidinFTNode : public rclcpp::Node
{
public:
    AidinFTNode()
        : Node("ethercat_wrench_node"),
          running_(true)
    {
        this->declare_parameter<std::string>("ifname", "enxd0c0bf2f676d");
	    this->declare_parameter<int>("update_rate", 100);

		ifname_ = this->get_parameter("ifname").as_string();
    	update_rate_ = this->get_parameter("update_rate").as_int();

        // Create ROS2 publisher
        pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench", 10);

        // Start EtherCAT thread
        worker_ = std::thread(&AidinFTNode::run_ethercat, this);
    }

    ~AidinFTNode()
    {
        running_ = false;
        if(worker_.joinable()) worker_.join();
        ecx_close(&ctx_);
    }

private:

    struct Measurement
    {
        int Fx, Fy, Fz;
        int Tx, Ty, Tz;
    };

    Measurement read_measurement()
    {
        Measurement m{0, 0, 0, 0, 0, 0};

        int wkc;

        *(ctx_.slavelist[0].outputs + 0) = 1;   // bias ON

        ecx_send_processdata(&ctx_);
        wkc = ecx_receive_processdata(&ctx_, EC_TIMEOUTRET);

        if(wkc >= expectedWKC_)
        {
            auto in = ctx_.slavelist[0].inputs;

            m.Fx = (*(uint16_t*)(in + 0));
            m.Fy = (*(uint16_t*)(in + 4));
            m.Fz = (*(uint16_t*)(in + 8));

            m.Tx = (*(uint16_t*)(in + 12));
            m.Ty = (*(uint16_t*)(in + 16));
            m.Tz = (*(uint16_t*)(in + 20));
        }

        return m;
    }

    void run_ethercat()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing SOEM on %s", ifname_.c_str());

        memset(&ctx_, 0, sizeof(ecx_contextt));
        if(!ecx_init(&ctx_, ifname_.c_str()))
        {
            RCLCPP_ERROR(this->get_logger(), "SOEM init on %s failed", ifname_.c_str());
            return;
        }

        if(ecx_config_init(&ctx_) <= 0)
        {
            RCLCPP_ERROR(this->get_logger(), "No EtherCAT slaves found");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "%d slaves found", ctx_.slavecount);

        uint8 IOmap[4096];
        ecx_config_map_group(&ctx_, &IOmap, 0);
        ecx_configdc(&ctx_);

        ecx_statecheck(&ctx_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTRET);
        ctx_.slavelist[0].state = EC_STATE_OPERATIONAL;
        ecx_writestate(&ctx_, 0);

        int chk = 40;
        do {
            ecx_send_processdata(&ctx_);
            ecx_receive_processdata(&ctx_, EC_TIMEOUTRET);
            ecx_statecheck(&ctx_, 0, EC_STATE_OPERATIONAL, 50000);
        } while(chk-- && ctx_.slavelist[0].state != EC_STATE_OPERATIONAL);

        if(ctx_.slavelist[0].state != EC_STATE_OPERATIONAL)
        {
            RCLCPP_ERROR(this->get_logger(), "Did not reach OP state");
            return;
        }

        expectedWKC_ = (ctx_.grouplist[0].outputsWKC * 2) + ctx_.grouplist[0].inputsWKC;
        RCLCPP_INFO(this->get_logger(), "Operational. Expected WKC = %d", expectedWKC_);

        // 100 Hz loop
        rclcpp::Rate rate(update_rate_);

        while(running_)
        {
            Measurement m = read_measurement();

            geometry_msgs::msg::WrenchStamped msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "sensor_frame";

            msg.wrench.force.x = (static_cast<double>(m.Fx)/100) - 300;
            msg.wrench.force.y = (static_cast<double>(m.Fy)/100) - 300;
            msg.wrench.force.z = (static_cast<double>(m.Fz)/100) - 300;

            msg.wrench.torque.x = (static_cast<double>(m.Tx)/500) - 50;
            msg.wrench.torque.y = (static_cast<double>(m.Ty)/500) - 50;
            msg.wrench.torque.z = (static_cast<double>(m.Tz)/500) - 50;

            pub_->publish(msg);
            rate.sleep();
        }
    }

    std::string ifname_;
	int update_rate_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
    std::thread worker_;
    std::atomic<bool> running_;
    ecx_contextt ctx_{};
    int expectedWKC_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AidinFTNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}