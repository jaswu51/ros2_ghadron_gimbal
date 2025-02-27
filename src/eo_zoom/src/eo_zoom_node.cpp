#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include "payloadSdkInterface.h"

#if (CONTROL_METHOD == CONTROL_UART)
T_ConnInfo s_conn = {
    CONTROL_UART,
    payload_uart_port,
    payload_uart_baud
};
#else
T_ConnInfo s_conn = {
    CONTROL_UDP,
    udp_ip_target,
    udp_port_target
};
#endif

class EoZoomNode : public rclcpp::Node {
private:
    PayloadSdkInterface* my_payload;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr zoom_range_sub_;

public:
    EoZoomNode() : Node("eo_zoom_node") {
        RCLCPP_INFO(this->get_logger(), "启动 EO 变焦节点...");
        
        // 创建 PayloadSDK 接口
        my_payload = new PayloadSdkInterface(s_conn);
        
        // 初始化连接
        my_payload->sdkInitConnection();
        RCLCPP_INFO(this->get_logger(), "等待负载设备连接...");
        my_payload->checkPayloadConnection();

        // 设置为可见光视图
        RCLCPP_INFO(this->get_logger(), "设置为可见光视图");
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIEW_SRC, PAYLOAD_CAMERA_VIEW_EO, PARAM_TYPE_UINT32);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // 设置初始变焦级别
        #if defined GHADRON
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_FACTOR, ZOOM_EO_1X, PARAM_TYPE_UINT32);
        #elif defined VIO || defined ZIO
        my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_SUPER_RESOLUTION_FACTOR, ZOOM_SUPER_RESOLUTION_1X, PARAM_TYPE_UINT32);
        #endif

        // 创建范围变焦订阅者
        zoom_range_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "eo_zoom/range", 10,
            std::bind(&EoZoomNode::zoomRangeCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "EO 变焦节点已准备就绪");
    }

    ~EoZoomNode() {
        if (my_payload != nullptr) {
            try {
                my_payload->sdkQuit();
                delete my_payload;
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "关闭设备时发生错误");
            }
        }
    }

private:
    void zoomRangeCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        float range = msg->data;
        if (range >= 0.0 && range <= 100.0) {
            RCLCPP_INFO(this->get_logger(), "设置变焦范围: %.1f%%", range);
            my_payload->setCameraZoom(ZOOM_TYPE_RANGE, range);
        } else {
            RCLCPP_WARN(this->get_logger(), "变焦范围无效: %.1f (应在0-100之间)", range);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EoZoomNode>());
    rclcpp::shutdown();
    return 0;
}