#include "std_msgs/msg/float32.hpp"
#include "xberry_test/helpers.h"
#include "xberry_test/sensor_template.h"
#include "xberry_test/srv/set_float.hpp"

class FloatPublisher : public TemplatedPublisher<std_msgs::msg::Float32> {
public:
  FloatPublisher(const std::string &topic_name, std::chrono::milliseconds publish_period)
      : TemplatedPublisher(topic_name, publish_period), current_message_() {

    current_message_.data = 1.0; // Default value

    // Service to change the float data
    data_service_ = this->create_service<xberry_test::srv::SetFloat>(
        "set_float_data",
        [this](const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<xberry_test::srv::SetFloat::Request> request,
               std::shared_ptr<xberry_test::srv::SetFloat::Response> response) {
          (void)request_header;
          std_msgs::msg::Float32 new_data;
          new_data.data = request->data;
          this->update_message_data(new_data);
          response->success = true;
    });
  }

protected:
  std_msgs::msg::Float32 create_message() override {
    return current_message_;
  }

  void update_message_data(std_msgs::msg::Float32 data) override {
    current_message_ = data; // Update the current message with new data
  }

private:
  std_msgs::msg::Float32 current_message_;
  rclcpp::Service<xberry_test::srv::SetFloat>::SharedPtr data_service_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  int timer_ms = randomTimer();
  auto frequency = 1.0 / timer_ms * 1000;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing frequency: %f Hz", frequency);

  auto node = std::make_shared<FloatPublisher>("float_topic", std::chrono::milliseconds(timer_ms));

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
