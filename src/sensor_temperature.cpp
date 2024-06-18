#include "sensor_msgs/msg/temperature.hpp"
#include "xberry_test/helpers.h"
#include "xberry_test/sensor_template.h"
#include "xberry_test/srv/set_temperature.hpp"

class TemperaturePublisher
    : public TemplatedPublisher<sensor_msgs::msg::Temperature> {
public:
  TemperaturePublisher(const std::string &topic_name,
                       std::chrono::milliseconds publish_period)
      : TemplatedPublisher(topic_name, publish_period), current_message_() {

    current_message_.temperature = 25.0;
    current_message_.variance = 0.1;

    // Service to change the temperature data
    data_service_ = this->create_service<xberry_test::srv::SetTemperature>(
        "set_temperature_data",
        [this](const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<xberry_test::srv::SetTemperature::Request> request,
               std::shared_ptr<xberry_test::srv::SetTemperature::Response> response) {
          (void)request_header;
          sensor_msgs::msg::Temperature temp_msg;
          temp_msg.temperature = request->temperature;
          temp_msg.variance = request->variance;
          this->update_message_data(temp_msg);
          response->success = true;
        });
  }

protected:
  sensor_msgs::msg::Temperature create_message() override {
    current_message_.header.stamp = this->now();
    return current_message_;
  }

  void update_message_data(sensor_msgs::msg::Temperature data) override {
    current_message_.temperature = data.temperature;
    current_message_.variance = data.variance;
  }

private:
  sensor_msgs::msg::Temperature current_message_;
  rclcpp::Service<xberry_test::srv::SetTemperature>::SharedPtr data_service_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  int timer_ms = randomTimer();
  auto frequency = 1.0 / timer_ms * 1000;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing frequency: %f Hz", frequency);

  auto node = std::make_shared<TemperaturePublisher>("temperature_topic", std::chrono::milliseconds(timer_ms));

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
