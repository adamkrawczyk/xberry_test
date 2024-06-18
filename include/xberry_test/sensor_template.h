#pragma once

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>

// Reconfiguration service for frequency
#include "xberry_test/srv/set_frequency.hpp"

template <typename T>
class TemplatedPublisher : public rclcpp::Node {
public:
  TemplatedPublisher(const std::string &topic_name, std::chrono::milliseconds publish_period)
      : Node("templated_publisher"), publish_period_(publish_period) {
    publisher_ = this->create_publisher<T>(topic_name, 10);
    timer_ = this->create_wall_timer(publish_period_, [this]() {
      auto message = create_message();
      RCLCPP_INFO(this->get_logger(), "Publishing a message");
      publisher_->publish(message);
    });

    // Initialize service to dynamically adjust publishing frequency
    frequency_service_ = this->create_service<xberry_test::srv::SetFrequency>(
        "set_publish_frequency",
        [this](const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<xberry_test::srv::SetFrequency::Request> request,
               std::shared_ptr<xberry_test::srv::SetFrequency::Response> response) {
          (void)request_header;
          this->change_publish_period(std::chrono::milliseconds(static_cast<int>(1000 / request->frequency)));
          response->success = true;
        });
  }

  virtual ~TemplatedPublisher() {}

protected:
  virtual T create_message() = 0;

  // Virtual function that derived classes can override to implement their specific data updates
  virtual void update_message_data(T data) = 0;

  // Change the publishing frequency
  void change_publish_period(std::chrono::milliseconds new_period) {
    if (timer_) {
      timer_->cancel(); // Stop the current timer
      timer_ = this->create_wall_timer(new_period, [this]() {
        auto message = create_message();
        RCLCPP_INFO(this->get_logger(), "Publishing a message");
        publisher_->publish(message);
      });
      publish_period_ = new_period; // Update the stored publish period
    }
  }

private:
  typename rclcpp::Publisher<T>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds publish_period_;
  rclcpp::Service<xberry_test::srv::SetFrequency>::SharedPtr frequency_service_;
};
