#pragma once

#include <QApplication>
#include <QGraphicsLineItem>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QGraphicsTextItem>
#include <QGraphicsView>
#include <QLabel>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <unordered_map>

class QtSubscriberNode : public rclcpp::Node
{
public:
  QtSubscriberNode(QGraphicsScene *scene);

private:
  void floatCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void temperatureCallback(const sensor_msgs::msg::Temperature::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr float_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temperature_subscription_;

  QGraphicsScene *scene_;
  std::unordered_map<std::string, QGraphicsRectItem*> float_boxes_;
  std::unordered_map<std::string, QGraphicsRectItem*> temperature_boxes_;
  std::unordered_map<std::string, QGraphicsLineItem*> float_lines_;
  std::unordered_map<std::string, QGraphicsLineItem*> temperature_lines_;
  std::unordered_map<std::string, QGraphicsTextItem*> line_labels_;
  std::unordered_map<std::string, int> publisher_counts_;
  std::unordered_map<std::string, QGraphicsTextItem*> publisher_count_labels_;
  std::unordered_map<std::string, QGraphicsTextItem*> float_data_labels_;
  std::unordered_map<std::string, QGraphicsTextItem*> temperature_data_labels_;

  QGraphicsRectItem *subscriber_box_;

  void addFloatPublisher(const std::string &publisher_id, const QString &topic_name);
  void addTemperaturePublisher(const std::string &publisher_id, const QString &topic_name);
  void updatePublisherCount(const std::string &publisher_id);
};
