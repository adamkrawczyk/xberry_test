#include "xberry_test/qt_subscriber_node.h"
#include <QTimer>
#include <QPen>
#include <QBrush>

QtSubscriberNode::QtSubscriberNode(QGraphicsScene *scene)
    : Node("qt_subscriber_node"), scene_(scene)
{
  subscriber_box_ = scene_->addRect(0, 0, 100, 50, QPen(Qt::black), QBrush(Qt::lightGray));
  subscriber_box_->setPos(300, 200);
  auto subscriber_label = scene_->addText("QtSubscriberNode");
  subscriber_label->setPos(300, 200);

  float_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "float_topic", 10,
      std::bind(&QtSubscriberNode::floatCallback, this, std::placeholders::_1));

  temperature_subscription_ = this->create_subscription<sensor_msgs::msg::Temperature>(
      "temperature_topic", 10,
      std::bind(&QtSubscriberNode::temperatureCallback, this, std::placeholders::_1));
}

void QtSubscriberNode::addFloatPublisher(const std::string &publisher_id, const QString &topic_name)
{
  if (float_boxes_.find(publisher_id) == float_boxes_.end()) {
    auto box = scene_->addRect(0, 0, 100, 50, QPen(Qt::black), QBrush(Qt::gray));
    box->setPos(50, 50 + 100 * float_boxes_.size());
    float_boxes_[publisher_id] = box;

    auto line = scene_->addLine(
        box->pos().x() + box->rect().width() / 2,
        box->pos().y() + box->rect().height() / 2,
        subscriber_box_->pos().x() + subscriber_box_->rect().width() / 2,
        subscriber_box_->pos().y() + subscriber_box_->rect().height() / 2,
        QPen(Qt::black));
    float_lines_[publisher_id] = line;

    auto label = scene_->addText(topic_name);
    label->setPos((box->pos().x() + subscriber_box_->pos().x()) / 2, (box->pos().y() + subscriber_box_->pos().y()) / 2);
    line_labels_[publisher_id] = label;

    auto count_label = scene_->addText(QString::number(1));
    count_label->setPos(box->pos().x(), box->pos().y() - 20);
    publisher_count_labels_[publisher_id] = count_label;
    publisher_counts_[publisher_id] = 1;

    auto data_label = scene_->addText("");
    data_label->setPos(box->pos().x(), box->pos().y() + box->rect().height() + 10);
    float_data_labels_[publisher_id] = data_label;
  } else {
    updatePublisherCount(publisher_id);
  }
}

void QtSubscriberNode::addTemperaturePublisher(const std::string &publisher_id, const QString &topic_name)
{
  if (temperature_boxes_.find(publisher_id) == temperature_boxes_.end()) {
    auto box = scene_->addRect(0, 0, 100, 50, QPen(Qt::black), QBrush(Qt::gray));
    box->setPos(50, 150 + 100 * temperature_boxes_.size());
    temperature_boxes_[publisher_id] = box;

    auto line = scene_->addLine(
        box->pos().x() + box->rect().width() / 2,
        box->pos().y() + box->rect().height() / 2,
        subscriber_box_->pos().x() + subscriber_box_->rect().width() / 2,
        subscriber_box_->pos().y() + subscriber_box_->rect().height() / 2,
        QPen(Qt::black));
    temperature_lines_[publisher_id] = line;

    auto label = scene_->addText(topic_name);
    label->setPos((box->pos().x() + subscriber_box_->pos().x()) / 2, (box->pos().y() + subscriber_box_->pos().y()) / 2);
    line_labels_[publisher_id] = label;

    auto count_label = scene_->addText(QString::number(1));
    count_label->setPos(box->pos().x(), box->pos().y() - 20);
    publisher_count_labels_[publisher_id] = count_label;
    publisher_counts_[publisher_id] = 1;

    auto data_label = scene_->addText("");
    data_label->setPos(box->pos().x(), box->pos().y() + box->rect().height() + 10);
    temperature_data_labels_[publisher_id] = data_label;
  } else {
    updatePublisherCount(publisher_id);
  }
}

void QtSubscriberNode::updatePublisherCount(const std::string &publisher_id)
{
  publisher_counts_[publisher_id]++;
  publisher_count_labels_[publisher_id]->setPlainText(QString::number(publisher_counts_[publisher_id]));
}

void QtSubscriberNode::floatCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::string publisher_id = std::string(this->get_namespace()) + "/" + this->get_name() + "_float";
  addFloatPublisher(publisher_id, "float_topic");

  QPen pen(Qt::green);
  float_lines_[publisher_id]->setPen(pen);
  QTimer::singleShot(200, [this, publisher_id]() { float_lines_[publisher_id]->setPen(QPen(Qt::black)); });

  // Update the data label
  float_data_labels_[publisher_id]->setPlainText("Data: " + QString::number(msg->data));
}

void QtSubscriberNode::temperatureCallback(const sensor_msgs::msg::Temperature::SharedPtr msg)
{
  std::string publisher_id = std::string(this->get_namespace()) + "/" + this->get_name() + "_temperature";
  addTemperaturePublisher(publisher_id, "temperature_topic");

  QPen pen(Qt::green);
  temperature_lines_[publisher_id]->setPen(pen);
  QTimer::singleShot(200, [this, publisher_id]() { temperature_lines_[publisher_id]->setPen(QPen(Qt::black)); });

  // Update the data label
  temperature_data_labels_[publisher_id]->setPlainText("Temp: " + QString::number(msg->temperature) + "\nVar: " + QString::number(msg->variance));
}
