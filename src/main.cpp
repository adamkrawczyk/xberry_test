#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMainWindow>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include "xberry_test/qt_subscriber_node.h"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  QGraphicsScene scene;
  QGraphicsView view(&scene);
  QMainWindow main_window;

  auto node = std::make_shared<QtSubscriberNode>(&scene);

  view.setScene(&scene);
  main_window.setCentralWidget(&view);
  main_window.resize(800, 600);
  main_window.show();

  QTimer timer;
  QObject::connect(&timer, &QTimer::timeout, [&]() {
    rclcpp::spin_some(node);
  });
  timer.start(100);

  return app.exec();
}
