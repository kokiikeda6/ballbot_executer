#include <rclcpp/rclcpp.hpp>
#include <linear_motor_msgs/srv/act.hpp>
#include <linear_motor_msgs/srv/mode.hpp>

bool check_future_complete(auto& node, auto& future)
{
  // 応答を待機
  if (rclcpp::spin_until_future_complete(node, future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    RCLCPP_INFO(node->get_logger(), "Response: success = %s", response->result.c_str());
    return true;
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "サービス呼び出しに失敗しました");
    return false;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ballbot_client_node");

  auto linear_client = node->create_client<linear_motor_msgs::srv::Act>("/action_command");
  auto tracker_client = node->create_client<linear_motor_msgs::srv::Act>("/tracker_mode");
  auto linear_request = std::make_shared<linear_motor_msgs::srv::Act::Request>();
  auto tracker_request = std::make_shared<linear_motor_msgs::srv::Mode::Request>();


  // 1. オムニホイール格納

  // サーバーが起動するまで待つ
  if (!linear_client->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(node->get_logger(), "リニアアクチュエータのサービスが見つかりません");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "オムニホイールを格納します");
  linear_request->action = "up";
  auto linear_future = linear_client->async_send_request(linear_request);

  // 2. 10秒待機 → オムニホイール展開
  if(check_future_complete(node, linear_future))
  {
  sleep(10);

  // サーバーが起動するまで待つ
  if (!linear_client->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(node->get_logger(), "リニアアクチュエータのサービスが見つかりません");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "オムニホイールを展開します");
  linear_request->action = "down";
  auto linear_future = linear_client->async_send_request(linear_request);

  }
  // 3. ピン認識起動
  else if(check_future_complete(node, linear_future))
  {
  // サーバーが起動するまで待つ
  if (!tracker_client->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(node->get_logger(), "ピン認識のサービスが見つかりません");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "ピンの認識を開始します");
  tracker_request->mode = "tracker_START";
  auto tracker_future = tracker_client->async_send_request(linear_request);
  }

  rclcpp::shutdown();
  return 0;
}