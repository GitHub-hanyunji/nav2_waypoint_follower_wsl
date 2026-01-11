#include <memory>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <tuple>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"


int getch(void)
{
  #if defined(__linux__) || defined(__APPLE__)

    struct termios oldt, newt;
    int ch;

    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 1;
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

    return ch;

  #elif defined(_WIN32) || defined(_WIN64)
    return _getch();
  #endif
}

int kbhit(void)
{
  #if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
      ungetc(ch, stdin);
      return 1;
    }
    return 0;
  #elif defined(_WIN32)
    return _kbhit();
  #endif
}

class WaypointClient : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  WaypointClient()
  : Node("lab_waypoint_client")
  {
    client_ = rclcpp_action::create_client<FollowWaypoints>(
      this,
      "follow_waypoints"
    );

    // timer_ = this->create_wall_timer(
    //   std::chrono::seconds(2),
    //   std::bind(&WaypointClient::send_goal, this)
    // );
    key_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&WaypointClient::keyLoop, this)
    );
  }

private:
  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr key_timer_;
  bool goal_sent_ = false;
  bool infinite_mode_ = false;
  bool stop_requested_ = false;
  rclcpp::TimerBase::SharedPtr restart_timer_;


  void send_goal()
  {
    if (goal_sent_) return;

    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "FollowWaypoints action server not available");
      return;
    }

    FollowWaypoints::Goal goal_msg;

    // ===== 연구실 내부 순환 경유점 (예시 좌표) =====
    std::vector<std::tuple<double, double, double>> waypoints = {
      {-2.0,  0.0,  -3.13},
      {-3.1, -0.55, -2.03},
      {-3.25, -1.44, -1.51},
      {-3.03, -2.64, -0.41},
      {-1.71, -3.02,  0.20},
      {-0.7,  -2.81,  0.28},
      {0.41, -2.23,  1.17},
      { 0.0,  0.0,   3.14}

    };

    for (auto & wp : waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = this->now();

      pose.pose.position.x = std::get<0>(wp);
      pose.pose.position.y = std::get<1>(wp);

      tf2::Quaternion q;
      q.setRPY(0, 0, std::get<2>(wp));
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();

      goal_msg.poses.push_back(pose);
    }

    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&WaypointClient::result_callback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, send_goal_options);
    goal_sent_ = true;

    RCLCPP_INFO(this->get_logger(), "Waypoint goal sent");
  }
  void keyLoop()
  {
    if (!kbhit()) return;
    int key = getch();

    if (key == '1') {
      RCLCPP_INFO(this->get_logger(), "Run ONE lap");
      infinite_mode_ = false;
      stop_requested_ = false;
      goal_sent_ = false;
      send_goal();
    }
    else if (key == 'i') {
      RCLCPP_INFO(this->get_logger(), "Run INFINITE laps");
      infinite_mode_ = true;
      stop_requested_ = false;
      goal_sent_ = false;
      send_goal();
    }
    else if (key == 's') {
      RCLCPP_INFO(this->get_logger(), "Stop after current lap");
      stop_requested_ = true;
        
      // 이미 예약된 다음 바퀴 타이머 취소
      if (restart_timer_) {
        restart_timer_->cancel();
        restart_timer_.reset();
      }
    }
    
    else if (key == 'q') {
      RCLCPP_INFO(this->get_logger(), "Quit");
      rclcpp::shutdown();
    }
  }


  void result_callback(const GoalHandleFollowWaypoints::WrappedResult & result)
  {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(this->get_logger(), "Waypoint failed or canceled");
      infinite_mode_ = false;
      stop_requested_ = false;
      goal_sent_ = false;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Waypoint lap completed");

    // 다음 바퀴 멈춤 요청이 있으면 종료
    if (stop_requested_) {
      RCLCPP_INFO(this->get_logger(), "Stopping. No more laps.");
      infinite_mode_ = false;
      stop_requested_ = false;
      goal_sent_ = false;
      return;
    }

    // 무한 모드면 다시 시작
    if (infinite_mode_) {
      RCLCPP_INFO(this->get_logger(), "Starting next lap...");
      goal_sent_ = false;

      restart_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        [this]() {
          send_goal();
        }
      );
    }
    else {
      // 한 바퀴 모드 → 그냥 종료
      goal_sent_ = false;
    }
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
