#include <memory>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <tuple>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "rclcpp/rclcpp.hpp"  
#include "rclcpp_action/rclcpp_action.hpp"      // Action Client 사용

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"


// 키입력
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

// 키 눌렸는지
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

// WaypointClient 클래스
// Nav2의 FollowWaypoints Action 서버에 goal을 보내서
// 여러 waypoint를 따라 로봇을 주행시키는 Action Client 노드
class WaypointClient : public rclcpp::Node
{
public:
  // using
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  WaypointClient()
  : Node("lab_waypoint_client")
  {
    // Nav2 BT Navigator가 제공하는 follow_waypoints Action 서버에 연결
    client_ = rclcpp_action::create_client<FollowWaypoints>(this,"follow_waypoints");

    key_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 0.1초마다 키보드 입력 체크
      std::bind(&WaypointClient::keyLoop, this)
    );
  }

private:
  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_;                   //FollowWaypoints Action Client
  rclcpp_action::ClientGoalHandle<FollowWaypoints>::SharedPtr goal_handle_;    // 현재 활성화 된 goal의 핸들

  rclcpp::TimerBase::SharedPtr key_timer_;        // 키 입력 감지를 위한 타이머
  size_t current_wp_index_ = 0;       // 전체 경로 기준으로 현재 성공 처리된 waypoint의 누적 인덱스
  size_t success_wp = 0;    // 현재 goal안에서 성공한 waypoint 개수

  
  bool goal_sent_ = false;       // 현재 waypoint goal이 이미 전송되었는지 여부
  bool infinite_mode_ = false;   // 무한 반복 주행 여부 (i 키)
  bool stop_requested_ = false;  // 현재 바퀴 끝나면 멈출지 여부 (s 키)
  bool paused_ = false;           // 일시정지 상태(q키)
  rclcpp::TimerBase::SharedPtr restart_timer_;  // 무한 주행 시 다음 바퀴 재시작 타이머

  // ===== 연구실 내부 순환 경유점 (x,y,yaw) =====
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

  // FollowWaypoints Action goal을 생성하고 전송하는 함수
  // current_wp: 전체 waypoint 중 어디서부터 다시 시작할지 결정
  void send_goal(size_t current_wp=0)
  {
    if (goal_sent_) return;  // 중복 전송 방지

    // Action 서버 대기
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "FollowWaypoints action server not available");
      return;
    }

    FollowWaypoints::Goal goal_msg;   // Goal 메시지 생성


    // current_wp부터 마지막 waypoint까지를 PoseStamped로 변환해서 goal에 추가
    for (size_t i=current_wp; i < waypoints.size(); i++) {
      auto & wp = waypoints[i];
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";     // Nav2는 반드시 "map" frame 사용
      pose.header.stamp = this->now();

      pose.pose.position.x = std::get<0>(wp);
      pose.pose.position.y = std::get<1>(wp);

      tf2::Quaternion q;
      q.setRPY(0, 0, std::get<2>(wp));   // yaw -> quaternion 
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();

      goal_msg.poses.push_back(pose);   // FollowWaypoints goal에 waypoint 추가
    }
    
    // SendGoalOptions
    // struct SendGoalOptions 정의
    // {
    //   GoalResponseCallback goal_response_callback;   // 서버가 goal을 수락/거절했는지 알려줌
    //   FeedbackCallback feedback_callback;            // waypoint 진행 중 중간 상태 -> 성공적으로 도달한 waypoint 의 개수
    //   ResultCallback result_callback;                // waypoint 주행 끝났을때 결과 보냄 -> 한 바퀴 끝냈는지 알기 위해 필요
    // };
    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

    // waypoint 진행 중 feedback 콜백(feedback->current_waypoint :"현재 goal 안에서 성공한 waypoint 개수")
    send_goal_options.goal_response_callback =  [this](GoalHandleFollowWaypoints::SharedPtr goal_handle) {goal_handle_ = goal_handle;  };
    // goal 전체가 끝났을 때 호출되는 결과 콜백 -> 로봇이 다시 원점으로 돌아왔을때 호출
    send_goal_options.feedback_callback =[this](auto, const auto & feedback) {  success_wp = feedback->current_waypoint;};

    // 주행 완료 / 실패 결과 처리용
    send_goal_options.result_callback =
      std::bind(&WaypointClient::result_callback, this, std::placeholders::_1);

    // async_send_goal 움직임 시작 
    /*WaypointClient (Action Client)
        ↓
    Nav2 FollowWaypoints Action Server
            ↓
    BT Navigator 실행
            ↓
    Planner → Controller
            ↓
    /cmd_vel 발행*/
    client_->async_send_goal(goal_msg, send_goal_options);  // Action 비동기 요청  -> 로봇이 실제로 움직이기 시작함
    goal_sent_ = true;    


    RCLCPP_INFO(this->get_logger(), "Waypoint goal sent");
  }

  // 키 값 제어 함수
  void keyLoop()
  {
    // 키 값 없으면 바로 return
    if (!kbhit()) return;
    int key = getch();
    
    // 한 바퀴 돌기
    if (key == '1') {
      RCLCPP_INFO(this->get_logger(), "Run ONE lap");
      infinite_mode_ = false;
      stop_requested_ = false;
      goal_sent_ = false;
      send_goal();
    }
    // 무제한 돌기
    else if (key == 'i') {
      RCLCPP_INFO(this->get_logger(), "Run INFINITE laps");
      infinite_mode_ = true;
      stop_requested_ = false;
      goal_sent_ = false;
      send_goal();
    }
    // 현재바퀴까지만 돌고 다음바퀴부터 멈춤
    else if (key == 's') {
      RCLCPP_INFO(this->get_logger(), "Stop after current lap");
      stop_requested_ = true;

      // 이미 예약된 다음 바퀴 타이머 취소
      if (restart_timer_) {
        restart_timer_->cancel();
        restart_timer_.reset();
      }
    }
    
    // 현재 주행 중인 로봇 멈추기
    else if (key == 'q') {
      // 현재 주행중인 waypoint 인덱스 저장
      // 루프를 시작할때마다 성공한 waypoint 개수를 current_wp에 저장
      // 즉 current_wp: 전체 경로 기준에서 이미 지나간 waypoint의 총 개수 (= 시작해야할 waypoint)
      current_wp_index_+=success_wp;   
      RCLCPP_INFO(this->get_logger(),"Cancel at waypoint %zu", current_wp_index_);
      
      // 현재 실행 중인 FollowWaypoints action goal 취소 -> Nav2 BT Navigator가 즉시 경로 추종을 중단
      if (goal_handle_) {
        client_->async_cancel_goal(goal_handle_);
      }
      paused_ = true;     // result_callback에서 자동 재시작(infinite loop) 방지
      goal_sent_ = false;

      // 이미 예약되어 있는 "다음 바퀴 시작 타이머" 제거
      if (restart_timer_) {
        restart_timer_->cancel();
        restart_timer_.reset();
      }
    }

    // 일시정지(paused) 상태에서만 주행 재개
    else if (key == 'r'&&paused_) {
      // 다시 시작하려는 waypoint 인덱스가 전체 waypoint 개수를 초고하면 return
      if (current_wp_index_ >= waypoints.size()) {
        RCLCPP_WARN(this->get_logger(), "No more waypoints to resume");
        return;
      }
      
      paused_ = false;   // pause 상태 해제
      goal_sent_ = false;
      send_goal(current_wp_index_); // 중단되었던 지점부터 주행 재개
    }



  }


  // 루프가 끝났을때 즉 한바퀴 돌고 로봇이 다시 원점으로 돌아왔을때 호출
  void result_callback(const GoalHandleFollowWaypoints::WrappedResult & result)
  {
    // 일시정지상태(q)이면 아무처리하지 않고 무시
    if (paused_) return;

    // waypoint 실패/ 취소 시 상태 초기화
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(this->get_logger(), "Waypoint failed or canceled");
      // 상태 초기화
      infinite_mode_ = false;   // 무한 중지 해제
      stop_requested_ = false;  // 정지 상태 해제
      goal_sent_ = false;       // 다음 goal 전송 가능 상태 변경
      return;
    }

    // waypoint 완료
    RCLCPP_INFO(this->get_logger(), "Waypoint lap completed");
    current_wp_index_=0;  // 한바퀴 끝났으므로 전체 경로 기준 waypoint 0으로 초기화

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
        std::chrono::seconds(2),   // 2초 대기
        [this]() {
          send_goal();     // 다음 바퀴 시작
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
