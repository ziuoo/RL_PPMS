#include <memory>

// ROS2
#include <rclcpp/rclcpp.hpp>

// 메시지 타입
#include <geometry_msgs/msg/pose.hpp>

// MoveIt2
#include <moveit/move_group_interface/move_group_interface.hpp>

// Custom Service
#include "system_interfaces/srv/move_to_point.hpp"

class TargetMoveServer : public rclcpp::Node
{
public:
  TargetMoveServer()
  : Node("target_move_server")
  {
    // Service 서버 생성
    service_ = this->create_service<system_interfaces::srv::MoveToPoint>(
      "move_to_point",
      std::bind(&TargetMoveServer::handle_service, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "Service server ready: /move_to_point");
  }

  void initialize()
  {
    // MoveGroupInterface 생성 (shared_from_this 사용을 위해 별도 함수에서 초기화)
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "manipulator"
    );

    // MoveIt 설정
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.1);

    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
  }

private:
  void handle_service(
    const std::shared_ptr<yolo_depth_interfaces::srv::MoveToPoint::Request> request,
    std::shared_ptr<yolo_depth_interfaces::srv::MoveToPoint::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received request: x=%.2f, y=%.2f, z=%.2f",
                request->target_position.x,
                request->target_position.y,
                request->target_position.z);

    // 목표 Pose 설정
    geometry_msgs::msg::Pose target_pose;
    
    // Orientation: 엔드 이펙터가 아래를 향하도록 설정 (X축 기준 180도 회전)
    // 쿼터니언: (x, y, z, w) = (1, 0, 0, 0)
    target_pose.orientation.x = 1.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;
    
    // Service로 받은 위치 설정
    target_pose.position.x = request->target_position.x;
    target_pose.position.y = request->target_position.y;
    target_pose.position.z = request->target_position.z;

    move_group_->setPoseTarget(target_pose);

    // 계획 생성
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));

    // 결과 실행
    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Planning succeeded! Executing trajectory...");
      move_group_->execute(plan);
      
      response->success = true;
      response->message = "Successfully moved to target position";
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      response->success = false;
      response->message = "Planning failed";
    }
  }

  rclcpp::Service<yolo_depth_interfaces::srv::MoveToPoint>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<TargetMoveServer>();
  node->initialize();  // shared_ptr 생성 후 초기화
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
