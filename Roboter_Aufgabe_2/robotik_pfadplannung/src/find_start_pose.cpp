#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/lifecycle_service_client.hpp"
#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


using namespace std::chrono_literals;


    class AmclPoseReader : public rclcpp::Node
{
public:
  AmclPoseReader()
  : Node("amcl_pose_reader")
  {

    /* hier wird ein Subscriber für die Amcl pose gebaut, welche wartet, bis er eine Estemat pose bekommen hat
    */
    sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      std::bind(&AmclPoseReader::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting for AMCL pose...");
  }
    //sagt ob die Pose erhalten wurde 
  bool hasPose() const
  {
    
    return pose_received_;
  }
// hier wird die gespeicherte Pose zurückgegeben
  geometry_msgs::msg::PoseStamped getPose() const
  {
    return pose_;
  }



  /*Diese Node wartet auf die erste empfangene /amcl_pose und speichert diese ab
  *und setzt pose_received_ auf true
  */
private:
  void callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    pose_.header = msg->header;
    pose_.pose = msg->pose.pose;
    pose_received_ = true;

    RCLCPP_INFO(this->get_logger(),
      "AMCL pose received: x=%.3f y=%.3f",
      pose_.pose.position.x,
      pose_.pose.position.y);
  }

  // Hier wird empfanden, ob eine amcl_pose gefunden wurde
  // dabei wird der speicherplatz und der Subscriber hier festgelegt bzw. empfangen
  bool pose_received_ = false;
  geometry_msgs::msg::PoseStamped pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;
};








/**
Diese Node spricht mit Nav2 und schickt das Navigationsziel hier raus
 */

class BasicNavigatorCpp : public rclcpp::Node
{
public:
//Aliase für kürzere Namen
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

//Dieser Konstruktor verbindet sich mit der Nav2-Action client navigate_to_pose
  

  BasicNavigatorCpp()
  : Node("basic_navigator_cpp")
  {
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

/*
Diese Funktion setzt die Initalpose
*/

  void setInitialPose(const geometry_msgs::msg::PoseStamped & pose)
  {
    //Hier guckt sie ob diese gesetzt wurde und wenn nicht setzt sie diese selbst
    if(!initial_pose_pub_){
   initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);

   }

   /*Hier wird die PoseStamped in PoseWithCovarianceStamped konvertiert und setzt dabei die Kovarianz mit den Werten (x, y yaw)
   */
      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header=pose.header;
      msg.pose.pose=pose.pose;

      msg.pose.covariance[0]=0.25;
      msg.pose.covariance[7]=0.25;
      msg.pose.covariance[35]=0.0685;

    initial_pose_pub_->publish(msg);
  }


  /*
  Blockiert navigate_to_pose bis der Action Server verfügbar ist
  */
  void waitUntilNav2Active()
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 to activate...");
    nav_client_->wait_for_action_server();
    RCLCPP_INFO(this->get_logger(), "Nav2 is active.");
  }


  // Baut hier die gewünschte Ziel Pose mit NavigateToPose::Goal, wobei es das Ziel asynchron an Nav2 schickt
  // dabei setzt es ein Fedback Callback und ein Result CAllback
  std::shared_future<GoalHandleNavigateToPose::SharedPtr> goToPose(
    const geometry_msgs::msg::PoseStamped & goal_pose)
  {
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose;

    RCLCPP_INFO(this->get_logger(), "Sending goal...");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback =
      std::bind(&BasicNavigatorCpp::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback =
      std::bind(&BasicNavigatorCpp::resultCallback, this, std::placeholders::_1);

    return nav_client_->async_send_goal(goal_msg, send_goal_options);
  }







private:

//Das ist der Feedback Callback zum Ziel, die die verbleibende Zeit und Distanz ausgibt
  
  void feedbackCallback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(),
      "Distance remaining: %.2f m, ETA: %d s",
      feedback->distance_remaining,
      feedback->estimated_time_remaining.sec);
  }


  //Das ist der Result Callback, der je nach Ergebnis eine passende Log-Meldung ausgibt
  void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted.");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled.");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
        break;
    }
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

// Erstellt die Navigations-Node und liest die AMCL-Node
  auto navigator = std::make_shared<BasicNavigatorCpp>();
 auto amcl_reader=std::make_shared<AmclPoseReader>();

 // Wartet auf die Pose
 while(!amcl_reader->hasPose()){
    rclcpp::spin_some(amcl_reader);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
 }

//aktuallisiert die AMCL-Pose und den Zeitstempel der Navigations-Node
 auto initial_pose=amcl_reader->getPose();
 initial_pose.header.stamp=navigator->now();

// Wartet, bis Nav2 bereit ist und publiziert zweimal die Initalpose, um sicherzusein, dass
//Amcl sie wirklich übernimmt
  navigator->waitUntilNav2Active();
   navigator->setInitialPose(initial_pose);
   std::this_thread::sleep_for(std::chrono::milliseconds(200));
   navigator->setInitialPose(initial_pose);

  // Hier wird die Zielpose definiert
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = navigator->now();
  goal_pose.pose.position.x = -4.284652233123779;
  goal_pose.pose.position.y = -30.075668334960938;
  goal_pose.pose.orientation.w = 1.0;

  navigator->goToPose(goal_pose);

  // Hier wird ein Multi_Threaded Executor gestartet, damit beide Nodes Paralell laufen
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(navigator);
  exec.add_node(amcl_reader);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
