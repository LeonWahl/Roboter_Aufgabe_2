#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

/**
 * @class PathFollowerNode
 * @brief Schienen-Fahrregler zur präzisen Bahnführung entlang des A*-Pfades.
 * 
 * 1. WAS DER CODE TECHNISCH MACHT:
 *    Abonniert das Topic "planned_path" (nav_msgs/msg/Path), transformiert zyklisch 
 *    die Fahrzeugpose (map -> base_link) und publiziert Geschwindigkeiten 
 *    auf /cmd_vel (geometry_msgs/msg/Twist) über einen 20 Hz Timer.
 * 
 * 2. MATHEMATISCHES / ALGORITHMISCHES VERFAHREN:
 *    Zweiphasiger Kursführungsregler (Pure-Pursuit / Line-Tracking):
 *    - Gierwinkelfehler: angle_error = atan2(dy, dx) - yaw (auf [-pi, pi] normiert).
 *    - Phase 1 (Ausrichtung): Ist |angle_error| > 0.3 rad (~17°), stoppt die Translation
 *      und der Roboter dreht sich auf der Stelle (kein Schneiden von Ecken!).
 *    - Phase 2 (Schienenfahrt): Ist der Roboter ausgerichtet, fährt er vorwärts
 *      (proportional gedrosselt bei Zielannäherung) mit P-Regelung auf den Gierwinkel.
 * 
 * 3. WELCHES PROBLEM ES IM PROJEKT LÖST:
 *    Verhindert das Abdriften des Volksbots in Fluren und garantiert, dass er
 *    Türen und Engstellen exakt zentriert auf der Schienenachse durchfährt.
 */


class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode() : Node("path_follower_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
    {
        // Subscriber für den vom A*-Planer generierten Pfad
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "planned_path", 10,
            std::bind(&PathFollowerNode::onPathReceived, this, std::placeholders::_1));

         // Publisher für Geschwindigkeitsbefehle an das Fahrwerk
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        
        // Regelkreis-Timer: 50 ms Taktzeit (20 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PathFollowerNode::onTimer, this));

        RCLCPP_INFO(this->get_logger(), "PathFollowerNode gestartet.");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    int current_wp_ = 0;

    /**
     * @brief Speichert die Wegpunktliste ab und setzt den Wegpunkt-Index zurück.
     */
    
    void onPathReceived(const nav_msgs::msg::Path::SharedPtr msg) {
        waypoints_ = msg->poses;
        current_wp_ = 0;
        RCLCPP_INFO(this->get_logger(), "Pfad empfangen: %zu Wegpunkte", waypoints_.size());
    }

    // Roboterpose im Kartenkoordinatensystem (map -> base_link) abfragen
        geometry_msgs::msg::TransformStamped tf;
    void onTimer() {
        if (current_wp_ >= waypoints_.size()) {
            stopRobot();
            return;
        }

        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (...) { // Wir ignoriert, da das WLAN schlecht ist und zu Problemen führen kann
            return;
        }

        //Hier wird die position des Roboters extrahiert
        double robot_x = tf.transform.translation.x;
        double robot_y = tf.transform.translation.y;

        // Orientierung: Umrechnung Quaternion -> Euler (Roll, Pitch, Yaw)
        tf2::Quaternion q(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Zielkoordinaten des aktuellen Wegpunktes auslesen
        auto &wp = waypoints_[current_wp_].pose.position;
        //hier werden diese zum berechnen des Abstandes benutzt
        double dx = wp.x - robot_x;
        double dy = wp.y - robot_y;
        double distance = std::sqrt(dx*dx + dy*dy);
        // Sollwinkel und Regelfehler berechnen
        double target_angle = std::atan2(dy, dx);
        //und zur Berechnung der Winkelabweichung genutzt und zu Normalisierung an -pi und pi
        double angle_error = target_angle - yaw;

        while (angle_error > M_PI) angle_error -= 2*M_PI;
        while (angle_error < -M_PI) angle_error += 2*M_PI;

        geometry_msgs::msg::Twist cmd;
        
        //Dreht sich in die richtige Richtung
        if (std::fabs(angle_error) > 0.3) {
            cmd.angular.z = std::clamp(0.8 * angle_error, -1.0, 1.0);
            cmd.linear.x = 0.0;
        } else {
            //Genauere Ausrichtung, wenn der Roboter grob ausgerichtet ist
            cmd.angular.z = std::clamp(0.5 * angle_error, -1.0, 1.0);
            cmd.linear.x = std::min(0.2, distance);
        }
        // Wegpunkt-Umschaltung bei Erreichen des Akzeptanzradius (15 cm)
        if (distance < 0.15) {
            current_wp_++;
        }

        cmd_pub_->publish(cmd);
    }

    void stopRobot() {
        geometry_msgs::msg::Twist cmd;
        
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
