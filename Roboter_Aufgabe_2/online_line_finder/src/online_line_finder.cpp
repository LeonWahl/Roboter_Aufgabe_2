
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "robotik_pfadplannung/srv/set_door_state.hpp"

using std::placeholders::_1;

/**
 * @class OnlineLineFinder
 * @brief Filtert Laserscans und extrahiert Wandlinien in Echtzeit.
 * 
 * 1. WAS ES TECHNISCH MACHT:
 *    Abonniert "/scan", wendet einen Medianfilter an, konvertiert Polar- in Kartesisch-Punkte
 *    und erkennt Wandstrukturen.
 * 
 * 2. MATHEMATISCHES / ALGORITHMISCHES VERFAHREN:
 *    - Gleitender 1D-Medianfilter (Fenster = 5).
 *    - Inkrementelle Linienverfolgung mit Distanzprüfung.
 * 
 * 3. GELÖSTES PROBLEM:
 *    Unterdrückt Rauschen und wandelt rohe Punktwolken in Wandlinien um.
 */

class OnlineLineFinder : public rclcpp::Node {
public:
  OnlineLineFinder() : Node("online_line_finder") {

    this->declare_parameter("epsilon_k_base", 0.2);
    this->declare_parameter("min_points", 2);
    this->declare_parameter("max_gap", 0.7);

    epsilon_k_base_ = this->get_parameter("epsilon_k_base").as_double();
    min_points_ = this->get_parameter("min_points").as_int();
    max_gap_ = this->get_parameter("max_gap").as_double();


//Laser Subscriber
    sub_filter_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&OnlineLineFinder::scanCallback, this, _1));
//Marker Publischer
    pub_filter_ =
        this->create_publisher<visualization_msgs::msg::Marker>("/lines", 10);

    RCLCPP_INFO(this->get_logger(), "Online Line Finder gestartet");
  
  
    door_client_=this->create_client<robotik_pfadplannung::srv::SetDoorState>("set_door_state");
//Publischer vom gefilterten scan
    pub_scan_=this->create_publisher<sensor_msgs::msg::LaserScan>("/scanout/scan",10);

  }

private:
  double epsilon_k_base_;
  int min_points_;
  double max_gap_;

  std::vector<std::pair<float, float>> aktuelle_linie_;
  std::vector<std::vector<std::pair<float, float>>> gefundene_linie_;

//Der scanCollback ruft sich hier immer wieder auf, wenn ein neues Laserscan-Paket empfangen wird
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

      //Der Median Filter, mit einem Fenster von der größe 5
     auto filtered_ranges = MedianFilter(
        msg->ranges,
        5,
        msg->range_min,
        msg->range_max
    );

    //Polarkoordinaten werden hier in kartesische Punkte umgewandelt
    std::vector<std::pair<float, float>> punkte;
    punkte.reserve(filtered_ranges.size());

    float winkelw = msg->angle_min;
    int n = filtered_ranges.size();

    for (int i = 0; i < n; i++) {

      float r = filtered_ranges[i];

      if (r < msg->range_min || r > msg->range_max) {
        winkelw += msg->angle_increment;
        continue;
      }

      float x = r * std::cos(winkelw);
      float y = r * std::sin(winkelw);

      punkte.emplace_back(x, y);

      winkelw += msg->angle_increment;
    }
/*
*
*
*
*/
//
    aktuelle_linie_.clear();
    gefundene_linie_.clear();



    //Prüft die linienverfolgung Inkrementell
    for (auto &p : punkte) {
      if (aktuelle_linie_.size() < 2) {
        aktuelle_linie_.push_back(p);
        continue;
      }

      auto &a_prev = aktuelle_linie_[aktuelle_linie_.size() - 2];
      auto &a_curr = aktuelle_linie_[aktuelle_linie_.size() - 1];
      int k = aktuelle_linie_.size();

      // prüft ob der neue Punkt auf der Linie liegt
      if (luftLinienOK(a_prev, a_curr, p, k)) {
        // wenn Abstand Ok dann wird er angehängt
        if (distanzFunktion(a_curr, p) <= max_gap_) {
          aktuelle_linie_.push_back(p);
        } else {
          //Wenn nicht wir eine neue Linie gebildet
          if (aktuelle_linie_.size() >= min_points_) {
            gefundene_linie_.push_back(aktuelle_linie_);
          }
          aktuelle_linie_.clear();
          aktuelle_linie_.push_back(p);
        }

      } else {
        //Der neue Punkt liegt nicht auf der Linie
        if (aktuelle_linie_.size() >= min_points_) {
          gefundene_linie_.push_back(aktuelle_linie_);
        }
        aktuelle_linie_.clear();
        aktuelle_linie_.push_back(p);
      }
    }
// Falls eine Linie am Ende offen ist wird sie gespeichert
    if (aktuelle_linie_.size() >= min_points_) {
      gefundene_linie_.push_back(aktuelle_linie_);
    }

      //Prüft alle Türbereiche und deren Zustand. Dabei schickt es asyncron eine Service-ANfrage an das Navigations-/ Pfadplannungssystem
    for(auto& door: doors){
      bool closed=detectDoorClosed(door);

      auto req=std::make_shared<robotik_pfadplannung::srv::SetDoorState::Request>();
      req->door_node_name=door.name;
      req->is_open=!closed;

      if(door_client_->wait_for_service(std::chrono::seconds(1))){
        door_client_->async_send_request(req);
      }
    }


    publishLines();

    sensor_msgs::msg::LaserScan filtered_msg=*msg;
    filtered_msg.ranges=filtered_ranges;

    pub_scan_->publish(filtered_msg);
  }

  /*
   *
   *
   *
   *
   *
   */
   
   //Visualisiert die linien
  void publishLines() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "laser";
    marker.header.stamp = this->now();
    marker.ns = "lines";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

      
    for (auto &linie : gefundene_linie_) {
      for (size_t i = 0; i < linie.size() - 1; i++) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = linie[i].first;
        p1.y = linie[i].second;
        p2.x = linie[i + 1].first;
        p2.y = linie[i + 1].second;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
      }
    }

    pub_filter_->publish(marker);
  }

  /*
   *
   *
   *
   *
   *
   *
   */

//Hier wird geguckt, bei den gefundenen Linien die Punkte von den Türen auch enthalten sind
   struct DoorRegion{
    std::string name;
    float xmin, xmax;
    float ymin, ymax;
   };

float TOLERANCE = 0.5f;

   std::vector<DoorRegion> doors={
        {
        "Flurtür_1",
        -1.708473563194275f - TOLERANCE, -1.708473563194275f + TOLERANCE,
        -33.34198760986328f - TOLERANCE, -33.34198760986328f + TOLERANCE
    },
    {
        "Flurtür_3",
         2.0195188522338867f - TOLERANCE,  2.0195188522338867f + TOLERANCE,
        -11.685135841369629f - TOLERANCE, -11.685135841369629f + TOLERANCE
    },
    {
        "Flurtür_4",
        -1.0703904628753662f - TOLERANCE, -1.0703904628753662f + TOLERANCE,
        -11.654767036437988f - TOLERANCE, -11.654767036437988f + TOLERANCE
    },
    {
        "Flurtür_5",
        -2.6971442699432373f - TOLERANCE, -2.6971442699432373f + TOLERANCE,
        -0.012658282183110714f - TOLERANCE, -0.012658282183110714f + TOLERANCE
    },
    {
        "Flurtür_6",
        -2.7506744861602783f - TOLERANCE, -2.7506744861602783f + TOLERANCE,
         9.916546821594238f - TOLERANCE,   9.916546821594238f + TOLERANCE
    },
    {
        "Flurtür_7",
         1.083484172821045f - TOLERANCE,  1.083484172821045f + TOLERANCE,
         8.14848804473877f - TOLERANCE,   8.14848804473877f + TOLERANCE
    }

   };

   bool detectDoorClosed(const DoorRegion& door){
    for(auto& linie : gefundene_linie_){

      for(auto& p :linie){
        if(p.first > door.xmin &&p.first<door.xmax &&
           p.second>door.ymin && p.second<door.ymax){
            float length=0.0;
            for(size_t i=0;i<linie.size()-1;i++){
              length +=distanzFunktion(linie[i], linie[i+1]);
            }
            if(length>0.3){
              return true;
            }
           }

      }
    }
    return false;
   }


   /*
   *
   *
   *
   *
   *
   *
   */
//Prüft die Distanz zu 
  float distanzFunktion(const std::pair<float, float> &a,
                        const std::pair<float, float> &b) {
    float dx = a.first - b.first;
    float dy = a.second - b.second;
    return std::sqrt(dx * dx + dy * dy);
  }
  /*
   *
   *
   *
   *
   *
   *
   *
   *
   */
   
   //Der Medianfilter
   std::vector<float>MedianFilter(const std::vector<float>& ranges, int window, float rmin, float rmax){
   const int n=ranges.size();
    const int halfte = window / 2;

    std::vector<float> filtered(n);

    for (int i = 0; i < n; i++) {

      int start = std::max(0, i - halfte);
      int end = std::min(n - 1, i + halfte);

      std::vector<float> values;
      values.reserve(window);

      for (int j = start; j <= end; j++) {
        float r = ranges[j];

        if (r >= rmin && r <= rmax) {
          values.push_back(r);
        }
      }

      if (values.empty()) {
        filtered[i] = ranges[i];
      }

      else {
        std::sort(values.begin(), values.end());
        filtered[i] = values[values.size() / 2];
      }
    }
    return filtered;
  }
   /*
   *
   *
   *
   *
   *
   *
   *
   */
//Prüft hier ob die Linien "gerade" sind
  bool luftLinienOK(const std::pair<float, float> &a_prev,
                    const std::pair<float, float> &a_curr,
                    const std::pair<float, float> &a_next, int k) {
    float d1 = distanzFunktion(a_prev, a_curr);
    float d2 = distanzFunktion(a_curr, a_next);
    float d3 = distanzFunktion(a_prev, a_next);

    if (d3 == 0) {
      return false;
    }

    float epsilon_k =
        static_cast<float>(epsilon_k_base_) / static_cast<float>(k);

    float R = d3 / (d1 + d2);

    return std::abs(d1+d2-d3)<0.2;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_filter_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_filter_;
  rclcpp::Client<robotik_pfadplannung::srv::SetDoorState>::SharedPtr door_client_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OnlineLineFinder>());
  rclcpp::shutdown();
  return 0;
}
