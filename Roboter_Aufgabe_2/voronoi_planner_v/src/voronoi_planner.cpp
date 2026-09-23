#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp" 
#include <opencv2/opencv.hpp>

class VoronoiFullMapAnalyzer : public rclcpp::Node
{
public:
  VoronoiFullMapAnalyzer() : Node("voronoi_path_analyzer")
  {
    // Best-Effort und Transient-Local QoS für sofortiges Abfangen der Karte beim Start
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos, 
      std::bind(&VoronoiFullMapAnalyzer::map_callback, this, std::placeholders::_1));

    // Marker Publisher für RViz2
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/visualization_marker", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    RCLCPP_INFO(this->get_logger(), "Voronoi Skelett-Analyzer (Mittelstrecke) gestartet.");
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Karte empfangen! Berechne präzise Mittellinien-Skelettierung...");
    latest_map_ = msg;
    compute_and_publish_full_voronoi();
  }

  void compute_and_publish_full_voronoi()
  {
    if (!latest_map_) return;

    int width = latest_map_->info.width;
    int height = latest_map_->info.height;
    float res = latest_map_->info.resolution; // 0.030m (3cm pro Pixel)
    float map_origin_x = latest_map_->info.origin.position.x; 
    float map_origin_y = latest_map_->info.origin.position.y; 

    // 1. Binärmaske aus der geladenen Karte erstellen (Invertiert für OpenCV)
    cv::Mat binary_map(height, width, CV_8UC1);
    for (int y = 0; y < height; ++y) {
      int ros_y = height - 1 - y; // ROS-Grid startet unten links, OpenCV oben links -> y invertieren
      for (int x = 0; x < width; ++x) {
        int index = x + ros_y * width;
        int cost = latest_map_->data[index];
        // 0 = Hindernis/Unbekannt, 255 = Sicher befahrbarer Freiraum
        binary_map.at<uchar>(y, x) = (cost > 65 || cost == -1) ? 0 : 255;
      }
    }

    // 2. Euklidische Distanztransformation (L2-Norm) berechnen
    cv::Mat distance_map;
    cv::distanceTransform(binary_map, distance_map, cv::DIST_L2, 3);

    // 3. Vorbereitung der Marker-Nachricht für RViz2 (POINTS)
    visualization_msgs::msg::Marker points_marker;
    points_marker.header.frame_id = latest_map_->header.frame_id; // Kopiert den Frame (meistens "map")
    points_marker.header.stamp = this->now();
    points_marker.ns = "voronoi_skelett_strecken";
    points_marker.id = 2;
    points_marker.type = visualization_msgs::msg::Marker::POINTS;
    points_marker.action = visualization_msgs::msg::Marker::ADD;

    // Punktgröße in RViz2 (Dicke der Skelett-Linie: 6 cm große Punkte auf deiner 3cm-Karte)
    points_marker.scale.x = 0.06; 
    points_marker.scale.y = 0.06; 

    // Farbe: Helles, stark sichtbares Grün
    points_marker.color.r = 0.0f;
    points_marker.color.g = 1.0f;
    points_marker.color.b = 0.0f;
    points_marker.color.a = 1.0f;

    // Textdatei im Home-Verzeichnis vorbereiten
    std::ofstream datei;
    std::string home_dir = std::getenv("HOME");
    datei.open(home_dir + "/alle_voronoi_koordinaten.txt", std::ios::trunc);

    if (datei.is_open()) {
      datei << "# Format: X-Koordinate (Meter), Y-Koordinate (Meter), Wandabstand (Meter)\n";
    }
    
    // Mindestsicherheitsabstand zur Wand (0.06m = 6 cm / 2 Pixel Platz zu Hindernissen)
    float min_clearance_pixels = 0.06f / res;
    int punkt_zaehler = 0;

    // 4. Präzise Skelettierung: Findet die exakte Mitte zwischen den Wänden
    for (int y = 1; y < height - 1; ++y) {
      for (int x = 1; x < width - 1; ++x) {
        float dist = distance_map.at<float>(y, x);

        // Kantenrauschen direkt an den Wänden ignorieren
        if (dist < min_clearance_pixels) continue; 

        // Nachbarpixel auslesen (4-er Nachbarschaft)
        float d_left  = distance_map.at<float>(y, x - 1);
        float d_right = distance_map.at<float>(y, x + 1);
        float d_up    = distance_map.at<float>(y - 1, x);
        float d_down  = distance_map.at<float>(y + 1, x);

        // Mathematische Skelettierungs-Bedingung für eine dünne Linie:
        // Ein Pixel gehört zum Skelett (Mittelstrecke), wenn es lokal den maximalen Abstand 
        // zu den Wänden aufweist. Wir prüfen strikt getrennt auf horizontale ODER vertikale Maxima.
        // Das Gleichheitszeichen `>=` stellt sicher, dass Plateaus in Türmitten lückenlos geschlossen werden.
        bool is_horizontal_max = (dist >= d_left && dist >= d_right) && (d_left > 0 && d_right > 0);
        bool is_vertical_max   = (dist >= d_up   && dist >= d_down)  && (d_up > 0   && d_down > 0);

        // Um Ausfransungen an dicken Kreuzungen zu verhindern, filtern wir rein flache Zonen heraus
        if ((is_horizontal_max && dist > d_left) || (is_horizontal_max && dist > d_right) ||
            (is_vertical_max && dist > d_up)     || (is_vertical_max && dist > d_down))
        {
          // Von OpenCV-Bildkoordinaten in das echte ROS-Weltkoordinatensystem (Meter) umrechnen
          int ros_y_pixel = height - 1 - y;
          geometry_msgs::msg::Point p;
          p.x = map_origin_x + (x * res);
          p.y = map_origin_y + (ros_y_pixel * res);
          p.z = 0.0;

          points_marker.points.push_back(p);
          punkt_zaehler++;

          // In Textdatei abspeichern
          if (datei.is_open()) {
            datei << p.x << ", " << p.y << ", " << (dist * res) << "\n";
          }
        }
      }
    }

    if (datei.is_open()) {
      datei.close();
    }

    // Marker an RViz2 senden
    marker_pub_->publish(points_marker);
    
    RCLCPP_INFO(this->get_logger(), "SKELETTIERUNG FERTIG! %d Mittellinien-Punkte extrahiert.", punkt_zaehler);
    RCLCPP_INFO(this->get_logger(), "Koordinaten (x, y) wurden in '~/alle_voronoi_koordinaten.txt' gespeichert.");
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoronoiFullMapAnalyzer>());
  rclcpp::shutdown();
  return 0;
}
