#include "rclcpp/rclcpp.hpp"
#include "robotik_pfadplannung/graph_structure.hpp"
#include "robotik_pfadplannung/a_star_solver.hpp"
#include "robotik_pfadplannung/msg/graph_state.hpp"
#include "robotik_pfadplannung/srv/get_path.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "nav_msgs/msg/path.hpp"

/**
 * @class PathPlannerNode
 * @brief Zentraler Pfadplanungs-Knoten für das Schienennetz.
 * 
 * 1. WAS DER CODE TECHNISCH MACHT:
 *    Initialisiert die Topologie aller Räume und Flursegmente, stellt den ROS 2 Service 
 *    "get_path" bereit, führt bei Aufruf AStarSolver::findPath() aus und publiziert 
 *    den Pfad sowohl als nav_msgs/msg/Path als auch als 3D-RViz-Linie.
 * 
 * 2. MATHEMATISCHES / ALGORITHMISCHES VERFAHREN:
 *    Topologisches Routing: Diskrete A*-Graphensuche mit euklidischer Heuristik.
 *    Dynamisches Kanten-Filtering (handleBlockedDoors) ermöglicht das Sperren von Kanten.
 * 
 * 3. WELCHES PROBLEM ES IM PROJEKT LÖST:
 *    Ermöglicht semantische Fahrbefehle ("Fahre von Raum_1_1 zu Studienraum")
 *    und wandelt diskrete Knotenfolgen in fahrbare Koordinatentrajektorien um.
 */


class PathPlannerNode : public rclcpp::Node {
private:
    TopologicalGraph graph_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::Subscription<robotik_pfadplannung::msg::GraphState>::SharedPtr state_sub_;
    rclcpp::Service<robotik_pfadplannung::srv::GetPath>::SharedPtr path_service_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;

    void initGraph() {
        // HAUPTWEG
        int r1_1      = graph_.addNode("Roblab_1_1", -2.754869222640991, -33.33885955810547);
        int r1_2    = graph_.addNode("Roblab_1_2", -4.557668209075928, -31.872848510742188);
        int r1_3    = graph_.addNode("Roblab_1_3", -4.284652233123779, -30.075668334960938);
        int tuer1   = graph_.addNode("Flurtür_1", -1.708473563194275, -33.34198760986328);
        int flur_0    = graph_.addNode("Flurteil0", -0.03991612792015076,-32.801170349121094);

        int flur_1=graph_.addNode("Flurteil1", 0.3051322400569916, -33.494625091552734);
        int flur_2=graph_.addNode("Flurteil2", 0.3998664617538452, -19.305286407470703);
        int flur_3=graph_.addNode("Flurteil3",0.012861302122473717, -17.810657501220703);
        int flur_4=graph_.addNode("Flurteil4", 0.4310295283794403, -11.746525764465332);
        int flur_5=graph_.addNode("Flurteil5", 0.42537298798561096, -9.49612045288086);
        int flur_6=graph_.addNode("FLurteil6", -3.6491968631744385, -9.367218017578125);
        int flur_7=graph_.addNode("Flurteil7", -3.8963027000427246, -0.04469136521220207);
        int flur_8=graph_.addNode("Flurteil8", -4.253158092498779, 8.59509563446045);
        int flur_9=graph_.addNode("Flurteil9", -2.9921622276306152, 8.477983474731445);
        int flur_10=graph_.addNode("Flurteil10", -0.29831233620643616, 8.59970474243164);

         int r2      = graph_.addNode("Raum_2", -0.14638783037662506, -37.243953704833984);

        int tuer3=graph_.addNode("Flurtür_3", 2.0195188522338867, -11.685135841369629);
        int r3=graph_.addNode("Studienraum", 4.203823089599609, -13.784309387207031);

        int tuer4=graph_.addNode("Flurtür_4", -1.0703904628753662, -11.654767036437988);
        int r4=graph_.addNode("Softlab", -3.600926399230957, -12.281171798706055);


        int tuer5=graph_.addNode("Flurtür_5", -2.6971442699432373,  -0.012658282183110714);
        int r5=graph_.addNode("Netlab", 1.750746488571167, -0.10640589892864227);

        int tuer6=graph_.addNode("Flurtür_6", -2.7506744861602783 , 9.916546821594238);
        int r6=graph_.addNode("Linuxlab", -3.498264789581299, 12.94306755065918);

        int tuer7=graph_.addNode("Flurtür_7", 1.083484172821045, 8.14848804473877);
        int r7=graph_.addNode("ESlab", 3.03608775138855, 9.285409927368164);



        // Kanten für Hauptweg
        graph_.addEdge(r1_3, r1_2);
        graph_.addEdge(r1_2, r1_1);
        graph_.addEdge(r1_1, tuer1);
        graph_.addEdge(tuer1, flur_0);
        graph_.addEdge(flur_0, flur_1);
        graph_.addEdge(flur_1, r2);
        


        graph_.addEdge(flur_0, flur_2);
        graph_.addEdge(flur_2, flur_3);
        graph_.addEdge(flur_3, flur_4);
        graph_.addEdge(flur_4, flur_5);
        graph_.addEdge(flur_5, flur_6);
        graph_.addEdge(flur_6, flur_7);
        graph_.addEdge(flur_7, flur_8);
        graph_.addEdge(flur_8, flur_9);
        graph_.addEdge(flur_9, flur_10);
        
        graph_.addEdge(flur_4, tuer3);
        graph_.addEdge(tuer3, r3);

        graph_.addEdge(flur_4, tuer4);
        graph_.addEdge(tuer4, r4);

        graph_.addEdge(flur_7, tuer5);
        graph_.addEdge(tuer5, r5);

        graph_.addEdge(flur_9, tuer6);
        graph_.addEdge(tuer6,r6);

        graph_.addEdge(flur_10, tuer7);
        graph_.addEdge(tuer7, r7);
    
    

    }

    void handleBlockedDoors(const robotik_pfadplannung::msg::GraphState::SharedPtr msg) {
        for (auto& edge : graph_.edges) {
            edge.is_blocked = false;
        }

        for (const auto& blocked_door_name : msg->node_names) {
            int idx = graph_.findNodeIndex(blocked_door_name);
            if (idx != -1) {
                for (auto& edge : graph_.edges) {
                    if (edge.from_index == idx || edge.to_index == idx) {
                        edge.is_blocked = true;
                    }
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Tür-Zustände im Planer aktualisiert.");
    }

    void handleGetPath(
        const std::shared_ptr<robotik_pfadplannung::srv::GetPath::Request> request,
        std::shared_ptr<robotik_pfadplannung::srv::GetPath::Response> response) 
    {
        RCLCPP_INFO(this->get_logger(), "Pfadanfrage erhalten: Von '%s' nach '%s'", 
                    request->start_node_name.c_str(), request->target_node_name.c_str());

        int start_idx = graph_.findNodeIndex(request->start_node_name);
        int target_idx = graph_.findNodeIndex(request->target_node_name);

        if (start_idx == -1 || target_idx == -1) {
            RCLCPP_ERROR(this->get_logger(), "Start- oder Zielknoten existiert nicht!");
            response->success = false;
            return;
        }

        // A*-Solver
        std::vector<int> path_indices = AStarSolver::findPath(graph_, start_idx, target_idx);

        if (path_indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "Kein gültiger Pfad gefunden (Türen blockiert?)");
            response->success = false;
            return;
        }

	    visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = this->now();
        path_marker.ns = "planned_path";
        path_marker.id = 99;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.15;
        path_marker.color.r = 1.0; path_marker.color.g = 0.5; path_marker.color.b = 0.0;
        path_marker.color.a = 1.0;

        for (int idx : path_indices) {
            geometry_msgs::msg::Point p;
            p.x = graph_.nodes[idx].x;
            p.y = graph_.nodes[idx].y;
            p.z = 0.2;
            response->waypoints_coordinates.push_back(p);
            response->passed_path_node_names.push_back(graph_.nodes[idx].name);
            path_marker.points.push_back(p);
	}

	path_marker_pub_->publish(path_marker);



// --- NEU: nav_msgs::Path (blauer Pfad) ---
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();

    for (int idx : path_indices) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = graph_.nodes[idx].x;
        pose.pose.position.y = graph_.nodes[idx].y;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);

    






        RCLCPP_INFO(this->get_logger(),"Pfad erfolgreich mit %zu Wegpunkten ausgegeben.", path_indices.size());
        response->success = true;
    }

public:
    PathPlannerNode() : Node("path_planner_node") {
        initGraph();

        // warte für Türänderungen
        state_sub_ = this->create_subscription<robotik_pfadplannung::msg::GraphState>(
            "blocked_doors", 10, std::bind(&PathPlannerNode::handleBlockedDoors, this, std::placeholders::_1));

        // Pfadplannung
        path_service_ = this->create_service<robotik_pfadplannung::srv::GetPath>(
            "get_path", std::bind(&PathPlannerNode::handleGetPath, this, std::placeholders::_1, std::placeholders::_2));

	path_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("planned_path_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Path Planner Node erfolgreich gestartet.");
   
   
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

    }
    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
