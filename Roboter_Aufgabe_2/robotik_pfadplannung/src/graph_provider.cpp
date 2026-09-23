#include "rclcpp/rclcpp.hpp"
#include "robotik_pfadplannung/graph_structure.hpp"
#include "robotik_pfadplannung/srv/set_door_state.hpp"
#include "robotik_pfadplannung/msg/graph_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/**
 * @class PathPlannerNode (Graph- & Planungs-Provider)
 * @brief Verwaltet das topologische Schienennetz, die Stationen und die Kantenstatus.
 * 
 * 1. WAS DER CODE TECHNISCH MACHT
 *    Initialisiert die topologische Weltkarte (Knoten wie Räume/Türen und Kanten als Schienen), 
 *    stellt einen ROS 2 Service ("get_path") für Routenabfragen bereit, verarbeitet Kanten-Blockaden 
 *    (über das Topic "blocked_doors") und publiziert den berechneten Pfad sowohl als 
 *    RViz-Marker-Linie als auch als "nav_msgs/msg/Path" für den Fahrregler.
 * 
 * 2. MATHEMATISCHES / ALGORITHMISCHES VERFAHREN
 *    - Topologische Graphendefinition $G = (V, E)$ mit gewichteten Kanten (euklidische Distanz).
 *    - Service-gesteuerte Adjazenzmatrix-Modifikation: Dynamisches Setzen des Flags is_blocked = true/false 
 *      an Kanten, wenn Türen gesperrt sind.
 * 
 * 3. WELCHES PROBLEM ES IM PROJEKT LÖST:
 *    Übersetzt abstrakte Gebäude- und Stationsstrukturen in einen mathematischen Graphen, 
 *    auf dem der A*-Algorithmus arbeiten kann. Zudem visualisiert er das Schienennetz 
 *    direkt in RViz2, sodass der Zustand des Roboters visuell nachvollzogen werden kann.
 */

class GraphProviderNode : public rclcpp::Node {
private:
    TopologicalGraph graph_;
    std::vector<std::string> blocked_doors_;

    // OS 2 Kommunikations-Referenzen wie Java-Objektreferenzen
    rclcpp::Service<robotik_pfadplannung::srv::SetDoorState>::SharedPtr door_service_;
    rclcpp::Publisher<robotik_pfadplannung::msg::GraphState>::SharedPtr state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr visual_timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;


    // feste raum-topologie
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

    void handleSetDoorState(
        const std::shared_ptr<robotik_pfadplannung::srv::SetDoorState::Request> request,
        std::shared_ptr<robotik_pfadplannung::srv::SetDoorState::Response> response) 
    {
        int idx = graph_.findNodeIndex(request->door_node_name);
        if (idx == -1) {
            RCLCPP_WARN(this->get_logger(), "Knoten '%s' nicht gefunden!", request->door_node_name.c_str());
            response->success = false;
            return;
        }

        // Kantenzustand synchronisieren
        for (auto& edge : graph_.edges) {
            if (edge.from_index == idx || edge.to_index == idx) {
                edge.is_blocked = !request->is_open;
            }
        }

        // Blockiert-Liste pflegen
	    if (!request->is_open) {
            if (std::find(blocked_doors_.begin(), blocked_doors_.end(), request->door_node_name) == blocked_doors_.end()) {
                blocked_doors_.push_back(request->door_node_name);
            }
        } else {
            blocked_doors_.erase(
                std::remove(blocked_doors_.begin(), blocked_doors_.end(), request->door_node_name), 
                blocked_doors_.end()
            );
        }

        RCLCPP_INFO(this->get_logger(), "Tür '%s' steht nun auf: %s", 
                    request->door_node_name.c_str(), request->is_open ? "OFFEN" : "GESCHLOSSEN");

        response->success = true;

	publishGraphState();
    }

    void publishGraphState() {
        auto msg = robotik_pfadplannung::msg::GraphState();
        msg.node_names = blocked_doors_; 
        state_pub_->publish(msg);
    }

    

    void publishRVizMarkers() {
        visualization_msgs::msg::MarkerArray marker_array;

        // Knoten als Punkte
        visualization_msgs::msg::Marker nodes_marker;
        nodes_marker.header.frame_id = "map";
        nodes_marker.ns = "topology_nodes";
        nodes_marker.id = 0;
        nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes_marker.action = visualization_msgs::msg::Marker::ADD;
        nodes_marker.scale.x = 0.3; nodes_marker.scale.y = 0.3; nodes_marker.scale.z = 0.3;
        nodes_marker.color.r = 0.0; nodes_marker.color.g = 0.0; nodes_marker.color.b = 1.0; nodes_marker.color.a = 1.0;

        for (const auto& node : graph_.nodes) {
            geometry_msgs::msg::Point p;
            p.x = node.x; p.y = node.y; p.z = 0.1;
            nodes_marker.points.push_back(p);
        }
        marker_array.markers.push_back(nodes_marker);

        // Kanten als Linien
        int edge_id = 1;
        for (const auto& edge : graph_.edges) {
            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = "map";
            line_marker.ns = "topology_edges";
            line_marker.id = edge_id++;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            line_marker.scale.x = 0.08;

            if (edge.is_blocked) {
                line_marker.color.r = 1.0; line_marker.color.g = 0.0; line_marker.color.b = 0.0;
            } else {
                line_marker.color.r = 0.0; line_marker.color.g = 1.0; line_marker.color.b = 0.0;
            }
            line_marker.color.a = 1.0;

            geometry_msgs::msg::Point p1, p2;
            p1.x = graph_.nodes[edge.from_index].x; p1.y = graph_.nodes[edge.from_index].y; p1.z = 0.05;
            p2.x = graph_.nodes[edge.to_index].x;   p2.y = graph_.nodes[edge.to_index].y;   p2.z = 0.05;

            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);
            marker_array.markers.push_back(line_marker);
        }

        marker_pub_->publish(marker_array);
    }

public:
    GraphProviderNode() : Node("graph_provider_node") {
        initGraph();

        // wait for Tür-Updates
        door_service_ = this->create_service<robotik_pfadplannung::srv::SetDoorState>(
            "set_door_state", 
            std::bind(&GraphProviderNode::handleSetDoorState, this, std::placeholders::_1, std::placeholders::_2));

        state_pub_ = this->create_publisher<robotik_pfadplannung::msg::GraphState>("blocked_doors", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("topology_markers", 10);

        // wird jede Sekunde aktualisiert
        visual_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), 
            std::bind(&GraphProviderNode::publishRVizMarkers, this));

        


        RCLCPP_INFO(this->get_logger(), "Graph Provider Node erfolgreich gestartet.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GraphProviderNode>());
    rclcpp::shutdown();
    return 0;
}
