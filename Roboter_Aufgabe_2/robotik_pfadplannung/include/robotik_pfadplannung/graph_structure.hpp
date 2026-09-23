#ifndef GRAPH_STRUCTURE_HPP
#define GRAPH_STRUCTURE_HPP

#include <string>
#include <vector>
#include <cmath>

// Punkt auf der Karte (z. B. eine Station oder eine Tür)
struct Node {
    std::string name;
    double x;
    double y;
};

// Verbindung zwischen zwei Punkten
struct Edge {
    int from_index;   // Startpunkt
    int to_index;     // Endpunkt
    double weight;    // Länge der Strecke (Gewicht für die Wegfindung)
    bool is_blocked;  // Sperrung
};

// Das gesamte Streckennetz
class TopologicalGraph {
public:
    std::vector<Node> nodes;
    std::vector<Edge> edges;

    // Fügt einen neuen Punkt hinzu und gibt seine ID (Index) zurück
    int addNode(const std::string& name, double x, double y) {
        nodes.push_back({name, x, y});
        return static_cast<int>(nodes.size()) - 1;
    }

    // Verbindet zwei Punkte miteinander
    int addEdge(int from, int to) {

        // Berechnet ganz einfach die Distanz (Luftlinie) zwischen den zwei Punkten
        double dx = nodes[to].x - nodes[from].x;
        double dy = nodes[to].y - nodes[from].y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        // automatische Verbindung für beide Richtungen:
        edges.push_back({from, to, dist, false});
        edges.push_back({to, from, dist, false});
        
        return static_cast<int>(edges.size()) - 1;
    }

    // wie IndexOf in Java
    int findNodeIndex(const std::string& name) const {
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (nodes[i].name == name) {
                return static_cast<int>(i);
            }
        }
        return -1; 
    }
};

#endif // GRAPH_STRUCTURE_HPP