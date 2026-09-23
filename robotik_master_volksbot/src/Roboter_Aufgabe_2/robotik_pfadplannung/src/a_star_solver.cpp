#include "robotik_pfadplannung/a_star_solver.hpp"
#include <cmath>
#include <queue>
#include <vector>
#include <algorithm>
#include <limits>

/**
 * @brief Berechnet die Luftlinie (Distanz) zwischen zwei Punkten.
 * 
 * 1. WAS ES TECHNISCH MACHT:
 *    Ermittelt den euklidischen Abstand zweier Stationen im Koordinatensystem.
 * 
 * 2. MATHEMATISCHES VERFAHREN:
 *    Heuristikfunktion h(n) = sqrt((x1 - x2)^2 + (y1 - y2)^2).
 * 
 * 3. GELÖSTES PROBLEM:
 *    Dient dem A*-Algorithmus als zulässige Heuristik zur Schätzung der Restkosten.
 */
static double getDistance(const Node& n1, const Node& n2) {
    double dx = n1.x - n2.x;
    double dy = n1.y - n2.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief Führt den A*-Suchalgorithmus auf dem topologischen Graphen aus.
 * 
 * 1. WAS ES TECHNISCH MACHT:
 *    Sucht den optimalen Schienenpfad zwischen Start- und Zielknoten unter
 *    Berücksichtigung von Kantenblockaden.
 * 
 * 2. MATHEMATISCHES / ALGORITHMISCHES VERFAHREN:
 *    Graphensuche mit Prioritätswarteschlange und Kostenfunktion f(n) = g(n) + h(n).
 * 
 * 3. GELÖSTES PROBLEM:
 *    Garantiert den kürzesten Weg auf dem Schienennetzwerk.
 */



std::vector<int> AStarSolver::findPath(const TopologicalGraph& graph, int start_idx, int target_idx) {
    int num_nodes = static_cast<int>(graph.nodes.size());

    // 1. VALIDIERUNG: Liegen Start und Ziel im erlaubten Bereich?
    if (start_idx < 0 || start_idx >= num_nodes || target_idx < 0 || target_idx >= num_nodes) {
        return {};
    }

    // 2. VORBEREITUNG (Listen erstellen)
    // g_score: Kürzeste Strecke vom Startpunkt zu jedem Knoten (am Anfang "unendlich")
    std::vector<double> g_score(num_nodes, std::numeric_limits<double>::infinity());
    // came_from: Speichert den Rückweg für die Pfad-Rekonstruktion
    std::vector<int> came_from(num_nodes, -1);

    // open_set: Merkliste für offene Wege. Sortiert automatisch den kleinsten Wert nach oben.
    // Ein Element besteht aus: pair<f_score (Kosten), node_index (Knoten-ID)>
    using Element = std::pair<double, int>; 
    std::priority_queue<Element, std::vector<Element>, std::greater<Element>> open_set;

    // Startpunkt eintragen
    g_score[start_idx] = 0.0;
    double h_start = getDistance(graph.nodes[start_idx], graph.nodes[target_idx]);
    open_set.push({h_start, start_idx});

    // 3. HAUPTSCHLEIFE
    while (!open_set.empty()) {
        double current_f = open_set.top().first;
        int current = open_set.top().second;
        open_set.pop();

        // 4.1. ZIEL ERREICHT! Pfad von hinten nach vorne aufbauen
        if (current == target_idx) {
            std::vector<int> path;
            for (int at = target_idx; at != -1; at = came_from[at]) {
                path.push_back(at);
            }
            std::reverse(path.begin(), path.end()); // Umdrehen, damit er beim Start beginnt
            return path;
        }

        // Veralteten Eintrag in der Queue überspringen
        double current_h = getDistance(graph.nodes[current], graph.nodes[target_idx]);
        if (g_score[current] + current_h < current_f - 1e-5) {
            continue;
        }
        
        // 4.2. NACHBARN PRÜFEN
        for (const auto& edge : graph.edges) {
            if (edge.is_blocked) continue; // Wenn blockiert (z.B. Tür zu) -> Ignorieren

            // Prüfen, ob die Kante am aktuellen Knoten hängt
            int neighbor = -1;
            if (edge.from_index == current) {
                neighbor = edge.to_index;
            } else if (edge.to_index == current) {
                neighbor = edge.from_index; // Ungerichteter Graph (Verbindung gilt in beide Richtungen)
            } 
            
            if (neighbor == -1) continue;

            // Wegkosten berechnen
            double edge_weight = getDistance(graph.nodes[current], graph.nodes[neighbor]);
            double tentative_g = g_score[current] + edge_weight;

            // Besserer Weg gefunden?
            if (tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;

                // Gesamtkosten schätzen und ab auf die Merkliste
                double f_score = tentative_g + getDistance(graph.nodes[neighbor], graph.nodes[target_idx]);
                open_set.push({f_score, neighbor});
            }
        }
    }

    return {}; // Sackgasse (Kein Pfad vorhanden)
}
