# Robotik Pfadplanung

Backend für topologische Pfadplanung eines autonomen Roboters:
- verwaltet Raum-Topologie (Graph), 
- reagiert dynamisch auf geschlossene oder blockierte Türen,
- berechnet mithilfe des A*-Algorithmus den mathematisch kürzesten Pfad.

Aktuell nutzt es einen festen Map, der in `src/graph_provider.cpp` und `src/path_planner.cpp` hardkodiert ist.

**Was ist gemacht?**

- Datenstrukturen (Graph) definiert.
- Suchalgorithmus (A*) gebaut.

**Was ist zu machen?**

- 2D-Rasterkarte daran einbinden
- Tür-Logik programmieren
- Roboter-Bewegung von Pfadfinder in /cmd_vel übersetzen

## Ordner-Struktur

- `/include/robotik_pfadplannung/` : Header-Dateien (.hpp); für andere Nodes sichtbare Schnittstellen

- `/src/` : Implementierung (.cpp); funktionale Logik

    - Bibliotheken (wie `a_star_solver`) besitzen ein Header-Gegenstück in `include/`
    
    - andere Nodes (wie `test_planner`) benötigen keinen Header, da sie nicht von anderen Paketen importiert werden sollen.

- `/msg/` & `/srv/` : Definitionen der eigenen ros2-Interfaces.

## Was macht was?

### Kern-Nodes (Knotenpunkte)

- `src/graph_provider.cpp`
    - erstellt beim Start das topologische Layout der Räume und Türen,
    - bietet einen Service (`/set_door_state`), über den Türen dynamisch geöffnet/geschlossen werden können, färbt die betroffenen Wege in RViz rot, funkt den neuen Zustand an den Planer.

- `src/path_planner.cpp`
    - lauscht permanent auf Tür-Zustände, 
    - sobald ein anderer Node den Service `/get_path` aufruft, berechnet per A* den optimalen Weg und wirft den Pfad als orangefarbene Linie nach RViz.

### Algorithmen & Strukturen

- `include/robotik_pfadplannung/graph_structure.hpp`
    - definiert grundlegenden C++ Datenstrukturen für `Node` (Knoten mit X/Y-Koordinaten) und `Edge` (Verbindungen zwischen Räumen)
    
- `include/robotik_pfadplannung/a_star_solver.hpp`
    - enthält reine mathematische Logik des A*-Suchalgorithmus inklusive der euklidischen Distanzberechnung als Heuristik.

### Custom-Schnittstellen (Interfaces)

- `msg/GraphState.msg` -> Übertragung der Liste aktuell blockierter Räume/Türen zwischen Nodes
- `srv/SetDoorState.srv` -> Service, um den Zustand einer Tür zu ändern
- `srv/GetPath.srv` -> Hauptschnittstelle für den Roboter für Pfadsuchalgorithmus

## Nützliche ROS 2 Befehle für dieses Projekt

### 1. Workspace bauen & sourcen
```bash
cd ~/ros2_ws
colcon build --packages-select robotik_pfadplannung
source install/setup.bash
```

### 2. Nodes starten (in getrennten Terminals)
```bash
ros2 run robotik_pfadplannung graph_provider
ros2 run robotik_pfadplannung path_planner
```

### 3. Services aufrufen & testen

#### Einen Pfad anfordern (Normalfall oder Umleitung):
```bash
ros2 service call /get_path robotik_pfadplannung/srv/GetPath "{start_node_name: 'Raum_1', target_node_name: 'Raum_2'}"
```

#### Eine Tür schließen (A zum Umweg zwingen):*
```bash
ros2 service call /set_door_state robotik_pfadplannung/srv/SetDoorState "{door_node_name: 'Flurtür_1', is_open: false}"
```

#### Eine Tür wieder öffnen:
```bash
ros2 service call /set_door_state robotik_pfadplannung/srv/SetDoorState "{door_node_name: 'Flurtür_1', is_open: true}"
```

### 4. System-Diagnose
```bash
ros2 topic list                      # alle aktiven Datenströme
ros2 node info /path_planner_node    # alle Verbindungen des Planers
ros2 run rviz2 rviz2                 # weiter mit add, by topic, beide markers)
```
