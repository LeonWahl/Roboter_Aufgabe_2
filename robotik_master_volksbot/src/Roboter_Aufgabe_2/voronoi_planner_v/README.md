## Aktevierung des voronoi_planner's

als erstes
```bash
ros2 run voronoi_planner_v voronoi_planner_node

```
als zweited
```bash
auto bot

```
als drittes
```bash
ros2 launch nav2_bringup bringup_launch.py map:=/home/leon-wahl/Documents/fertige_version_von_rplidar_robo/robotik_master_volksbot/src/meine_karte2.yaml 

```
als viertes
```bash
ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz

```
Dann müsst ihr in Rviz2 mit add das Topic "visualization_marker" aktevieren und dann könnt ihr die mittlern strecken mit den maximal abstand sehen

wenn das nicht funktioniert, dann nur den 3. Schritt neu starten lassen und dann kann man die Pfade sehen. 

Dabei kann man, wenn man einen bestimmten Punkt haben möchte im Terminal
```bash
ros2 topic echo /clicked_point
```
eingeben und dann mit "Publish Point" sich die exakten Koordinaten der Punkte ausgeben lassen