#               Thema 2 – Schienenverkehr: Topologisches Planen
### Autonome Navigation auf topologischen Graphen mit maximaler Hindernisvermeidung

                                               [Vladyslav, Boris, Leon]
                                                 [Robotik, SoSe 2026]

##    Problemspezifikation


    Die Aufgabe: Entwicklung eines autonomen Navigationssystems für ein komplettes Stockwerk. Über eine detaillierte Rasterkarte wird ein topologischer Graph aus benannten Referenzpunkten       gelegt. Der Roboter muss den kürzesten Weg finden und dabei sichere Abstände zu den Hindernissen halten.

    Umgebungsabstraktion: Transformation einer hochauflösenden Rasterkarte (Stockwerk mit vielen Räumen) in einen logischen, benannten topologischen Graphen.
    
    Pfadoptimierung: Berechnung des mathematisch kürzesten Weges zwischen zwei benannten Knotenpunkten (A nach B) mittels Graphensuche.
    
    Sicherheitsabstand: Optimierung des Abstands zu allen Wänden und Hindernissen während der Fahrt, um Kollisionen zu verhindern.


##    Aufgabenaufteilung


Vladyslav: Graph & Pfadplannung & A*-Algorithm

Boris: Online-Linienfinder & Voronoi-Planner & Kommentierung

Leon: Mapping & Pfadplannung & Online-Linienfinder & Voronoi-Planner & Übersetzungen und Verfeinerungen

 ##   Projektcode Aufgaben
```
online_line_finder              # // Ist die umgebaute Variante aus dem Unterricht, die jetzt auch dazu verwendet wird zu sagen, ob eine Tür offen oder geschlossen ist

voronoi_planner_v               # // Zum herausfinden des Maximalen Abstands

robotik_pfadplannung
     |__src
          |__a_star_solver       #// Ist der A* Algorithmus aber selbst erstellt
          |__find_start_pose     #// Ist dazu verantwortlich, das es genau zum Startpunkt fährt
          |__graph_provider      #//
          |__path_follower       #// Ist dazu da, dass der Roboter der Topologie verfolgt
          |__path_planner        #//

```

nähereres dazu in den Ordnern unter Roboter_Aufgabe_2 in den einzelnen README.md's 
