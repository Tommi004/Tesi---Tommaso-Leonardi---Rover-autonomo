# Sviluppo e Simulazione Software per un Rover Planetario Autonomo in Ambito Spaziale

Tesi di laurea triennale in Informatica — Università degli Studi di Camerino
**Laureando**: Tommaso Leonardi | **Relatrice**: Barbara Re | A.A. 2025/2026

## Descrizione

Simulazione 3D di un rover planetario autonomo, sviluppata con ROS 2 Jazzy e Gazebo Harmonic. Il sistema riproduce le funzionalità chiave delle missioni robotiche reali (come Perseverance e VIPER), seguendo l'approccio ROS 2/SpaceROS.

Il progetto realizza tre obiettivi principali:
1. **Navigazione ed evitamento ostacoli** — percezione tramite LiDAR simulato e pianificazione con Nav2
2. **Raggiungimento di target (waypoint)** — sequenza autonoma di destinazioni con calcolo del percorso ottimale
3. **Scansione dell'area** — acquisizione fotografica a 360° e accumulo di una nuvola di punti 3D ad ogni waypoint

## Architettura

Il sistema è organizzato in tre package ROS 2:

| Package | Responsabilità |
|---|---|
| `descrizione_rover` | Modello URDF del rover, mondo Gazebo, sensori (LiDAR, camera), filtro self-hit |
| `configurazione_nav2` | Stack di navigazione Nav2 (costmap, planner, controller, behavior server) |
| `nodo_missione` | Nodo Python che orchestra la sequenza di waypoint e la routine di scansione |

## Prerequisiti

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Nav2
- `laser_filters` (`sudo apt install ros-jazzy-laser-filters`)
- `cv_bridge`, `opencv-python`

## Compilazione

```bash
cd ~/rover_ws
colcon build
source install/setup.bash
```

## Utilizzo

**Terminale 1 — simulazione:**
```bash
ros2 launch descrizione_rover simulazione.launch.py
```

**Terminale 2 — stack di navigazione:**
```bash
ros2 launch configurazione_nav2 nav2.launch.py
```

**Terminale 3 — missione autonoma:**
```bash
ros2 run nodo_missione navigatore_missione
```

Il rover naviga in sequenza sui waypoint configurati, eseguendo ad ogni tappa una rotazione di scansione a 360° con acquisizione di 4 fotografie e di una nuvola di punti 3D. I risultati vengono salvati in `~/rover_ws/scansioni/`.

## Limiti noti

- Il LiDAR simulato è 2D puro (scansione orizzontale singola quota): la "scansione 3D" è quindi un accumulo di dati 2D nel tempo durante la rotazione, non una vera acquisizione volumetrica
- Il rendering di terreni heightmap non è supportato dall'ambiente grafico utilizzato (WSL2); il terreno è quindi piano, con texture e ostacoli in stile marziano

## Documentazione

Per l'analisi tecnica completa, la cronologia di sviluppo e debug, e i dettagli implementativi, vedere la [Wiki](../../wiki) del repository.
