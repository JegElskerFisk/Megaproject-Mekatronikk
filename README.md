# UR-robot Cube Pointer – ROS 2 Workspace

Dette prosjektet består av flere ROS 2-pakker som sammen kontrollerer en UR-robotarm til å peke på tre kuber i en planleggingsscene. Systemet bruker MoveIt for planlegging og utførelse, samt TF2 for å oppdatere posisjonene til objektene i sanntid.

---

## Pakker

### 1. `cube_pointer_controller`
- Styrer UR-manipulatoren til å peke på kubene i rekkefølge.
- Bruker MoveIt sin action-klient for planlegging og utførelse.
- Tilbyr tjenester for restart og hjemposisjonering.
- Lytter på transformasjoner fra TF og sender mål til `move_group`.

[README_cube_pointer_controller.md](./README_cube_pointer_controller.md)

---

### 2. `scene`
- Publiserer kuber og bord som kollisjonsobjekter i MoveIt.
- Kringkaster posisjoner for kubene via TF (10 Hz).
- Lytter på `/monitored_planning_scene` for interaktiv endring.
- Bord legges til én gang via en egen node.

 [README_scene.md](./README_scene.md)

---

## Nodekommunikasjon

Systemet er koblet sammen via følgende noder og topics:

### Viktige topics og actions

| Topic / Action                                | Beskrivelse                                |
|-----------------------------------------------|--------------------------------------------|
| `/move_action`                                 | Brukes av `cube_pointer_controller` for å sende mål til MoveIt |
| `/apply_planning_scene`                        | Brukes av `scene`-pakkens noder for å legge til objekter |
| `/monitored_planning_scene`                    | Brukes av `interactive_scene` for å oppdage interaktive endringer i RViz |
| `/tf`                                          | Publisere posisjonene til `cube1`, `cube2`, `cube3` |
| `/robot_state_publisher`, `/joint_states`      | Standard robottoppics |
| `/scaled_joint_trajectory_controller`          | Utfører planlagte bevegelser |
| `/move_group`                                  | Planleggingsnode fra MoveIt |
| `/interactive_scene`                           | TF-publisher og objektoppdaterer |

Se også vedlagte grafiske oversikt over nodeforbindelser.

---

##  Oppstart

### 1. Bygg workspace

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### 2. Start systemet

Start MoveIt og robothardware (f.eks. med `ur_bringup`), deretter:

```bash
ros2 launch scene scene.launch.py
ros2 launch cube_pointer_controller controller.launch.py
```

---

## Avhengigheter

Systemet krever:

- ROS 2 Jazzy
- `moveit_ros_planning_interface`
- `tf2_ros`, `geometry_msgs`, `std_srvs`
- UR3 MoveIt-konfigurasjon (`ur_bringup` / `Universal_Robots_ROS2_Driver`)

---


