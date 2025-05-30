# cube\_tracker

En ROS 2-pakke for deteksjon og posisjonering av kuber etter farge ved hjelp av et web-kamera, med republisering av TF-frames for hver kube.

---

## Innhold

* [Oversikt](#oversikt)
* [Avhengigheter](#avhengigheter)
* [Bygging](#bygging)
* [Kjøring](#kjøring)
* [Launch-fil](#launch-fil)
* [Parametre](#parametre)
* [Node-spesifikasjoner](#node-spesifikasjoner)
* [Systeminteraksjon](#systeminteraksjon)
* [Ressurser](#ressurser)

---

## Oversikt

`cube_tracker` består av tre noder:

1. **`cube_detector`**: Leser inn bilder fra et kamera, detekterer kuber ved hjelp av OpenCV og kamera­kalibrering.
2. **`cube_listener`**: Lytter på detekterings­data og publiserer geometri­meldinger.
3. **`cube_tf_republisher`**: Konverterer posisjons­data til TF2-frames for hver kube (`cube1`, `cube2`, ...) i koordinatsystemet til robot-base.

---

## Avhengigheter

* ROS 2 (Jazzy eller nyere)
* Python 3
* `rclpy`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `cv_bridge`
* `numpy`, `opencv-python`

---

## Bygging

Fra prosjektets rotmappe:

```bash
colcon build --packages-select cube_tracker
```

Husk å source installasjonen:

```bash
source install/setup.bash
```

---

## Kjøring

Du kan starte alle nodene med launch-fil:

```bash
ros2 launch cube_tracker static_camera.launch.py
```

Alternativt kan du kjøre nodene individuelt:

```bash
ros2 run cube_tracker cube_detector --ros-args --params-file ...
ros2 run cube_tracker cube_listener
ros2 run cube_tracker cube_tf_republisher
```

---

## Launch-fil

`launch/static_camera.launch.py` starter nodene med standard­parametre og bruker den forhåndskalibrerte kamera­filen (`resource/camera_calib.npz`).

---

## Parametre

| Parameter              | Beskrivelse                        | Standardverdi               |
| ---------------------- | ---------------------------------- | --------------------------- |
| `~image_topic`         | ROS-topic for kamera­bilder        | `/camera/image_raw`         |
| `~camera_info_topic`   | Topic for kamera­info              | `/camera/camera_info`       |
| `~calibration_file`    | Filbane til kalibreringsdata       | `resource/camera_calib.npz` |
| `~cube_size`           | Forventet kubestørrelse (meter)    | `0.05`                      |
| `~detection_threshold` | Threshold for farge-/formdeteksjon | `0.8`                       |

---

## Systeminteraksjon

* **Kamera:** Henter råbilder og kamerainfo
* **OpenCV:** Deteksjon basert på farge og form
* **TF2:** Republisering av posisjon som TF-frames
