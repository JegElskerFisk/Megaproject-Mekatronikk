# cube_pointer_controller

En ROS 2-node som bruker MoveIt og TF2 til å styre en UR-manipulator mot tre kuber i rekkefølge. Robotarmen peker på hver kube basert på posisjonene deres i TF-rammen, og returnerer til hjemmeposisjon etterpå. Systemet benytter MoveIt's actiongrensesnitt og støtter restart og hjemposisjonering via tjenester.

---

## Innhold

- [Oversikt](#oversikt)
- [Avhengigheter](#avhengigheter)
- [Bygging](#bygging)
- [Kjøring](#kjøring)
- [Parametre](#parametre)
- [Tjenester](#tjenester)
- [Systeminteraksjon](#systeminteraksjon)

---

## Oversikt

Denne pakken inneholder én hovednode (`controller_node.cpp`) som:

- Initierer MoveIt action-klient
- Lytter etter posisjoner til `cube1`, `cube2`, `cube3` i TF
- Planlegger bevegelser for å peke på dem i rekkefølge
- Sender roboten hjem etter siste kube
- Har støtte for pause, restart og hjemposisjonering via tjenester

---

## Avhengigheter

Følgende ROS 2-pakker må være installert:

```bash
sudo apt install \
  ros-${ROS_DISTRO}-rclcpp \
  ros-${ROS_DISTRO}-rclcpp-action \
  ros-${ROS_DISTRO}-control-msgs \
  ros-${ROS_DISTRO}-geometry-msgs \
  ros-${ROS_DISTRO}-moveit-ros-planning-interface \
  ros-${ROS_DISTRO}-tf2-ros
```

---

## Bygging

```bash
cd ~/ros2_ws/src
git clone <repo-url>
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cube_pointer_controller
source install/setup.bash
```

---

## Kjøring

Start node og parameterkonfigurasjon:

```bash
ros2 launch cube_pointer_controller controller.launch.py
```

> Forutsetter at du har startet `move_group` fra en gyldig UR-konfigurasjon (f.eks. `ur_moveit_config`).

---

## Parametre

Plassert i `config/params.yaml`:

| Parameter            | Beskrivelse                                  | Standardverdi    |
|----------------------|----------------------------------------------|------------------|
| `home_joint_values`  | Liste med 6 joint-verdier for hjemposisjon   | `[]`             |
| `z_offset`           | Høyde over kuben for pekerposisjon           | `0.20`           |

---

## Tjenester

| Tjeneste   | Funksjon                          |
|------------|-----------------------------------|
| `/restart` | Starter sekvensen fra kube 1     |
| `/go_home` | Sender roboten til hjemposisjon  |

---

## Systeminteraksjon

- **TF**: Leser posisjoner fra `cube1`, `cube2`, `cube3` relativt til `base_link`
- **MoveIt**: Planlegger bevegelser via `move_group` action-klienten
- **Tjenester**: Ekstern styring via `std_srvs/Trigger`

---


