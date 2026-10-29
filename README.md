# proxima_robot
Packages common to both physical and simulated Proxima robot.

## Test moteusów

W `proxima_description/` (katalogi `config/`, `launch/`, `robots/`) są pliki `moteus_test*`: szybko wygenerowany xacro i launch do testu samego manipulatora (3 × moteus, CAN ID 1–3).

Stan kodu: `MoteusSystem` łączy się z moteusami, używany jest Joint Trajectory Controller do sterowania silnikami. Można wysyłać komendy na dwa sposoby:
- publikowanie na topic /joint_trajectory_controller/joint_trajectory
- suwaki przez rqt

### Przygotowanie (jednorazowo)

Dostęp do fdcanusb bez roota:
```bash
echo 'SUBSYSTEM=="tty", ATTRS{manufacturer}=="mjbots", ATTRS{product}=="fdcanusb", MODE="0666", SYMLINK+="fdcanusb"' \
  | sudo tee /etc/udev/rules.d/99-fdcanusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```
Po wykonaniu odłącz i podłącz fdcanusb ponownie.

### Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select proxima_manip_controller proxima_description
source install/setup.bash
```

### Test bez silników

Sprawdza, czy URDF, launch i kontrolery działają:
```bash
ros2 launch proxima_description moteus_test.launch.py use_mock_hardware:=true
```

### Test z silnikami

```bash
ros2 launch proxima_description moteus_test.launch.py
```

Nowy terminal
```bash
source install/setup.bash
ros2 control load_controller --set-state active joint_trajectory_controller
```

Sprawdzenie, że kontrolery działają:
joint_state_broadcaster i JTC powinny być active

```bash
ros2 control list_controllers
```

Sterowanie silnikiem przez publikowanie komend:

```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ["joint1", "joint2", "joint3"], points: [{positions: [P1, P2+0.1, P3], time_from_start: {sec: 4}}]}"
```

- Ustawić P1 - P3 na oczekiwane pozycje (W radianach)
- Ruch będzie trwał ustaloną ilość sekund

Sterowanie suwakami:
- Uruchomić rqt z terminala systemowego (nie VS Code)
- Plugins -> Robot tools -> Joint trajectory controller
- Wybrać uruchomiony kontroler i włączyć

### Zatrzymanie
Ctrl + C w terminalu z launchem

### Gdy coś nie działa
Prawdopodobnie trzeba będzie pobrać pakiety:
- ros-jazzy-ros2-controllers (controller_manager, joint_state_broadcaster, joint_trajectory_controller)
- ros-jazzy-rqt-joint-trajectory-controller (opcjonalne, suwaki)


- `Permission denied` przy fdcanusb: wykonaj krok z udev.
- `No reply from moteus id X`: sprawdź silnik poza ROS-em:
```bash
python3 -m moteus.moteus_tool --target X --info
```
