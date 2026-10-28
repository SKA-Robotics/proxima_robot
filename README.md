# proxima_robot
ROS2 packages for Proxima robot.

## Test moteusów

W `proxima_description/` (katalogi `config/`, `launch/`, `robots/`) są pliki `moteus_test*`: szybko wygenerowany xacro i launch do testu samego manipulatora (3 × moteus, CAN ID 1–3).

Stan kodu: `MoteusSystem` łączy się z moteusami i odczytuje pozycję, prędkość i moment. `write()` nie jest jeszcze zaimplementowany, więc silniki się **nie ruszają**.

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

W logach powinny pojawić się 3 linie `Moteus id X OK: ...`.

W drugim terminalu (po `source install/setup.bash`):
```bash
ros2 control list_hardware_components
ros2 topic echo /joint_states
```
Po obróceniu wału ręką pozycja odpowiedniego jointa powinna się zmieniać (w radianach).

### Gdy coś nie działa

- `Permission denied` przy fdcanusb: wykonaj krok z udev.
- `No reply from moteus id X`: sprawdź silnik poza ROS-em:
```bash
  python3 -m moteus.moteus_tool --target X --info
```
