# ez-Wheel SWD® ROS2 Controllers

## Overview

This package has been tested on ROS2 Foxy, Galactic, Humble and Iron. It contains ROS2 nodes to control motors powered by the [ez-Wheel](https://www.ez-wheel.com) Safety Wheel Drive (SWD®) technology.

|![SWD-Core](https://www.ez-wheel.com/storage/image-product/visuels-swd-core-2-0-0.png) |![SWD-125](https://www.ez-wheel.com/storage/image-product/ezswd125im-31102023-photo-hd-det.png) |![SWD-150](https://www.ez-wheel.com/storage/image-product/roue-electrique-swd-150-2-0-0-0.png) |![SWD-StarterKit](https://www.ez-wheel.com/storage/image-product/starterkit-ez-wheel-web-0-0-0.png)|
| ------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| [SWD® Core](https://www.ez-wheel.com/en/safety-gear-motor)  | [SWD® 125](https://ez-wheel.com/en/safety-wheel-drive-swd-125) | [SWD® 150](https://www.ez-wheel.com/en/swd-150-safety-wheel-drive) | [SWD® StarterKit](https://www.ez-wheel.com/en/development-kit-for-agv-and-amr) |
| Safety gear motor                                           | Medium duty Safety Wheel Drive                                 | Heavy duty Safety Wheel Drive                                      | Development kit for AGV and AMR |

Users should regularly inform themselves about updates of this driver (Activating GitHub notifications with "Watch", 'All activity' button on top of this page).

## Prerequisites

- Two SWD® based wheels
- `SWD firmware` (**`>= 1.0.1`**)
- Ubuntu :
  - 20.04 (Focal Fossa) and ROS2 (Foxy or Galactic)
  - 22.04 (Jammy Jellyfish) and ROS2 (Humble or Iron)
- `swd-services` (**`>= 2.0.0`**)

## Pre-built debian package

It is available for the following platforms:

- ARM 64-bits, i.e. **arm64** debian packages
- AMD 64-bits, i.e. **amd64** debian packages, for x86 machines

## Installation

### METHOD 1: Ubuntu package manager

In order to install `swd_ros2_controllers` with apt, you need to add ez-Wheel repository to your Apt sources configuration file as sudo in: `/etc/apt/sources.list`. Type the following command:

```shell
echo "deb http://packages.ez-wheel.com:8081/ubuntu/ $(lsb_release -cs) main" | sudo tee -a /etc/apt/sources.list
```

Then, download and add the GPG key. Type the following command:

```shell
sudo bash -c "wget -qO - http://packages.ez-wheel.com:8081/archive.key | gpg --dearmor > /etc/apt/trusted.gpg.d/ez-wheel-keyring.gpg"
```

Now, you should be able to install ez-Wheel's packages using `Advanced Packaging Tool (apt)`:

```shell
sudo apt update && sudo apt install swd-services ros-${ROS_DISTRO}-swd-ros2-controllers
```

### METHOD 2: Compiling from source

To compile the package, make sure you added ez-Wheel repository to your Apt sources configuration file as sudo in: `/etc/apt/sources.list`, as specified above in METHOD 1.
So that you can install required `swd-services`, by typing:

```shell
sudo apt-get update && sudo apt install swd-services
```

In the following instructions, replace `<rosdistro>` with the name of your ROS2 distro (e.g., `galactic`).

```shell
source /opt/ros/<rosdistro>/setup.bash
mkdir -p ~/ros2_ws/src/
cd ~/ros2_ws/src/
git clone https://github.com/IDEC-ezWheel/swd_ros2_controllers.git
cd ..
colcon build
source ~/ros2_ws/install/setup.bash
```

## Usage

### A- On the SWD® Starter Kit

The package comes with a preconfigured launchfile which can be started using the `ros2 launch` command:

- `swd_diff_drive_controller_launch.py`: is a launchfile for the [SWD® Starter Kit](https://www.ez-wheel.com/en/development-kit-for-agv-and-amr) which has a differential kinematic. To use it, run the following command:

```shell
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ezw/usr/lib
ros2 launch swd_ros2_controllers swd_diff_drive_controller.launch.py
```

You can always use the node with the `ros2 run` command, the minimum required parameters are:

```shell
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ezw/usr/lib
ros2 run swd_ros2_controllers swd_diff_drive_controller --ros-args -p baseline_m:=0.485
```

The corresponding D-Bus services have to be started in order to use the nodes.

Example with the [SWD® Starter Kit](https://www.ez-wheel.com/en/development-kit-for-agv-and-amr):

- The service `ezw-dbus-user-session.service` is equivalent to running:

```shell
dbus-launch > /tmp/SYSTEMCTL_dbus.id ## [OPTIONAL]
```

- Prepare the environment

```shell
export $(cat /tmp/SYSTEMCTL_dbus.id) ## [OPTIONAL]
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ezw/usr/lib
```

- The service `ezw-swd-left.service` is equivalent to running:

```shell
/opt/ezw/usr/bin/ezw-smc-service /opt/ezw/usr/etc/ezw-smc-core/swd_left_config.ini
```

- The service `ezw-swd-right.service` is equivalent to running:

```shell
/opt/ezw/usr/bin/ezw-smc-service /opt/ezw/usr/etc/ezw-smc-core/swd_right_config.ini
```

Example of configuration files for [SWD® Starter Kit](https://www.ez-wheel.com/en/development-kit-for-agv-and-amr):

### `swd_left_config.ini`

```ini
# SMC Drive service config file
contextId = 12
nodeId = 4
coreNodeId = 6
coreNodeIsMaster = true # Slave:false Master:true
canDevice = can0
dbusNamespace = swd_left

HWConfigurationEntry = SWD_CORE
HWConfigurationFile = /opt/ezw/data/configuration.json

CANOpenEDSFile = /opt/ezw/usr/etc/ezw-canopen-dico/swd_core.eds
```

### `swd_right_config.ini`

```ini
# SMC Drive service config file
contextId = 12
nodeId = 5
coreNodeId = 7
coreNodeIsMaster = true # Slave:false Master:true
canDevice = can0
dbusNamespace = swd_right

HWConfigurationEntry = SWD_CORE
HWConfigurationFile = /opt/ezw/data/configuration.json

CANOpenEDSFile = /opt/ezw/usr/etc/ezw-canopen-dico/swd_core.eds
```

### `configuration.json`

```json
[
   {
      "name": "SWD_CORE",
      "nbStepRevolutionElec": 6,
      "nbPolePair": 5,
      "reduction": 14.0,
      "diameter": 125.0
   }
]
```

### B- On your own IPC

As the minimal SWD® Starter Kit config files do not exist on your IPC, you can install them manually as specified above or install them via a third package.

In this case, make sure you added ez-Wheel repository to your Apt sources configuration file as sudo in: `/etc/apt/sources.list`, as specified above in METHOD 1.

First, create `swd_sk` user with sudo rights, with `swd_sk` as default password:

```shell
sudo addgroup swd_sk
sudo useradd -m -s /bin/bash -g swd_sk swd_sk
sudo bash -c 'echo swd_sk:swd_sk | chpasswd'
sudo usermod -aG sudo swd_sk
```

Then log in with user `swd_sk`:

```shell
su - swd_sk
```

Then install `swd-system-config-2wheels` using:

```shell
sudo apt-get update && sudo apt install swd-system-config-2wheels
```

This package will configure your system to start at boot up four new services (with user `swd_sk` account):

- `ezw-stack.service` : initialize can0
- `ezw-dbus-user-session` : initialize D-Bus session
- `ezw-swd-left.service` : start left D-Bus service
- `ezw-swd-right.service` : start right D-Bus service

and add the following config files as specified above :

- `/opt/ezw/data/configuration.json`
- `/opt/ezw/usr/etc/ezw-smc-core/swd_left_config.ini`
- `/opt/ezw/usr/etc/ezw-smc-core/swd_right_config.ini`

This packages comes also with the `commissioning scripts` used for each wheels :

- `/opt/ezw/commissioning/`

You can modify them and apply the commissioning using:

```shell
cd /opt/ezw/commissioning/
./swd_left_4_commissioning.py
./swd_right_5_commissioning.py
```

Then refer to "Usage on a SWD® Starter Kit" for more information.

## About ROS2 `swd_diff_drive_controller` node

This controller manages two ez-Wheel SWD® wheels as a differential kinetic robot.

### Parameters

- `baseline_m` of type **`double`**: The distance (in meters) between the 2 wheels (**mandatory**).
- `left_swd_config_file` of type **`string`**: Path to the `.ini` configuration file of the left motor (default /opt/ezw/usr/etc/ezw-smc-core/swd_left_config.ini) (**read only**).
- `right_swd_config_file` of type **`string`**: Path to the `.ini` configuration file of the right motor (default /opt/ezw/usr/etc/ezw-smc-core/swd_right_config.ini) (**read only**).
- `pub_freq_hz` of type **`int`**: Frequency (in Hz) of published odometry and TFs (default `20`).
- `watchdog_receive_ms` of type **`int`**: The delay (in milliseconds) before stopping the wheels if no command is received (default `500`) (**read only**).
- `base_frame` of type **`string`**: Frame ID for the moving platform, used in odometry and TFs (default `'base_link'`) (see [REP-150](https://www.ros.org/reps/rep-0105.html) for more info).
- `odom_frame` of type **`string`**: Frame ID for the `odom` fixed frame used in odometry and TFs (default `'odom'`) (see [REP-150](https://www.ros.org/reps/rep-0105.html) for more info).
- `publish_odom` of type **`bool`**: Publish odometry messages (default `true`).
- `publish_tf` of type **`bool`**: Publish TF messages (default `true`).
- `publish_safety_functions` of type **`bool`**: Publish **`swd_ros2_controllers::msg::SafetyFunctions`** message (default `true`).
- `motor_max_speed_rpm` of type **`int`**: Maximum allowed motor speed (in RPM), if a target speed of one of the motor is above this limit, the controller will limit the speed of the two motor without changing the robot's trajectory (default `1050`).
- `motor_max_safety_limited_speed_1_rpm` of type **`int`**: Motor safety limited speed 1 (SLS_1) (in RPM), if an SLS_1 signal is detected (from a security LiDAR for example), the motor will be limited internally to the configured SLS_1 limit. The ROS2 controller uses this value to limit the target speed sent to the motor in the SLS_1 case (default `560`).
- `motor_max_safety_limited_speed_2_rpm` of type **`int`**: Motor safety limited speed 2 (SLS_2) (in RPM), if an SLS_2 signal is detected (from a security LiDAR for example), the motor will be limited internally to the configured SLS_2 limit. The ROS2 controller uses this value to limit the target speed sent to the motor in the SLS_2 case (default `680`).
- `have_backward_sls` of type **`bool`**: Specifies if the robot have a backward SLS signal, coming for example from a back-facing security LiDAR. If an SLS signal is available for backward movements, set this to `true` to take it into account. Otherwise, set the parameter to `false`, this will limit all backward movements to the selected `motor_max_safety_limited_speed_1_rpm` (default `false`).
- `left_encoder_relative_error` of type **`double`**: Relative error for left wheel encoder, used to calculate variances and propagate them to calculate the uncertainties in the odometry message. Each encoder acquisition **`DIFF_LEFT_ENCODER`** is modeled as: **`DIFF_LEFT_ENCODER +/- abs(left_encoder_relative_error * DIFF_LEFT_ENCODER)`** (default `0.2` corresponding to 20% of error).
- `right_encoder_relative_error` of type **`double`**: Relative error for right wheel encoder, used to calculate variances and propagate them to calculate the uncertainties in the odometry message. Each encoder acquisition **`DIFF_RIGHT_ENCODER`** is modeled as: **`DIFF_RIGHT_ENCODER +/- abs(right_encoder_relative_error * DIFF_RIGHT_ENCODER)`** (default `0.2` corresponding to 20% of error).

### Subscribed Topics

- `/cmd_vel` of type **`geometry_msgs/msg/Twist`**: Target linear and angular velocities.
- `/set_speed` of type **`geometry_msgs/msg/Point`**: Target speeds in rad/s for left (`Point.x`) and right (`Point.y`) wheels.
- `/soft_brake` of type **`std_msgs/msg/Bool`**: Activate or release the soft brake, send `false` to release the brake, or `true` to activate it.

### Published Topics

- `/odom` of type **`nav_msgs/msg/Odometry`**: Odometry message based on wheels encoders, containing the pose and velocity of the robot with their's associated uncertainties. Unless disabled by the `publish_tf` parameter, TFs with the same information are also published.
- `/safety` of type **`swd_ros2_controllers/msg/SafetyFunctions`**: Safety messages communicated by the wheels via CANOpen, the message includes information about Safe Torque Off (STO), Safety Limited Speed (SLS_1/SLS_2), Safe Direction Indication (forward/backward) (SDI+/-), and Safe Brake Control (SBC).

### Custom message types

#### `swd_ros2_controllers::msg::SafetyFunctions` message

This message provides information about the state of the Safety functions.
The value is `True` if the safety function is enabled.

```text
std_msgs/Header header
bool safe_torque_off                        # Safe Torque Off (STO)
bool safe_brake_control                     # Safe Brake Control (SBC)
bool safety_limited_speed_1                 # Safety Limited Speed 1 (SLS_1)
bool safety_limited_speed_2                 # Safety Limited Speed 2 (SLS_2)
bool safe_direction_indication_forward      # Safe Direction Indication (positive)
bool safe_direction_indication_backward     # Safe Direction Indication (negative)
```

The main safe drive function is the STO whereby the immediately torque-off on the motor may be accompanied by an SBC command to close the brakes.
The SLS functions cause the drive to decelerate (if required) and monitor whether the velocity is held within the defined limits.
The functions SDIp and SDIn enable the motor movement only in the corresponding (positive or negative) direction.

## Support

For any questions, [open a GitHub issue](https://github.com/IDEC-ezWheel/swd_ros2_controllers/issues).

## About ez-Wheel®

**ez-Wheel®** is an innovative company founded in 2009 and located in Angoulême, France.
ez-Wheel has developed the first industrial wheel drive, integrating motorisation, embedded electronics and batteries.

This revolutionary solution, which quickly turns any manually handled platform into an electrically assisted one.
Our solutions have been adopted by hundreds of end-users to improve productivity and prevent work accidents caused by manual handling.
Our products are used in a variety of applications, in fields of Automotive, Factory logistics, Warehouses, Food processing, Hospitals and Pharmaceutical industries.

SWD® products tackles industrial robotics applications, like Autonomous Mobile Robots (AMRs) and Automatic Guided Vehicles (AGVs).
It provides a unique solution for safety critical systems, with safety features related to the **ISO 3691-4** standard.

[ez-Wheel®](https://www.ez-wheel.com) has developed a unique know-how in embedded electronics, including safety critical systems, applied to battery powered electric traction.

## Troubleshooting

### Update `swd-services`

```shell
sudo apt update && sudo apt install swd-services
```

Check if other `swd-packages` can be upgraded:

```shell
sudo apt search swd-
```

### Error during commissioning

Before applying commissioning, stop `swd_diff_drive_controller` node.
Then, execute again python scripts in the commissionning directory:

```shell
./swd_"[...]"_commissioning.py
```

### Error with dbus session

```shell
export LD_LIBRARY_PATH=/opt/ezw/usr/lib
export $(cat /tmp/SYS*.id)
```

Then restart ezw-swd-left service and ezw-swd-right service.

### Debug SWD with diagnostic tool

Use `remote.py` script installed with `swd-services` package in `/opt/ezw/usr/sbin` directory, with its dbusNamespace as argument, e.g.:

``` shell
/opt/ezw/usr/sbin/remote.py smc_drive
/opt/ezw/usr/sbin/remote.py swd_left
/opt/ezw/usr/sbin/remote.py swd_right
```

### CAN socket is not configured

``` shell
sudo ip link set down can0
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 1000
