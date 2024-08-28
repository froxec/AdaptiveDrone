# Adaptive drone

#### REPOSITORY IS UNDER MIGRATION PROCESS. IT IS BEING MIGRATED FROM LEGACY VERSION TO SIMPLIFY THE WHOLE ARCHITECTURE. LEGACY VERSION: https://github.com/froxec/AdaptiveDroneLegacy.

This repository contains:

- Docker compose configuration for [Ardupilot SITL simulation with Gazebo](https://ardupilot.org/dev/docs/sitl-with-gazebo.html) (ardupilot_gazebo image)
- Automonous system based on ROS2 (autonomous_system image).

It is assumed that drone mass can change during exploatation. The goal of a control system is to adapt to these changes.

Autonomous system implements Model Predictive Control algorithm which is aided by Gaussian Process to estimate current drone mass.

## Run containers from VSCode

1. Install VSCode and with [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).
2. While in repository working directory press Ctrl+Shift+P and run Dev Containers: Reopen in Container.

You have to do this both for ardupilot_gazebo and autonomous_system images.

## Ardupilot Software in the Loop simulation with gazebo

In ardupilot_gazebo container you have to run both ardupilot and gazebo.

### Run ardupilot_sitl

```
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -m "--out udp:localhost:6788"
```

### Run gazebo

In the 2nd terminal run:

```
gz sim -v4 -r iris_runway.sdf
```

Voilà, simulation should be running and you should be seeing gazebo GUI with Iris quadcopter.

### Troubleshooting

#### Gazebo doesn't find models run

Configure environment by running:

```
./run_entrypoint.bash
```

Then reload terminal or run:

```
source ~/.bashrc
```

#### Gazebo cannot connect to display :0 (Qt.xcb error)

Run the following command on HOST machine:

```
xhost +local:docker
```

## Autonomous system

In autonomous_system container run:

```
ros2 launch mavros apm.launch fcu_url:=udp://:6788@127.0.0.1
```

When you look at topics list you should see /mavros/\* topics.

```
ros2 topic list
```

### Running node

#### Debug mode

Build package

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

Then use visual studio code with launch configuration:

```
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Ros2 node debug",
            "type": "ros",
            "request": "launch",
            "target": "/home/rosdev/ws/mpc_control/launch/mpc_node_launch.py"
        },
    ]
}
```

#### Troubleshooting

If colcon build throws stderr SetuptoolsDeprecationWarning then you should downgrade setuptools as described [HERE](https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/).

### Interacting with ArduPilot

You can interact with Ardupilot by reading messages from /mavros topics and calling /mavros services.

Important services:

- /mavros/set_mode -> Setting quadcopter mode -> e.g. custom_mode='GUIDED'
- /mavros/cmd/arming -> arming command -> True/False

### RQT GUI

For easy gui topic/service development run RQT GUI:

```
rqt
```

### Visual Studio Code setup for ROS2 project development

To setup Visual Studio Code for effective development follow instructions from the article ["ROS 2 and VSCode"](https://picknik.ai/vscode/docker/ros2/2024/01/23/ROS2-and-VSCode.html) by Giovanni Remigi.
