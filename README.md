# Manual Rover Control System (MRCS)
A repository for developing the User Interface for receiving data and controlling the rover
## Packages Info

# Docker Containers for XBox controller Host and microcontroller with the Jetson

This guide will show you how to set up and launch the MRCS (Manual Rover Control System) using Docker on a bash terminal
## Required Packages
1. On a Linux OS or VM
2. Docker Installation (Do not install the snap store version of docker)
3. Latest version of Tmux: A link to a very useful guide: https://www.hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/

## Instructions Common
1. Open your bash terminal
2. Clone the latest version of the MRCS repo using `git clone git@github.com:GonzagaRobotics/ManualRoverControlSystem.git`

Steps 3 to 5 only if wanting to connect the joystick

3. type `ls -l /dev/input/j`
4. type `Tab` twice to find which number with `js#` is the XBox controller
    - Then press `Enter`
5. If the joystick is not `js0` some other `js#` then change the following:
    - run `cd ManualRoverControlSystem/autolaunch/launch`
    - in `auto_launch.py` change all instances of `js0` to `js#`
    - go back and run `cd ManualRoverControlSystem/docker_scripts/for_host`
    - in `docker-compose.yml` change all instances of `js0` to `js#`

## Instructions if only using one PC

Steps 1 to 2 only if wanting to connect a microcontroller
1. Plug in microconroller to computer with USB
2. Run `sudo chmod 666 /dev/ttyUSB0`
   - May have to change `ttyUSB#`
3. Go back and run `cd ManualRoverControlSystem/docker_scripts`
4. Run the command `tmux` (could also open a new terminal screen if you do not want to use `tmux`)
5. Open a tmux split screen terminal with `CTRL+B` and pressing `"`
   - To close out of a panel at any time, type `exit` or hit `CTRL+D`.
6. On the terminal navigate to `for_host` by running `cd for_host`
7. Run `docker compose up` to start the Xbox controller
8. Press `CTRL+B` and then the `up arrow` to navigate to the other termnial
9. In that terminal navigate to `for_jetson` by running `cd for_jetson`
10. Run `docker compose up -d`

For debugging

11. Open a tmux split screen terminal from here with `CTRL+B` and pressing `"`

Step 12 only if wanting to connect a microcontroller

12. In this terminal run `docker start -ai for_jetson-micro-ros-agent-1`
    - There should be messages from the microcontrollers here
13. Press `CTRL+B` and then the `up arrow` to navigate to the other `for_jetson` termnial
14. In this terminal run `docker exec -it for_jetson-ros_echo-1 bash`
15. In this docker container run `ros2 topic list` to see if `motor_command` is listed
16. Then run `ros2 topic echo /motor_command/left_trigger` to see if this container can see the left trigger

## Instructions if using the host PC and Jetson
1. setup antennas for the host PC and Jetson from the `antenna_setup.md` file in the repo
2. Go back and run `cd ManualRoverControlSystem/docker_scripts/for_host`
3. Run `docker compose up` to start the Xbox controller
4. On another terminal instance run `ssh robotics@192.168.0.2`
   - enter the password to enter into the Jetson
5. Plug in microconroller to Jetson with USB
6. In the Jetson if not automatically done run `sudo chmod 666 /dev/ttyUSB0`
   - May have to change `ttyUSB#`
7. In the Jetson run `cd ~/ManualRoverControlSystem/docker_scripts/for_jetson`
8. Run `docker compose up -d`

For debugging

9. On the Jetson run the command `tmux`
10. Open a tmux split screen terminal from here with `CTRL+B` and pressing `"`
11. In this terminal run `docker start -ai for_jetson-micro-ros-agent-1`
    - There should be messages from the microcontrollers here
13. Press `CTRL+B` and then the `up arrow` to navigate to the other `for_jetson` termnial
14. In this terminal run `docker exec -it for_jetson-ros_echo-1 bash`
15. In this docker container run `ros2 topic list` to see if `motor_command` is listed
16. Then run `ros2 topic echo /motor_command/left_trigger` to see if this container can see the left trigger

## Instructions the common result of from both
1. Press the `right trigger` on the XBox controller and the motors should spin

## Instructions common for debugging both

# Autolaunch

This guide will show you how to set up and launch the MRCS (Manual Rover Control System) using ROS 2 on a Docker bash terminal.

## Required Packages

1. ROS2 Docker Terminal
2. Latest version of Tmux: A link to a very useful guide can be found [here](https://www.hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/).

## Instructions

1. Open your ROS 2 Docker terminal.
2. Create a new directory named `ros2_ws` with a subdirectory called `src` using `mkdir -p ros2_ws/src`.
3. Navigate to `src` by running `cd ros2_ws/src`.
4. Clone the latest version of the MRCS repo using `git clone git@github.com:GonzagaRobotics/ManualRoverControlSystem.git`.
5. Navigate out of `src` in the top of the directory `ros2_ws` by running the command `cd ros2_ws`.
6. Build the project by running `colcon build`.
   - If you encounter the error message `stderr: teleop_twist_joy`, run `colcon build` again to resolve it.
7. Source ROS2 in the parent directory by running the command: `. install/setup.bash`.
8. Startup `tmux`.
   - A green status bar should appear. If it doesn't, make sure you have tmux installed in your docker container and try again.
9. Open a tmux split screen terminal
   - A link to tmux keybinds can be found above.
   - To close out of a panel at any time, type `exit` or hit `CTRL+D`.
   
10. Navigate to a terminal and run the command to launch all nodes at once: `ros2 launch src/ManualRoverControlSystem/autolaunch/launch/auto_launch.py`
   
   - By default, all nodes are launched with main launch command.
   - To stop specific nodes from launching, you can add any of the following commands after the main launch command: 
      - Add `xbox_broker:=False` to stop the Command Broker node from launching.
      - Add `command_exposer:=False` to stop the Command Exposer node from launching.
      - Add `command_receiver:=False` to stop the Jetson_Comm node from launching.
      - For now, `teleop_twist_node` always launches by default.
   - For example, if you wanted to turn stop both the Command Exposer and Jetson_Comm nodes from launching, you'd use the command: `ros2 launch src/ManualRoverControlSystem/autolaunch/launch/auto_launch.py command_exposer:=False command_receiver:=False`
   
      - At any time with the following command, you can stop the launch file by pressing `CTRL+C`. If you choose to launch all nodes at once, the output should look something like this:
    
      ``` [INFO] [launch]: All log files can be found below /root/.ros/log/2023-03-10-22-51-42-519396-2f922f3c3a13-19096
      [INFO] [launch]: Default logging verbosity is set to INFO
      [INFO] [joy_node-1]: process started with pid [19097]
      [INFO] [XboxBroker-2]: process started with pid [19099]
      [INFO] [CommandExposer-3]: process started with pid [19101]
      [INFO] [CommandReceiver-4]: process started with pid [19103]
      [CommandReceiver-4] [INFO] [1678488703.228953101] [CommandReceiver]: I have initialized up successfully.
      ```
      
11. Navigate to another terminal panel.
12. In the bottom panel, run `ros2 topic list`.
    - The list should look something like this:
    ```/cmd_vel
    /joy
    /joy/set_feedback
    /motor_command
    /motor_command_exposed
    /parameter_events
    /rosout
    ```
       
13. Congratulations, you’re all set!


### command_broker:
* Source from ros_ws: `. install/setup.bash`
* Command: `ros2 run command_broker XboxBroker`
* A node named XboxBroker that subscribes to 'joy' and publishes to 'motor_command' topic
* The topic 'motor_command' only includes the relevant data from the Xbox controller. The contents are likely to change.
* 'motor_command' contains multiple float messages that is mapped as follows:
    - left trigger. Resting is 0.99999 and fully compressed is -0.99999
    - right trigger: Resting is 0.99999 and fully compressed is -0.99999
    - left shoulder: Resting is 0, compressed is 1
    - right shoulder: Resting is 0, compressed is 1
    - d pad left and right. resting is 0, left is 1 and right is -1


### teleop_twist_node:
* From inside the ros_ws directory, run `. install/setup.bash`
* Command: `ros2 launch teleop_twist_joy teleop-launch.py joy_config:=’xbox’ joy_dev:=’dev/inputs/js1’`

    - Note the the name of the USB might change so js1 might be different

* Cloned from this repo: [Link](https://github.com/ros2/teleop_twist_joy/tree/humble)
* Responsible for receiving the Xbox controller commands and publishing them to the 'joy' topic




## Microcontroller Connection Instructions
These instructions assume the micro_ros_agent is already created on the host machine. To do this, follow the instructions on this link under Creating the micro-ROS agent https://micro.ros.org/docs/tutorials/core/first_application_linux/

Steps to connect microcontroller to ROS2:

* Plug microcontroller into Jetson over USB
* Determine the USB connection name: `ls -l /dev/tty*`
* (Optional) You might need to change the permissions on the USB to allow it to communicate: `sudo chmod 666 /dev/'connection name'`
* Go into microros_ws directory
* `source install/setup.bash`
* `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/'connection name'`
* Press the enable button on the microcontroller
* You should now be able to see the microcontroller's nodes and topics using ros2 topic list and ros2 node list.

## USB connection to Docker container
 * Linux automatically does this
 * Windows and Mac possible methods for USB connection
  * USB over IP: https://github.com/jiegec/usbip

## Goals
* The rover and its various sub-systems will be controllable via a Manual Rover Control System (MRCS). 
* The MRCS will allow a human operator to drive the rover and manage its various sub-systems, including the operation of the manipulator arm. 

* The MRCS will have a User Interface (UI), that displays critical environmental information to the operator, including video stream and sensor data. 

## Methodology
* We will design, implement, test, and document an intuitive UI to display environmental data to a human operator and allow them to control the rover. 

* The MRCS will utilize ROS2 to send commands to and receive data from the rover over a wireless connection. 
* We will test the various components and overall whole of the MRCS using a [Simulation](https://github.com/orgs/GonzagaRobotics/projects/11) of our rover. 

## Deliverables
* A Manual Control System (MRCS) that allows a human operator to drive the rover and operate its various components. 
    - The MRCS will display important data transmitted from the rover including video stream and other data. 

## Timeline
### October 2022: 
* Preliminary MRCS R&D complete: Architecture designed and framework(s) chosen for building, development started
### December 2022:
* MRCS v1.0.0 released: Major release of MRCS, rover sim can be fully controlled over ROS2 actions in a UI. Data from the rover (or rover sim) is displayed, including but not limited to video stream and sensor data. Physical rover will likely not be in a state for real world testing. Will need to test sending actions over the comms platform to the jetson once Electrical/Embedded has the comms platform ready to test.
### February 2023:
* MRCS (minor release v1.x.x) capable of controlling physical rover, if complete. If not complete, MRCS capable of communicating over Comms system to send and receive all data and commands that would be necessary for control. MRCS capable of initiating ARCS control sequence.
