# Manual Rover Control System (MRCS)
A repository for developing the User Interface for receiving data and controlling the rover


## Packages Info:

### command_broker:
* Command: `ros2 run command_broker XboxBroker`
* A node named XboxBroker that subscribes to 'joy' and publishes to 'motor_command' topic
* The topic 'motor_command' only includes the relevant data from the Xbox controller. The contents are likely to change.
* 'motor_command' contains a float array that is mapped as follows:
    - Data[0]  is left trigger. Resting is 0.99999 and fully compressed is -0.99999
    - Data[1] is right trigger.  Resting is 0.99999 and fully compressed is -0.99999
    - Data[2] is left shoulder: Resting is 0, compressed is 1
    - Data[3] is probably right shoulder. Controller button was broken **Check**
    - Data[4] is d pad left and right. resting is 0, left is 1 and right is -1


### teleop_twist_node:
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
* `source install/local_setup.bash`
* `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/'connection name'`
* Press the enable button on the microcontroller
* You should now be able to see the microcontroller's nodes and topics using ros2 topic list and ros2 node list.
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
