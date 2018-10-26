#DENMPC

    @file    README.md
    @Author  Jan Dentler (jan.dentler@uni.lu)
             University of Luxembourg
    @date    26.October, 2017
    @time    16:14h
    @license GPLv3
    @brief   README

## Outline
DENMPC is providing an object-oriented real-time nonlinear model predictive control (NMPC) framework which has been developed at the Automation & Robotics Research Group http://wwwde.uni.lu/snt/research/automation_robotics_research_group at the University of Luxembourg. 

The basic idea of DENMPC is to provide a fast nonlinear MPC that can adjust at runtime to different systems.
This refers to: 
*   Multi-agent systems that can change tasks, objectives and topology
*   Fault-tolerant control, where the controller has to adapt to different system conditions
*   Control prototyping, where you want to explore different scenarios without creating the underlying Optimal Control Problem (OCP) from scratch

In order to do so, DENMPC features an object-oriented modularization approach.
This allows structuring the control scenario into agents, constraints and couplings.
Out of these single components, DENMPC is dynamically creating the OCP at runtime.
As a result, agents, constraints and couplings can be added, removed, and parameters can be changed at runtime. This addition, respectively subtraction is triggered by events
which can be for example timer events, ROS-messages events, etc.
For very complex tasks, this can further be used to combine step chains with DENMPC,
to specialize the MPC for each task stage individually.

##Literature and Publication
DENMPC is open-source software, available under available under [https://github.com/snt-robotics/denmpc](https://github.com/snt-robotics/denmpc) and [https://github.com/DentOpt/denmpc](https://github.com/DentOpt/denmpc). The usage of DENMPC use regulated under the terms of the GPL3 license (Proprietary licences are available under request.). If you are using the software in your research work, you are supposed to cite one or more of the following references:

    J. Dentler, 
    "Real-time Model Predictive Control of Cooperative Aerial Manipulation",
    [http://orbilu.uni.lu/handle/10993/36965](http://orbilu.uni.lu/handle/10993/36965),
    PhD Thesis, University of Luxembourg, July 2018

    Jan Dentler, Somasundar Kannan, Souad Bezzaoucha, Miguel Angel Olivares-Mendez, and Holger Voos, 
    Model predictive cooperative localization control of multiple UAVs using potential function sensor constraints. 
    Autonomous Robots, March 2018, pages 1–26.
    doi: 10.1007/s10514-018-9711-z, url: https://doi.org/10.1007/s10514-018-9711-z

    J. Dentler, S. Kannan, M. A. O. Mendez and H. Voos,
    "A modularization approach for nonlinear model predictive control of distributed fast systems",
    24th Mediterranean Conference on Control and Automation (MED), Athens, Greece, 2016, pp. 292-297.
    doi: 10.1109/MED.2016.7535973

    Jan Dentler and Somasundar Kannan and Miguel Angel Olivares Mendez and Holger Voos,
    "A real-time model predictive position control with collision avoidance for commercial low-cost quadrotors",
    Proceedings of 2016 IEEE Multi-Conference on Systems and Control (MSC 2016), Argentina, Buenos Aires, 2016 

If you are using the "Condensed Multiple Shooting Generalized Minimal Residuum Method (CMSCGMRES)" kernel contributed by the team of Prof. Dr. Toshiyuki OHTSUKA, please refer to:

    Ohtsuka, T.,
    “A Continuation/GMRES Method for Fast Computation of Nonlinear Receding Horizon Control,”
    Automatica, Vol. 40, No. 4, Apr. 2004, pp. 563-574.

    Seguchi, H., and Ohtsuka, T.,
    “Nonlinear Receding Horizon Control of an Underactuated Hovercraft,”
    International Journal of Robust and Nonlinear Control, Vol. 13, Nos. 3-4, Mar.-Apr. 2003, pp. 381-398. 

## DENMPC features:

    Nonlinear model predictive control (e.g. a quadrotor with nonlinear system dynamics)
    Central control of single-agent systems (e.g. a single robot)
    Central control of multi-agent systems (e.g. multiple robots that are interacting)
    Object-oriented code to easily adapt it:
        Controller: Interface class for implementations of controllers, e.g.CMSCGMRES
        Agent: Interface class for implementations of agents, respective system or robot types, e.g. Quadrotor
        Constraint: Interface class for implementations of single-agent constraints
        Coupling: Interface class for implementations for coupling agents 
    Open-source code 


## Installation

    # Navigate to your ROS catkin workspace (e.g. catkin_ws):`
    cd catkin_ws/src
    #Clone repository
    git clone https://github.com/DentOpt/denmpc.git
    cd ..
    #Build package
    catkin_make

### To use the AR.Drone 2.0 scenario with the tum simulator, install

    cd catkin_ws/src
    git clone https://github.com/DentOpt/ardrone_simulator_gazebo7.git
    cd ..
    catkin_make

To run the AR.Drone 2.0 scenario in gazebo, run

    roslaunch cvg_sim\_gazebo ardrone_testworld.launch 

Launch drone (Takeoff) from commandline: 

    rostopic pub -1 /ardrone/takeoff std_msgs/Empty

The AR.Drone 2.0 simulator is configured to subscribe control commands under the topic "/cmd_vel".
The AR.Drone 2.0 pose is published under "/pose"


To control the AR.Drone 2.0 in with denmpc:
either to track center of UAV:

    rosrun denmpc scenario_ardrone_pose_tracking_node
 
 or to track with sensor constraint:
 
    rosrun denmpc scenario_ardrone_sensor_tracking_node
 
 and to send desired pose use rqt or commandline, e.g 
 
    rostopic pub /desiredpose geometry_msgs/PoseStamd '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 2.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
