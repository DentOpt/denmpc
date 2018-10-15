New users start here:
=======

	mkdir -p ~/ros/src/denmpc && cd ~/ros/src/denmpc 
	git init
	git remote add gh git@github.com:benjaminabruzzo/denmpc.git
	git pull gh master
	git fetch gh
	git checkout denmpc_ardrone_gazebo
	cd ~/ros
	catkin build denmpc

---
This branch also depends upon a separate repo: (I haven't tested just denmpc and usma_ardrone on a standalone PC)

	mkdir -p ~/ros/src/usma_ardrone && cd ~/ros/src/usma_ardrone
	git init
	git remote add gh git@github.com:westpoint-robotics/usma_ardrone.git
	git pull gh master
	cd ~/ros
	catkin build usma_ardrone

To run:
=======

Launch gazebo first:

	roslaunch usma_ardrone_gazebo spawn_robots.launch 


Launch denmpc second:

	roslaunch denmpc test.launch 


send desired pose for ardrone:

	rostopic pub -r 20 /usma_ardrone/mpc/desiredpose geometry_msgs/PoseStamped '{header: {seq: 1,stamp: {secs: 1, nsecs: 0},frame_id: ''},pose: {position: {x: 1.0, y: 0.0, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' 


signal the ardrone to takeoff:

	rostopic pub -1 /ardrone/takeoff std_msgs/Empty

