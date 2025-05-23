# Crazyswarm Landing
Modified Crazyswarm for UAV-USV landing in MoCap space.  
Contact: 19jls9@queensu.ca

## Usage
1. Follow instructions found at https://crazyswarm.readthedocs.io/en/latest/installation.html to clone the crazyswarm repository and set up configurations in `~/crazyswarm/ros_ws/src/crazyswarm/launch/allCrazyflies.yaml` and `~/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml` and `~/crazyswarm/ros_ws/src/crazyswarm/launch/hover_swarm.launch`. The recommended setup for these files is included in the folder [launch](./launch).

2. Replace the file `vicon.cpp` located in `~/crazyswarm/ros_ws/src/crazyswarm/externalDependencies/libmotioncapture/src` with the version from this repo.

3. Replace the file `crtp_commander_rpyt.c` located in `~/crazyswarm/crazyflie-firmware/src/modules/src` with the version from this repo.

4. You should now be able to run the test script `helloWorld.py` from crazyswarm without issues. Troubleshooting for MoCap and Crazyflie setup can be found in the Notion.  
  
The following instructions will set up the cooperative MPC for Crazyflie and Husky in the MoCap space. For details on the files that we will add and modify see the section [Modified Files](#modified-files).

5. Add the file `pubCF.py` to `~/crazyswarm/ros_ws/src`. In `~/crazyswarm/ros_ws/src` create a new folder called `cf_sub_pub`. In `~/crazyswarm/ros_ws/src/cf_sub_pub` create two new folders `src` and `scripts`. Add the file `cf_sub_pub_node.py` to the `scripts` folder.

6. Add (as individual files) the three files included in the folder [crazyswarm_scripts](./crazyswarm_scripts/) to the directory `~/crazyswarm/ros_ws/src/crazyswarm/scripts`.

7. Place the entire folder [MPC_take2](./MPC_take2/) in the directory `~/crazyswarm/ros_ws/src/crazyswarm/scripts`.

8. Replace the file `crazyflie.py` located in `~/crazyswarm/ros_ws/src/crazyswarm/scripts/pycrazyswarm/` with the version from this repo.

You should now be able to run the MPC on the Crazyflie.

9. First ensure that the master node that is run by the Husky is prepared. To do this follow the instructions found at https://github.com/Robora-Lab/husky_landing. Ensure that the device running the Crazyflie is able to communicate with the Husky by setting the ROS_MASTER_URI variable in `.bashrc ` do this using `nano .bashrc` and write two lines at the end of the code specifying `export ROS_MASTER_URI=http://ip addr of Husky` and `export ROS_IP=ip addr your machine`. Then you can open a terminal window and set a preliminary goal for the Crazyflie on the topic */goalPoseCF*.

```bash
rostopic pub /goalPoseCF geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"
```

10. Open two more terminals and navigate to your crazyswarm directory `cd crazyswarm` then source the setup `source ros_ws/devel/setup.bash`.

11. In the first terminal start the crazyswarm server (and place the Crazyflie in the space) as you would to run `helloWorld.py` using `roslaunch crazyswarm hover_swarm.launch`.

12. In the second terminal navigate the the scripts folder `cd ros_ws/src/crazyswarm/scripts` and start the MPC using `python3 run_UAV_exp.py`. Flight data will be collected in `~/crazyswarm/ros_ws/src/crazyswarm/scripts/MPC_take2/data/node_data.csv` and the data can be visualized by running `python3 plots.py`.

## Demo
In order to run a demo of the landing use the `live_plot.py` file located in [Demo](./Demo/). Before starting the demo ensure the vehicles are placed in the MoCap space, and preliminary goals are initialized as per the instructions above for the Crazyflie setup and the Husky setup instructions found at https://github.com/Robora-Lab/husky_landing. Then the `live_plot.py` file can be run on any computer that is connected to the multi-master-setup (the computer requires access to the position and goal topics).

## Modified Files
`~/crazyswarm/ros_ws/src/pubCF.py`

Creates new node PubCF. Publishes Crazyflie goal to topic */goalPoseCF*. Subscribes to Husky goal topic */goalPose*. Runs at 10Hz.

`~/crazyswarm/ros_ws/src/cf_sub_pub/scripts/cf_sub_pub_node.py`

Creates new node *cf_sub_pub_node*. Subscibes to all output data from vrpn and Crazyflie and stores in csv `node_data.csv` located in `~/crazyswarm/ros_ws/src/crazyswarm/scripts/MPC_take2/data`. Controls coordination of time between the vehicles by periodically publishing to topic */systemTime*.


/crazyswarm/ros_ws/src/crazyswarm/externalDependencies/libmotioncapture/src/vicon.cpp

/crazyswarm/crazyflie-firmware/src/modules/src/crtp_commander_rpyt.c


## Topics
