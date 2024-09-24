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


## Topics

## Modified Files
`~/crazyswarm/ros_ws/src/pubCF.py`

Creates new node PubCF. Publishes Crazyflie goal to topic */goalPoseCF*. Subscribes to Husky goal topic */goalPose*. Runs at 10Hz.

`~/crazyswarm/ros_ws/src/cf_sub_pub/scripts/cf_sub_pub_node.py`

Creates new node *cf_sub_pub_node*. Subscibes to all output data from vrpn and Crazyflie and stores in csv `node_data.csv` located in `~/crazyswarm/ros_ws/src/crazyswarm/scripts/MPC_take2/data`. Controls coordination of time between the vehicles by periodically publishing to topic */systemTime*.


/crazyswarm/ros_ws/src/crazyswarm/externalDependencies/libmotioncapture/src/vicon.cpp

/crazyswarm/crazyflie-firmware/src/modules/src/crtp_commander_rpyt.c