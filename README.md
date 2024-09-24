# Crazyswarm Landing
Modified Crazyswarm for UAV-USV landing in MoCap space.

## Usage
1. Follow instructions found at https://crazyswarm.readthedocs.io/en/latest/installation.html to clone the crazyflie repository and set up configurations in `~/crazyswarm/ros_ws/src/crazyswarm/launch/allCrazyflies.yaml` and `~/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml` and `~/crazyswarm/ros_ws/src/crazyswarm/launch/hover_swarm.launch`. The recommended setup for these files is included in the folder [launch](./launch).

2. 

## Topics

## Modified Files
`~/crazyswarm/ros_ws/src/pubCF.py`

Creates new node PubCF. Publishes Crazyflie goal to topic */goalPoseCF*. Subscribes to Husky goal topic */goalPose*. Runs at 10Hz.

`~/crazyswarm/ros_ws/src/cf_sub_pub/scripts/cf_sub_pub_node.py`

Creates new node *cf_sub_pub_node*. Subscibes to all output data from vrpn and Crazyflie and stores in csv `node_data.csv` located in `~/crazyswarm/ros_ws/src/crazyswarm/scripts/MPC_take2/data`. Controls coordination of time between the vehicles by periodically publishing to topic */systemTime*.

`~/