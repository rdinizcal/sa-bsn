# BSN implementation on ROS

This is a Body Sensor Network implementation on ROS. So far, the BSN was used for experimentation on solutions for adaptation on the Self-Adaptive Software Systems domain, refer to https://arxiv.org/pdf/1804.00994.pdf and https://arxiv.org/pdf/1905.02228.pdf for more information.  The following instructions will guide you to to compile, deploy and run the BSN on Linux Ubuntu 18.04 with ROS Melodic distributions. We have not yet tested on other distributions. Also, it is strongly advised to use catkin for managing the ROS packages, refer to http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment after installing ROS Melodic.

## Dependencies:
* https://github.com/rdinizcal/lepton
* https://github.com/rdinizcal/libbsn
* https://github.com/Microsoft/cpprestsdk
* https://github.com/arch

Once ALL the dependencies have been successfully installed,

## Compiling

1. Clone the repository inside the 'catkin_ws/src' directory:
```
cd ~/catkin_ws/src/ && 
git clone https://github.com/rdinizcal/bsn_ros
``` 

2. Then, compile under 'catkin_ws' directory:
```
cd ~/catkin_ws/ && 
catkin_make
``` 
## Configuration and Execution

3. Configure roslaunch files for personalized execution under '/catkin_ws/src/bsn_ros/configurations';

4. Execute the BSN either by executing the pre-set run.sh file, that executes all nodes:
```
cd ~/catkin_ws/src/bsn_ros/ && 
bash run.sh
``` 
or use roslaunch x.launch to execute a single node.

## Authors

* **Ricardo D. Caldas** - https://github.com/rdinizcal
* **Eric B. Gil** - https://github.com/ericbg27/
* **Gabriel Levi** - https://github.com/gabrielevi10
* **Léo Moraes** - https://github.com/leooleo 
* **Samuel Couto** - https://github.com/SCouto97
* **Jorge Mendes** - https://github.com/luzmendesj 

Adviser: **Dr. Genaína Nunes Rodrigues** - https://cic.unb.br/~genaina/
