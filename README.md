# Self-Adaptive Body Sensor Network (SA-BSN)

This is an implementation of the SA-BSN. So far, the SA-BSN was used for experimentation on solutions for adaptation on the Self-Adaptive Software Systems domain [[1]](https://doi.org/10.1145/3194133.3194147)[[2]](https://doi.org/10.1109/SEAMS.2019.00020)[[3]](https://doi.org/10.1145/3387939.3391595). Moreover, information regarding the prototype behavior and how to develop your own manager is provided in the [website](https://bodysensornetwork.herokuapp.com/), which contains an executable instance of the BSN. 

Check our [demonstration video of the SA-BSN](https://youtu.be/iDEd_tW9JZE).

Download our [SA-BSN Virtual Machine (Virtual Box) for People in a Hurry](https://drive.google.com/file/d/18APQNBNZM7Pp_bFGyuChOH1uivf4YEV1/view?usp=sharing). 
(_~10 minutes!!_)

To compile and run the SA-BSN, follow the instructions (tested on Linux Ubuntu 18.04 with ROS Melodic): 

## Detailed instructions to build, execute and analyze the SA-BSN

### Build the SA-BSN

_Jump this step if you are in our provided VM_

#### **Dependencies**

ROS Melodic is the underlying framework in which the SA-BSN runs. We suggest the installation of the [ROS Melodic for Ubuntu 18.04](http://wiki.ros.org/melodic/Installation/Ubuntu). Also, we suggest [Catkin](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) for ROS packages management. 

#### **Create workspace, clone and build**

Once ROS Melodic and catkin are properly installed, you need to create and initialize the catkin workspace:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

Then, inside the catkin/src folder, clone the sa-bsn.

```
cd ~/catkin_ws/src
git clone https://github.com/rdinizcal/sa-bsn.git
```

When cloned, enter the sa-bsn folder and then compile the SA-BSN:

```
cd ~/catkin_ws
catkin_make
```

The skilled user can rely on the catkin commands to compile the SA-BSN.

### Execute the SA-BSN

The SA-BSN's execution rely on a single command, where the argument (i.e., 600) represents the amount of seconds of execution:

```
cd ~/catkin_ws/src/sa-bsn/ && 
bash run.sh 600
``` 

For a customized execution, check the configuration files under sa-bsn/configurations.

### Analyze the SA-BSN

During the execution a logging mechanism records data in logfiles, that can be found in sa-bsn/src/knowledge_repository/resource/logs. Each logfile is named after a type of message and an id (i.e., logName_logID) and the entries are composed by the messages content.

To compute the evolution of the QoS attributes (i.e., reliability and cost) over time and how well the adaptation manager adapts the system, we use a python script that reads the logs as input and plots a timeseries + displays adaptation metrics as output.

The python script can be found at sa-bsn/src/simulation/analyzer.

To compute the QoS attributes and analyze the adaptation, type:

```
cd ~/catkin_ws/src/sa-bsn/src/simulation/analyzer/
python3 analyzer.py [logID] [metric] [plot_component_metrics] [setpoint]
```

where:

* [logID] is the ID for the execution log files.
* [metric] is the name of the QoS attribute (e.g., reliability)
* [plot_component_metrics] is a boolean parameter that defines whether individual component's QoS attribute should be plotted or not.
* [setpoint] is the value of the setpoint used in the execution.

One example of the command usage is:

```
python3 analyzer.py 1610549979516318295 reliability False 0.9
```

## Common Mistakes

### In case of error due to the ROS path

You might want to source the setup.bash inside the catkin workspace:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Main Authors

* [**Ricardo Caldas**](https://github.com/rdinizcal)
* [**Gabriel Levi**](https://github.com/gabrielevi10)
* [**Léo Moraes**](https://github.com/leooleo)  
* [**Eric B. Gil**](https://github.com/ericbg27)

Adviser: [**Genaína N. Rodrigues**](https://cic.unb.br/~genaina)