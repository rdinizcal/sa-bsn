#!/bin/bash
x=1
cycles=1

while [ $x -le $cycles ]
do
bsn=$PWD

gnome-terminal -x roscore & sleep 5s

################# KNOWLEDGE REPOSITORY #################
gnome-terminal --working-directory=${bsn}/configurations/knowledge_repository -e 'roslaunch --pid=/var/tmp/data_access.pid data_access.launch' & sleep 1s
################# MANAGER #################
gnome-terminal --working-directory=${bsn}/configurations/system_manager -e 'roslaunch --pid=/var/tmp/enactor.pid enactor.launch'

################# LOGGING INFRASTRUCTURE #################
gnome-terminal --working-directory=${bsn}/configurations/logging_infrastructure -e 'roslaunch --pid=/var/tmp/logger.pid logger.launch' & sleep 1s

################# REPOSITORY #################
#gnome-terminal --working-directory=${bsn}/configurations/repository -e 'roslaunch data_access_node.launch'
#gnome-terminal --working-directory=${bsn}/configurations/repository -e 'roslaunch illness_identifier.launch'

################# APPLICATION #################
gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch --pid=/var/tmp/probe.pid probe.launch' & sleep 1s
gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch --pid=/var/tmp/effector.pid effector.launch' & sleep 1s

gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch --pid=/var/tmp/g4t1.pid g4t1.launch' & sleep 5s
gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch --pid=/var/tmp/g3t1_1.pid g3t1_1.launch'
gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch --pid=/var/tmp/g3t1_2.pid g3t1_2.launch'
gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch --pid=/var/tmp/g3t1_3.pid g3t1_3.launch'
gnome-terminal --working-directory=${bsn}/configurations/environment   -e 'roslaunch --pid=/var/tmp/patient.pid patient.launch'

#gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_4.launch'

################# SIMULATION #################
gnome-terminal --working-directory=${bsn}/configurations/simulation -e 'roslaunch --pid=/var/tmp/analyzer.pid analyzer.launch' & sleep 1s

gnome-terminal --working-directory=${bsn}/configurations/simulation -e 'roslaunch --pid=/var/tmp/injector.pid injector.launch'

gnome-terminal --working-directory=${bsn}/configurations/system_manager -e 'roslaunch --pid=/var/tmp/engine.pid engine.launch' & sleep 300s

kill $(cat /var/tmp/data_access.pid && rm /var/tmp/data_access.pid) & sleep 1s
kill $(cat /var/tmp/enactor.pid && rm /var/tmp/enactor.pid) & sleep 1s
kill $(cat /var/tmp/logger.pid && rm /var/tmp/logger.pid) & sleep 1s
kill $(cat /var/tmp/probe.pid && rm /var/tmp/probe.pid) & sleep 1s
kill $(cat /var/tmp/effector.pid && rm /var/tmp/effector.pid) & sleep 1s
kill $(cat /var/tmp/g4t1.pid && rm /var/tmp/g4t1.pid) & sleep 1s
kill $(cat /var/tmp/g3t1_1.pid && rm /var/tmp/g3t1_1.pid) & sleep 1s
kill $(cat /var/tmp/g3t1_2.pid && rm /var/tmp/g3t1_2.pid) & sleep 1s
kill $(cat /var/tmp/g3t1_3.pid && rm /var/tmp/g3t1_3.pid) & sleep 1s
kill $(cat /var/tmp/patient.pid && rm /var/tmp/patient.pid) & sleep 1s
kill $(cat /var/tmp/analyzer.pid && rm /var/tmp/analyzer.pid) & sleep 1s
kill $(cat /var/tmp/injector.pid && rm /var/tmp/injector.pid) & sleep 1s
kill $(cat /var/tmp/engine.pid && rm /var/tmp/engine.pid) & sleep 1s

kill $(pgrep roscore)

sleep 15s

x=$(( x + 1 ))

done