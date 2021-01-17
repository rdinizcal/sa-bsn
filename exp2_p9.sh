bsn=$PWD

gnome-terminal -e roscore & sleep 3s

gnome-terminal --working-directory=${bsn}/configurations/experiments/p3/target_system -e 'roslaunch probe.launch' & sleep 1s
gnome-terminal --working-directory=${bsn}/configurations/experiments/p3/target_system -e 'roslaunch effector.launch' & sleep 1s

gnome-terminal --working-directory=${bsn}/configurations/experiments/p3/target_system -e 'roslaunch g4t1.launch' & sleep 5s
gnome-terminal --working-directory=${bsn}/configurations/experiments/p3/target_system -e 'roslaunch g3t1_1.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p3/target_system -e 'roslaunch g3t1_2.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p3/target_system -e 'roslaunch g3t1_3.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p3/environment   -e 'roslaunch patient.launch'  & sleep 30s

rosnode kill -a

gnome-terminal -e roscore & sleep 3s

gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P3 -e 'roslaunch P3_oximeter_analyzer.launch'
gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P3 -e 'roslaunch P3_ecg_analyzer.launch'
gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P3 -e 'roslaunch P3_thermometer_analyzer.launch'

gnome-terminal --working-directory=${bsn}/configurations/experiments/p3/target_system -e 'roslaunch probe.launch' & sleep 1s
gnome-terminal --working-directory=${bsn}/configurations/experiments/p3/target_system -e 'roslaunch effector.launch' & sleep 1s

gnome-terminal --working-directory=${bsn}/configurations/experiments/p3/target_system -e 'roslaunch g4t1_noise.launch' & sleep 5s
gnome-terminal --working-directory=${bsn}/configurations/experiments/p3/target_system -e 'roslaunch g3t1_1_noise.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p3/target_system -e 'roslaunch g3t1_2_noise.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p3/target_system -e 'roslaunch g3t1_3_noise.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p3/environment   -e 'roslaunch patient.launch'  & sleep 30s

################# SIMULATION #################
gnome-terminal --working-directory=${bsn}/configurations/experiments/p3/simulation -e 'roslaunch injector.launch' & sleep 90s

rosnode kill -a
