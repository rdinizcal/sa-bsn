bsn=$PWD

gnome-terminal -x roscore & sleep 10s

gnome-terminal --working-directory=${bsn}/configurations/logging -e 'roslaunch logger.launch'

gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch g4t1.launch'
gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch g3t1_1.launch'
gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch g3t1_2.launch'
gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch g3t1_3.launch'
gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch g3t1_4.launch'
#gnome-terminal --working-directory=${bsn}/configurations -e 'roslaunch bloodpressure.launch'