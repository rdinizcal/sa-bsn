bsn=$PWD

gnome-terminal -x roscore & sleep 3s

gnome-terminal --working-directory=${bsn}/launch -e 'roslaunch centralhub.launch'
gnome-terminal --working-directory=${bsn}/launch -e 'roslaunch thermometer.launch'
gnome-terminal --working-directory=${bsn}/launch -e 'roslaunch ecg.launch'
gnome-terminal --working-directory=${bsn}/launch -e 'roslaunch oximeter.launch'
gnome-terminal --working-directory=${bsn}/launch -e 'roslaunch bloodpressure.launch'