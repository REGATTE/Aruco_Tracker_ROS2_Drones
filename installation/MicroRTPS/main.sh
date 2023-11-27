#! /bin/sh
cd
cd

echo " Installing "
{
sudo apt install curl
sudo apt install git
} > /dev/null 2>&1

if [[ $(lsb_release -rs) == "20.04" ]]; then
# install pixhawk
    bash installation/PX4.sh

    cd
    cd
# install FastRTPS_DDS.sh
    bash installation/FastRTPS_DDS.sh

    cd
    cd
# install ROS2_FOXY
    bash installation/ros2_foxy.sh
    cd
    cd
    bash installation/PX4_WS.sh

else
    echo "Non-compatible version"
    echo "Only use Ubuntu 20.04"
fi 
