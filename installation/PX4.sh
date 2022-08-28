#! /bin/sh
cd
cd

echo "Installing PX4 Autopilot"
cd Desktop
# Clone only forked repo
git clone https://github.com/UAV-TECH/PX4-Autopilot.git --recursive > /dev/null 2>&1
# installs Java JDK 11
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh > /dev/null 2>&1
echo " Done Installing PX4 Autopilot"

echo "Build px4 fmu-v5x"
# build px4 firmware for FMU_v5x
# this builds a new BUILD folder in PX4-Autopilot root directory and stores the firmware there.
cd PX4-Autopilot
make px4_fmu-v5x_default > /dev/null 2>&1
echo "Done Building px4 fmu-v5x"
# make px4_fmu-v5x_default

cd
cd
