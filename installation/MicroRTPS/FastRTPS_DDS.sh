#! /bin/sh
cd
cd

echo "----- Install Gradle -----"
# Gradle
curl -s "https://get.sdkman.io" | bash > /dev/null 2>&1
source "$HOME/.sdkman/bin/sdkman-init.sh"

sdk install gradle 6.3

echo "Done installing Gradle"
echo "cloning Fast RTPS Gen"
# Fast-RTPS-Gen
{
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen \
    && cd ~/Fast-RTPS-Gen \
    && gradle assemble \
    && sudo env "PATH=$PATH" gradle install
} > /dev/null 2>&1
echo "Done"
cd
cd