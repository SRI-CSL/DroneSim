#!/usr/bin/env bash

apt-get update
apt-get upgrade
apt-get install -y emacs24 dbus-x11
apt-get install -y git python-pip

pip install pymavlink MAVProxy


#java 8
sudo apt-get install -y software-properties-common python-software-properties
echo oracle-java8-installer shared/accepted-oracle-license-v1-1 select true | sudo /usr/bin/debconf-set-selections
sudo apt-add-repository ppa:webupd8team/java -y
sudo apt-get update
sudo apt-get install oracle-java8-installer
echo "Setting environment variables for Java 8.."
sudo apt-get install -y oracle-java8-set-default


#plambda
pip install antlr4-python2-runtime
git clone https://github.com/SRI-CSL/PLambda
cd PLambda
make develop
cd ..

#iop
export IOPBINDIR=/home/vagrant/bin/IOP
mkdir -p /home/vagrant/bin/IOP
git clone https://github.com/SRI-CSL/iopc
cd iopc
make
make install
cd ..


#sitl et al
apt-get install  -y  python-matplotlib python-serial python-wxgtk2.8 python-lxml
apt-get install  -y  python-scipy python-opencv ccache gawk python-pexpect

pip install pymavlink MAVProxy
sudo apt-get install -y libtool automake autoconf libexpat1-dev

export PATH=$PATH:/home/vagrant/jsbsim/src:/home/vagrant/ardupilot/Tools/autotest:/usr/lib/ccache

git clone git://github.com/ArduPilot/ardupilot.git
git clone git://github.com/tridge/jsbsim.git
cd jsbsim
./autogen.sh --enable-libraries
make
cd ..
cd ardupilot/ArduPlane
sim_vehicle.sh -w
#sim_vehicle.sh --console --map --aircraft test

