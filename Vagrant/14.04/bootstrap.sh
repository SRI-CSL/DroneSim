#!/usr/bin/env bash

sudo apt-get update
sudo apt-get upgrade
sudo apt-get install -y emacs24 dbus-x11
sudo apt-get install -y git python-pip

#java 8
sudo apt-get install -y software-properties-common python-software-properties
echo oracle-java8-installer shared/accepted-oracle-license-v1-1 select true | sudo /usr/bin/debconf-set-selections
sudo apt-add-repository ppa:webupd8team/java -y
sudo apt-get update
sudo apt-get install -y oracle-java8-installer
echo "Setting environment variables for Java 8.."
sudo apt-get install -y oracle-java8-set-default


#take ownership of /usr/local
sudo chown -R vagrant:staff /usr/local/


#plambda
sudo apt-get -y install ipython
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
sudo apt-get install  -y  python-matplotlib python-serial python-lxml
sudo apt-get install  -y  python-scipy python-opencv ccache gawk python-pexpect libpython-all-dev
sudo apt-get install  -y  python-dev python-numpy python-opencv python-pyparsing python-wxgtk2.8
pip install dronekit-sitl -UI



pip install pymavlink MAVProxy
sudo apt-get install -y libtool automake autoconf libexpat1-dev

#setup the path
export PATH=${PATH}:/home/vagrant/jsbsim/src:/home/vagrant/ardupilot/Tools/autotest:/usr/lib/ccache
echo '. /vagrant/bash_profile' >> ~/.bashrc

git clone git://github.com/ArduPilot/ardupilot.git
git clone git://github.com/tridge/jsbsim.git
cd jsbsim
./autogen.sh --enable-libraries
make
cd ..
cd ardupilot/ArduPlane
sim_vehicle.sh -w
#sim_vehicle.sh --console --map --aircraft test

