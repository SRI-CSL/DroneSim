#!/usr/bin/env bash

echo "Installing the basics"

sudo apt-get update
sudo apt-get upgrade
sudo apt-get install -y emacs24 dbus-x11
sudo apt-get install -y git python-pip
sudo apt-get install -y python-dev
sudo apt-get install -y ipython
sudo apt-get install -y libtool automake autoconf libexpat1-dev
sudo apt-get install -y software-properties-common python-software-properties



echo "Installing Oracle's Java 8"

echo oracle-java8-installer shared/accepted-oracle-license-v1-1 select true | sudo /usr/bin/debconf-set-selections
sudo apt-add-repository ppa:webupd8team/java -y
sudo apt-get update
sudo apt-get install -y oracle-java8-installer

echo "Setting environment variables for Java 8"

sudo apt-get install -y oracle-java8-set-default


echo "Taking ownership of /usr/local"

sudo chown -R vagrant:staff /usr/local/


echo "Installing PLambda"

pip install antlr4-python2-runtime
git clone https://github.com/SRI-CSL/PLambda
cd PLambda
make develop
cd ..

echo "Installing IOP"

export IOPBINDIR=/home/vagrant/bin/IOP
mkdir -p /home/vagrant/bin/IOP
git clone https://github.com/SRI-CSL/iopc
cd iopc
make
make install
cd ..

echo "Installing IMaude"
git clone https://github.com/SRI-CSL/imaude


echo "Installing SITL"

sudo apt-get install  -y  python-matplotlib python-serial python-lxml
sudo apt-get install  -y  python-scipy python-opencv ccache gawk python-pexpect libpython-all-dev
sudo apt-get install  -y  python-dev python-numpy python-opencv python-pyparsing python-wxgtk2.8


echo "Installing dronekit"

pip install dronekit-sitl -UI

echo "Installing MAVProxy"

pip install pymavlink MAVProxy

echo "Cloning jsbsim"

git clone git://github.com/tridge/jsbsim.git

echo "Cloning ardupilot"

git clone git://github.com/ArduPilot/ardupilot.git


echo "Prepping the jsbsim build"

export PATH=${PATH}:/home/vagrant/jsbsim/src:/home/vagrant/ardupilot/Tools/autotest:/usr/lib/ccache
echo '. /vagrant/bash_profile' >> /home/vagrant/.bashrc


echo "Building jsbsim"

cd jsbsim
./autogen.sh --enable-libraries
make
cd ..


#
# ardupilot seems to be in a state of motion
# they are moving their build system from "make" to "waf" and
# changing their command line arguments.
#
# so from here on, you may have to do this stuff by hand if it fails.
#
#

echo "Configuring ardupilot"
cd ardupilot
# readme says run once. but we have to run twice.
./waf configure --board minlure
./waf configure --board minlure

echo "Building ardupilot"
./waf copter


echo "Time to build/testing ardupilot"
#
# Building bits:
#
#
#sim_vehicle.py -v ArduCopter
#
#Once built you should be able to launch via:
#(may fail the first time, usually works the second time)
#
#sim_vehicle.py -v ArduCopter --map --console


