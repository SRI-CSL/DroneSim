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
sudo apt-get install -y pkg-config
sudo apt-get install -y zip



echo "Installing Oracle's Java 8"

echo oracle-java8-installer shared/accepted-oracle-license-v1-1 select true | sudo /usr/bin/debconf-set-selections
sudo apt-add-repository ppa:webupd8team/java -y
sudo apt-get update
sudo apt-get install -y oracle-java8-installer

echo "Setting environment variables for Java 8"

sudo apt-get install -y oracle-java8-set-default


echo "Taking ownership of /usr/local"

sudo chown -R vagrant:staff /usr/local/

echo "Silencing libdc1394 complaints."

sudo ln /dev/null /dev/raw1394

mkdir -p /home/vagrant/Repositories

echo "Installing Maude"
cd /home/vagrant/Repositories
mkdir -p /home/vagrant/bin/Maude
wget http://maude.cs.illinois.edu/w/images/5/5d/Maude-2.7.1-linux.zip
unzip Maude-2.7.1-linux.zip -d /home/vagrant/bin/Maude
chmod a+x /home/vagrant/bin/Maude/maude.linux64
ln -s /home/vagrant/bin/Maude/maude.linux64 /home/vagrant/bin/Maude/maude
rm Maude-2.7.1-linux.zip


echo "Installing PLambda"
cd /home/vagrant/Repositories
pip install antlr4-python2-runtime
git clone https://github.com/SRI-CSL/PLambda
cd PLambda
make develop



echo "Installing IOP"
cd /home/vagrant/Repositories
export IOPBINDIR=/home/vagrant/bin/IOP
mkdir -p /home/vagrant/bin/IOP
git clone https://github.com/SRI-CSL/iopc
cd iopc
make
make install

echo "Installing IMaude"
cd /home/vagrant/Repositories
git clone https://github.com/SRI-CSL/imaude


echo "Installing SITL"

sudo apt-get install  -y  python-matplotlib python-serial python-lxml
sudo apt-get install  -y  python-scipy python-opencv ccache gawk python-pexpect libpython-all-dev
sudo apt-get install  -y  python-dev python-numpy python-opencv python-pyparsing python-wxgtk2.8


echo "Installing dronekit-sitl"

cd /home/vagrant/Repositories
git clone https://github.com/dronekit/dronekit-sitl.git
cd dronekit-sit
pip install -e .

#
# 12/13/2016 the pip package is too old, and the API is quite different.
#
#echo "Installing dronekit"
#
#pip install dronekit-sitl -UI

echo "Installing MAVProxy"

pip install pymavlink MAVProxy

echo "Cloning jsbsim"
cd /home/vagrant/Repositories
git clone git://github.com/tridge/jsbsim.git

echo "Building jsbsim"

cd jsbsim
./autogen.sh --enable-libraries
make


#
# ardupilot seems to be in a state of motion
# they are moving their build system from "make" to "waf" and
# changing their command line arguments.
#
# so from here on, you may have to do this stuff by hand if it fails.
#
#
echo "Cloning ardupilot"

pip install future

cd /home/vagrant/Repositories
git clone git://github.com/ArduPilot/ardupilot.git


echo "Prepping the ardupilot build"

export PATH=${PATH}:/home/vagrant/jsbsim/src:/home/vagrant/ardupilot/Tools/autotest:/usr/lib/ccache
echo '. /vagrant/bash_profile' >> /home/vagrant/.bashrc


echo "Configuring ardupilot"
cd ardupilot
# readme says run once. but we have to run twice.
./waf configure --board minlure
./waf configure --board minlure

echo "Building arducopter"
./waf copter

#
# plane is not yet ready for prime time?
#
#  12/13/2016: It could be now that we can point to a binary
#
# https://github.com/dronekit/dronekit-sitl/issues/79
#
#echo "Building arduplane"
#./waf plane


echo "Time to build/testing ardupilot"
#
# Building bits:
#
#
#sim_vehicle.py -v ArduCopter
#sim_vehicle.py -v ArduPlane
#
#Once built you'll have to ^D to exit.
#Then should be able to launch via:
#
#sim_vehicle.py -v ArduCopter --map --console
