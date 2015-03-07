# pi2go_ros
ROS driver of the Pi2Go Robot

You can find about this awesome robot here: http://pi2go.co.uk/

## Software (Raspberry Pi)
#### Important: This guide will help you to install ROS on your Raspberry Pi board.

The software stack is built on
[Raspbian](http://www.raspbian.org/) (Debian Wheezy for Raspberry Pi)
[ROS Hydro](http://wiki.ros.org/hydro)

## Raspbian installation

### Download the latest premade image from [here](http://downloads.raspberrypi.org/raspbian_latest) and extract it.
### Write the image to your sd card
#### Linux
On a Linux system, use the dd command to write the downloaded image to whichever device is you SD card. Typing ```mount``` or ```ls -l /dev/disk/by-id``` will show you the devices and where they are mapped.
```
dd if=[path_to_image] of=[path_to_device]
```
For example:
```
dd if=/home/user/raspbian.img of=/dev/sdc
```

#### Windows
Download a tool called Rawrite32 and use it to write the image to your SD card.
http://www.netbsd.org/~martin/rawrite32/download.html

When you have succesfully written the data to the SD-card, then connect it to your Raspberry Pi and boot it.

## Connecting to Pi and first run

It is encouraged to connect to your Raspberry Pi using internet connection (for example via ethernet cable). Connect your Pi to your local network and scan the local area for the IP address with the following command:
```
nmap -sP 192.168.1.1/24
```
Note that your local IP address might be different, so test also 192.168.2.1 etc. The best guess is achieved by checking your development PCs own local IP address.

Then connect to your Pi with ssh connection (Putty on Windows):
```
ssh pi@youripaddress
```
where default password is "raspberry". You should change this later.

When you have connected to your Raspberry Pi, you can adjust your installation with configuration tool which is represented during the welcome message. You can do whatever you want, but the preferred tasks are 1) expanding your SD card installation size to the maximum 2) changing your password 3) overclocking your CPU to somewhere mid-performance (high performance could lead to severe crashes). However, when overclocking, remember that it is always a risk, but encouraged here to increase performance in several taskes.

After that, you can update your system to the latest version, kernel and firmware with following commands:
```
sudo apt-get update
sudo apt-get dist-upgrade
sudo rpi-update
```

## Install ROS Hydro

When you have configured your Raspberry Pi, it is time for the actual ROS installation. Be aware, that this installation requires hours to compile, so take your time.

First, setup the repositories.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu raring main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade
```

Bootstrap dependencies
```
sudo apt-get install python-setuptools
sudo apt-get install python-stdeb
sudo easy_install pip
sudo pip install -U rosdep rosinstall_generator wstool rosinstall
```

Ǹext, the source downloading and compiling, which requires some time.

```
mkdir ~/catkin_ws
cd ~/catkin_ws
rosinstall_generator ros_comm --rosdistro hydro --deps --wet-only > hydro-ros_comm-wet.rosinstall
wstool init -j8 src hydro-ros_comm-wet.rosinstall
sudo rosdep init
rosdep update
rosdep install  --from-paths src --ignore-src --rosdistro hydro -y --os=debian:wheezy
```

If ```wstool init``` fails or is interrupted, you can continue from previous state with

```
wstool update -j 4 -t src
```

IMPORTANT: When runnind rosdep install-call, this automatically provides all the dependencies from debian repositories. However, as seen above, we installed some of the dependencies with pip, not with apt. rosdep cannot detect these even though they are installed, so it can provide an error. Thse packages are most probably titles with ```python-*``` name, To avoid this and manually check which packages must be installed, run instead

```
rosdep check  --from-paths src --ignore-src --rosdistro hydro -y --os=debian:wheezy
```

and then install manually all missing packages (non python-*) with apt

```
sudo apt-get install [package-name]
```

Our software might require some tweaking with dependencies, which are not provided by Raspbian stable branch. So we should install them harder way.


####sbcl
rosdep might provide an error regarded to the sbcl which is not provided to Pi, so we have to remove roslisp.
```
cd src
wstool rm roslisp
rm -rf roslisp
cd ..
rosdep install  --from-paths src --ignore-src --rosdistro hydro -y --os=debian:wheezy
```

For the rest, make an external folder, which contains all these "extra" packages.

```
mkdir ~/catkin_ws/external_src
sudo apt-get install checkinstall cmake
```

####libconsole-bridge-dev
```
cd ~/catkin_ws/external_src
sudo apt-get install libboost-system-dev libboost-thread-dev
git clone https://github.com/ros/console_bridge.git
cd console_bridge
cmake .
sudo checkinstall make install
```

Remember to change the file name to ```libconsole-bridge-dev``` when asked to ensure package manager functionality.

####liblz4-dev
```
cd ~/catkin_ws/external_src
wget http://archive.raspbian.org/raspbian/pool/main/l/lz4/liblz4-1_0.0~r122-2_armhf.deb
wget http://archive.raspbian.org/raspbian/pool/main/l/lz4/liblz4-dev_0.0~r122-2_armhf.deb
sudo dpkg -i liblz4-1_0.0~r122-2_armhf.deb liblz4-dev_0.0~r122-2_armhf.deb
```

####rosbag_migration_rule
```
cd ~/catkin_ws/src
git clone https://github.com/ros/rosbag_migration_rule.git
cd ..
rosdep install  --from-paths src --ignore-src --rosdistro hydro -y --os=debian:wheezy
```

If everything went as expected, it is time to install your fresh ROS.

```
catkin_make_isolated --install
```

This is the point where you can take a nap.

When this step is over, you should add your setup.bash to your .bashrc to improve useability.

```
echo "source ~/catkin_ws/install_isolated/setup.bash" >> /home/pi/.bashrc
source ../.bashrc
```

## Additional packages

We want to install some own stuff, because "normal" ROS installation won't help us a lot. To install a new package from git repository, you should do the following:

```
cd ~/catkin_ws/src
git clone https://address.to.the.git
cd ..
rosdep check  --from-paths src --ignore-src --rosdistro hydro -y --os=debian:wheezy
```

or "rosdep install" if you sure that your dependencies are ok. Install the dependencies as in previous steps and after that. Some of the depencies might be also ROS packages, so check their Git repository from ROS Wiki (just google and you can find it) and clone the right branch, in this case is hydro: ```git clone -b brach_name https://address.to.the.git```

Then buiĺd the whole thing.

```
catkin_make_isolated --install
```
