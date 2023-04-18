default:
	echo Waiter_Ros

.PHONY: install
install: install-dynamixelsdk install-odrive install-rplidar install-laser-filters install-diff-drive install-gmapping catkin_make

.PHONY: install-dynamixelsdk 
install-dynamixelsdk:
	cd ~/catkin_ws/src && \
	gh repo clone ROBOTIS-GIT/DynamixelSDK

	cd ~/catkin_ws/src/DynamixelSDK && \
	git checkout noetic-devel

	cd ~/catkin_ws/src/DynamixelSDK/python && \
	sudo python3 setup.py install

.PHONY: install-odrive 
install-odrive:
	sudo apt install python3 python3-pip
	pip3 install --upgrade odrive
	sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"
	echo 'export PATH=$PATH:~/.local/bin' >> ~/.bashrc

.PHONY: install-rplidar
install-rplidar:
	cd ~/catkin_ws/src && \
	gh repo clone Slamtec/rplidar_ros

.PHONY: install-laser-filters
install-laser-filters:
	cd ~/catkin_ws/src && \
	gh repo clone ros-perception/laser_filters && \
	cd ~/catkin_ws/src/laser_filters && \
	git checkout noetic-devel
	
.PHONY: install-diff-drive
install-diff-drive:
	cd ~/catkin_ws/src && \
	gh repo clone jfstepha/differential-drive

.PHONY: install-gmapping
install-gmapping:
	sudo apt-get install ros-noetic-slam-gmapping

.PHONY: catkin_make
catkin_make:
	cd ~/catkin_ws && catkin_make

.PHONY: give-permission
give-permission:
	make -C scripts give-permission

.PHONY: move-robot-description
move-robot-description:
	cp -R robot_description $$HOME/catkin_ws/src
