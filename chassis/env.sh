source /opt/ros/humble/setup.bash
for s in interface driver move navigation server steward slam; do source ~/robot/bin/$s/install/setup.bash 2>/dev/null; done
export ROS_DOMAIN_ID=24 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///etc/ros/cyclonedds/cyclonedds.xml
