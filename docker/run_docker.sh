#!/bin/bash

# Based on the ETH Robotics Summer school docker: 
# https://github.com/ETHZ-RobotX/smb_docker/

# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.

# Default options
USER=gem
DOCKER=nvblox
DOCKERFILE=Dockerfile
NAME=nvblox
BUILD=false
WORKSPACE=/home/$USER/nvblox_ws

help()
{
    echo "Usage: run_docker.sh 
               [ -b | --build ] [ -n | --name <docker name> ]
               [ -h | --help  ] [ -w | --workspace </workspace/path> ]"
    exit 2
}

SHORT=b,n:,w:,h
LONG=build,name:,workspace:,help
OPTS=$(getopt -a -n run_docker --options $SHORT --longoptions $LONG -- "$@")
echo $OPTS

eval set -- "$OPTS"

while :
do
  case "$1" in
    -b | --build )
      BUILD="true"
      shift
      ;;
    -n | --name )
      NAME="$2"
      shift 2
      ;;
    -w | --workspace )
      WORKSPACE="$2"
      shift 2
      ;;
    -h | --help)
      help
      ;;
    --)
      shift;
      break
      ;;
    *)
      echo "Unexpected option: $1"
      help
      ;;
  esac
done


if [ "$BUILD" = true ]; then
    echo "Building docker: $DOCKERFILE as $DOCKER"
    docker build -f $DOCKERFILE -t $DOCKER .
fi

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ -n "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

docker run -it --rm\
    --runtime=nvidia --gpus all\
    --env="DISPLAY=$DISPLAY" \
    --env="FRANKA_IP=$FRANKA_IP" \
    --volume=$WORKSPACE:/root/nvblox_ws \
    --volume=/home/$USER/data:/root/data \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    --name=$NAME \
    ${DOCKER} \
    bash

echo "Done."

# source ~/nvblox_ws/devel/setup.bash
# sudo docker exec -it nvblox bash

# rosrun imu_filter_madgwick imu_filter_node _use_mag:=false _publish_tf:=false _world_frame:="enu" /imu/data_raw:=/camera/imu
# roslaunch realsense2_camera rs_camera.launch align_depth:=true filters:=pointcloud enable_gyro:=true enable_accel:=true unite_imu_method:=linear_interpolation

# [[-9.67663948e-01  4.98994368e-02 -2.47258021e-01  2.87836373e-02]
#  [ 2.52242798e-01  1.92327581e-01 -9.48358409e-01 -3.99214594e-02]
#  [ 2.31986706e-04 -9.80061298e-01 -1.98695240e-01 -1.74024667e-02]
#  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

# source ~/catkin_ws/devel/setup.bash
# roslaunch natnet_ros_cpp natnet_ros.launch
# roslaunch realsense2_camera rs_camera.launch align_depth:=true depth_width:=640 depth_height:=480 depth_fps:=30 color_width:=640 color_height:=480 color_fps:=30
# rosbag record /camera/color/image_raw /camera/color/camera_info /camera/aligned_depth_to_color/image_raw /camera/aligned_depth_to_color/camera_info /tf /tf_static
# ~/nvblox_ws/src/nvblox_ros1/nvblox_ros/launch$ roslaunch optitrack.launch
