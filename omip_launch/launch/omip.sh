#!/bin/bash

blue='\033[0;34m'
red='\033[0;31m'
NC='\033[0m' # No Color

kill_child_processes() {
    isTopmost=$1
    curPid=$2
    childPids=`ps -o pid --no-headers --ppid ${curPid}`
    for childPid in $childPids
    do
        kill_child_processes 0 $childPid
    done
    if [ $isTopmost -eq 0 ]; then
        kill -9 $curPid 2> /dev/null
    fi
}

# Ctrl-C trap. Catches INT signal
trap "kill_child_processes 1 $$; stty sane; exit 0" INT

#This is just in case we want to define something specific for the console messages
#export ROSCONSOLE_CONFIG_FILE=`rospack find omip_launch`/cfg/rosconsole.config

RGBDSENSOR=1
PAUSEBETWEENNODES=1
ROSWINBRIDGE=0
OMIP=1
WAITKEY=0
RVIZ=1

while test $# -gt 0; do
  case "$1" in
    -h|--help)
      echo "Script to start Online Iteractive Perception. It initializes on-demand the RGB-D sensor and the openni node, the self-occlusion filter, the ros-win-bridge and the omip node."
      echo "Options:"
      echo -e "\t--omip= -> Launch Online Multimodal Interactive Perception (default ${OMIP}):"
      echo -e "\t\t0 -> Do not launch Online Multimodal Interactive Perception"
      echo -e "\t\t1 -> Launch Online Multimodal Interactive Perception"
      echo -e "\t\t2 -> Launch Online Multimodal Interactive Perception and keep the terminals open"
      echo -e "\t\t3 -> Launch Online Multimodal Interactive Perception with shape reconstruction"
      echo -e "\t\t4 -> Launch Online Multimodal Interactive Perception with shape reconstruction and shape tracker"
      echo -e "\t--rgbd= -> Launch openni node for RGB-D sensor (default ${RGBDSENSOR}):"
      echo -e "\t\t0 -> Do not launch the openni node"
      echo -e "\t\t1 -> Do not launch the openni node but launch the static transformations"
      echo -e "\t\t2 -> Launch the openni2 node for a standard RGBD sensor"
      echo -e "\t--rwb= -> Launch the ros_win_bridge (default ${ROSWINBRIDGE}):"
      echo -e "\t\t0 -> Do not launch the ros_win_bridge"
      echo -e "\t\t1 -> Launch the ros_win_bridge to connect to bottom-1"
      echo -e "\t\t2 -> Launch the ros_win_bridge to connect to bottom-2"
      echo -e "\t\t3 -> Launch the ros_win_bridge to connect to bottom-3"
      echo -e "\t--rviz= -> Launch rviz (default ${RVIZ}):"
      echo -e "\t\t0 -> Do not launch rviz"
      echo -e "\t\t1 -> Launch rviz with config file for OMIP"
      echo -e "\t\t2 -> Launch rviz with config file for Shape Reconstruction"
      echo -e "\t-p=SECS -> Pause (secs) between launching nodes (default ${PAUSEBETWEENNODES})."
      exit 0
      ;;
    --omip*|-omip*)
      OMIP=`echo $1 | sed -e 's/^[^=]*=//g'`
      shift
      ;;
    --rgbd*|-rgbd*)
      RGBDSENSOR=`echo $1 | sed -e 's/^[^=]*=//g'`
      shift
      ;;
    --rwb*|-rwb*)
      ROSWINBRIDGE=`echo $1 | sed -e 's/^[^=]*=//g'`
      shift
      ;;
    --rviz*|-rviz*)
      RVIZ=`echo $1 | sed -e 's/^[^=]*=//g'`
      shift
      ;;
    -p*|--p*)
      PAUSEBETWEENNODES=`echo $1 | sed -e 's/^[^=]*=//g'`
      shift
      ;;
    *)
      echo -e "\e[1mNot recognizable option! $1\n\n\n"
      break
      ;;
  esac
done

#This is just in case you are using a distrubuted ROS with one master and you need to synchronize
#sudo /etc/init.d/chrony stop
#sudo ntpdate 192.168.0.1
#sudo /etc/init.d/chrony start

echo -e "\e[1mUser options:"
echo -e "\t\e[1mRGBDSENSOR=${RGBDSENSOR}"
echo -e "\t\e[1mROSWINBRIDGE=${ROSWINBRIDGE}"
echo -e "\t\e[1mRVIZ=${RVIZ}"
echo -e "\t\e[1mPAUSEBETWEENNODES=${PAUSEBETWEENNODES}"

case "$RGBDSENSOR" in
  0)
  echo -e "${blue}NO RGBD sensor! No static transformations!${NC}"
  WAITKEY=0
  ;;
  1)
  echo -e "${blue}NO RGBD sensor! Publishing static transformation for the camera_link to camera_rgb_optical_frame and from base link of the XR4000 to camera_link${NC}"
  roslaunch omip_launch asus_mounted_on_xr4000_only_tf.launch &
  WAITKEY=0
  ;;
  2)
  echo -e "${blue}Launch openni node for standard RGBD sensor on XR4000${NC}"
  roslaunch omip_launch asus_mounted_on_xr4000.launch &
  WAITKEY=1
  ;;
  *)
  ;;
esac

if [ ${WAITKEY} -eq 1 ]; then
  echo  -e "${blue}Press enter to continue (it continues automatically after ${PAUSEBETWEENNODES} seconds)${NC}"
  read -n 1 -s -t ${PAUSEBETWEENNODES}
fi

case "$ROSWINBRIDGE" in
  0)
  echo -e "${blue}NO ros_win_bridge!${NC}"
  WAITKEY=0
  ;;
  1)
  echo -e "${blue}Launch the ros_win_bridge to connect to bottom-1${NC}"
  roslaunch ros_win_bridge xr1.launch &
  WAITKEY=1
  ;;
  2)
  echo -e "${blue}Launch the ros_win_bridge to connect to bottom-2${NC}"
  roslaunch ros_win_bridge xr2.launch &
  WAITKEY=1
  ;;
  3)
  echo -e "${blue}Launch the ros_win_bridge to connect to bottom-3${NC}"
  roslaunch ros_win_bridge xr3.launch &
  WAITKEY=1
  ;;
  *)
  ;;
esac
if [ ${WAITKEY} -eq 1 ]; then
  echo  -e "${blue}Press enter to continue (it continues automatically after ${PAUSEBETWEENNODES} seconds)${NC}"
  read -n 1 -s -t ${PAUSEBETWEENNODES}
fi

#OMIP has to start before SR and ST because they need some parameters defined in OMIP
case "$OMIP" in
  0)
  echo -e "${blue}NO Online Multimodal Interactive Perception!${NC}"
  WAITKEY=0
  ;;
  1)
  echo -e "${blue}Launch Online Multimodal Interactive Perception${NC}"
  roslaunch omip_launch omip.launch &
  terminator --layout omip &
  WAITKEY=1
  ;;
  2)
  echo -e "${blue}Launch Online Multimodal Interactive Perception and keep the terminals open${NC}"
  roslaunch omip_launch omip.launch &
  terminator --layout omip_hold &
  WAITKEY=1
  ;;
  3)
  echo -e "${blue}Launch Online Multimodal Interactive Perception with Shape Reconstruction${NC}"
  roslaunch omip_launch omip.launch &
  terminator --layout omip_sr &
  WAITKEY=1
  ;;
  4)
  echo -e "${blue}Launch Online Multimodal Interactive Perception with Shape Reconstruction and Shape Tracking${NC}"
  roslaunch omip_launch omip.launch &
  terminator --layout omip_sr_st &
  WAITKEY=1
  ;;
  *)
  ;;
esac

case "$RVIZ" in
  0)
  echo -e "${blue}NO rviz!${NC}"
  WAITKEY=0
  ;;
  1)
  echo -e "${blue}Launch rviz for OMIP${NC}"
  rosrun rviz rviz -d $(rospack find omip_launch)/cfg/omip.rviz &
  WAITKEY=1
  ;;
  *)
  ;;
esac
if [ ${WAITKEY} -eq 1 ]; then
  echo  -e "${blue}Press enter to continue (it continues automatically after ${PAUSEBETWEENNODES} seconds)${NC}"
  read -n 1 -s -t ${PAUSEBETWEENNODES}
fi

for job in `jobs -p`
do
  wait $job 
done
