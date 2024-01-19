#!/bin/bash

#如果用户输入除了Y或者y之外的都会结束shell
InputYesOrExit()
{
   read -p "确认请输入y/n:" yn
   if [ "${yn}" != "Y" -a "${yn}" != "y" ] && [ "${yn}" != ""  ]; then
         echo -e "结束脚本" 
         exit 0
   fi    
}

pathConfigFile="/home/delljun/p2/3-slam/7-1-vins-fusion/1-ws-ori/src/VINS-Fusion-2401/config/euroc/euroc_stereo_imu_config.yaml"
pathDataset='/media/delljun/Bubby/data2/datasets/EuRoC/V1_03_difficult.bag'

# pathConfigFile="/home/delljun/p2/3-slam/7-1-vins-fusion/1-ws-ori/src/VINS-Fusion-2401/config/euroc/euroc_stereo_imu_config.yaml"
# pathDataset='/media/delljun/Bubby/data2/datasets/EuRoC/V2_03_difficult.bag'

echo "test vins by:$pathDataset"

time=$(date "+%Y%m%d_%H%M%S")

mkdir -p log/ori/$time
log1=log/ori/$time/log1_rviz_${time}.txt
log2=log/ori/$time/log2_vins_${time}.txt
log3=log/ori/$time/log3_loop_${time}.txt
touch $log1 $log2 $log3

# Step 1: 启动rviz界面
# gnome-terminal -- roslaunch vins vins_rviz.launch 2>&1 | tee -a log/$time/log1_rviz_${time}.txt

# InputYesOrExit

# gnome-terminal -- rosrun vins vins_node $pathConfigFile 2>&1 | tee -a log/$time/log3_vins_${time}.txt

# InputYesOrExit

# gnome-terminal -- rosrun loop_fusion loop_fusion_node $pathConfigFile 2>&1 | tee -a log/$time/log2_loop_${time}.txt

# InputYesOrExit

# gnome-terminal -- rosbag play $pathDataset



gnome-terminal -- bash -c "script -c 'roslaunch vins vins_rviz.launch' $log1"

InputYesOrExit

gnome-terminal -- bash -c "script -c 'rosrun vins vins_node $pathConfigFile' $log2"

InputYesOrExit

gnome-terminal -- bash -c "script -c 'rosrun loop_fusion loop_fusion_node $pathConfigFile' $log3"

InputYesOrExit

gnome-terminal -- rosbag play $pathDataset

# gnome-terminal -- bash -c "roslaunch vins vins_rviz.launch 2>&1 | tee -a $log1"

# InputYesOrExit

# gnome-terminal -- bash -c "rosrun vins vins_node $pathConfigFile 2>&1 | tee -a $log2"

# InputYesOrExit

# gnome-terminal -- bash -c "rosrun loop_fusion loop_fusion_node $pathConfigFile 2>&1 | tee -a $log3"

# InputYesOrExit

# gnome-terminal -- rosbag play $pathDataset