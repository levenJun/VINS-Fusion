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

source ../../devel/setup.bash

# pathConfigFile="/home/delljun/p2/3-slam/7-1-vins-fusion/1-ws-ori/src/VINS-Fusion-2401/config/vslam-2023_11_17_17_10_28/vslam_stereo_imu_config.yaml"
# # pathDataset='/media/delljun/Bubby/data2/datasets/12-910-debug/1-/2023_11_17_17_10_28'
# pathDataset='/media/delljun/Bubby/data2/datasets/12-910-debug/1-/2023_11_17_17_26_30'

pathConfigFile="/home/hojun/Disk/E/pjts/2-slams/2-vins/ws/src/VINS-Fusion/config/yy_stereo_1/vslam_stereo_imu_config.yaml"
# pathDataset='/media/leven/d2/250715-a1088/office-2-p1/20250715154004'
# pathDataset='/media/leven/d2/250715-a1088/0731/2-lawn-noleaf/20250731115908-3-c-ok'
# pathDataset='/media/leven/d2/250715-a1088/2-lawn/1-small/20250715102707-1-3circles'
pathDataset='/media/leven/d2/250715-a1088/2-lawn/1-small/20250715103004-2-2circles'

# pathConfigFile="/home/delljun/p2/3-slam/7-1-vins-fusion/1-ws-ori/src/VINS-Fusion-2401/config/vslam-2023_11_23_15_04_18/vslam_stereo_imu_config.yaml"
# pathDataset='/media/delljun/Bubby/data2/datasets/12-910-debug/2-/2023_11_23_15_04_18'

echo "test vins by:$pathDataset"

time=$(date "+%Y%m%d_%H%M%S")

mkdir -p log/vslam/$time
log1=log/vslam/$time/log1_rviz_${time}.txt
log2=log/vslam/$time/log2_vins_${time}.txt
log3=log/vslam/$time/log3_loop_${time}.txt
touch $log1 $log2 $log3

# Step 1: 启动rviz界面
# gnome-terminal -- roslaunch vins vins_rviz.launch 2>&1 | tee -a log/$time/log1_rviz_${time}.txt
# InputYesOrExit

# gnome-terminal -- rosrun vins vins_node $pathConfigFile 2>&1 | tee -a log/$time/log3_vins_${time}.txt
# InputYesOrExit

# gnome-terminal -- rosrun loop_fusion loop_fusion_node $pathConfigFile 2>&1 | tee -a log/$time/log2_loop_${time}.txt

gnome-terminal -- bash -c "script -c 'roslaunch vins vins_rviz.launch' $log1"
InputYesOrExit

gnome-terminal -- bash -c "script -c 'rosrun loop_fusion loop_fusion_node $pathConfigFile' $log3"
InputYesOrExit

gnome-terminal -- bash -c "script -c 'rosrun vins vslam_test3 $pathConfigFile $pathDataset 250' $log2"