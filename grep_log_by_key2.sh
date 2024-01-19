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

#读取参数
#-d:处理文件夹下所有.csv文件
#-b:输入数据文件夹/文件
#-A:融合轴模式:1-3轴；2-6轴；3-9轴；5-369轴全部来一遍；其他-9轴
#-r:统计文件的前缀
while getopts "f:" opt
do
  case "$opt" in
    f ) path="$OPTARG" ;;
    ? ) echo "invalid params" ;;
  esac
done

#关键字提取统计文件
KeyStrList=("processMeasurements, time=," "latest_time=,")


if [ -f "$path" ]; then
    echo "The path is a valid file:$path"

    directory=$(dirname "$path")
    filename=$(basename "$path")

    echo "Directory: $directory"
    echo "Filename: $filename"

    # Iterate over the list and print its contents with indices
    for index in "${!KeyStrList[@]}"; do
        item="${KeyStrList[$index]}"
        echo "Index: $index, Item: $item"
        grep -a "$item" $path > $path.$index.csv
    done    

else
    echo "The path is not a valid file."
fi
