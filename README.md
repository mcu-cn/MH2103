# 1.运行环境
  linux，并安装和运行git, python3
# 2.klipper固件
  下载klipper固件： https://github.com/Klipper3d/klipper
  
  解压到指定目录
# 3.下载本软件包
  使用命令cd到klipper固件同级目录（注意，不是进入到klipper固件文件夹内，而是写它的顶层目录同级）
  
  使用命令git clone git@github.com:mcu-cn/MH2103.git 下载本软件包
# 4.安装MH2103到klipper固件
  命令： python MH2103/mh2.py <你的klipper文件夹名>
  
  注意，命令中<你的klipper文件夹名>必须是klipper解压后的文件夹名，区分大小写，不能为空，例如：python MH2103/mh2.py klipper-master
  
  命令运行成功后，即可到klipper文件夹中使用make menuconfig，像配置其它芯片一样，选择MH2103即可
  
  注意，MH2103主频可以配置到216M，在make menuconfig那里可以选择72M和216M中其中一个。