# 2025全国大学生智能车竞赛极速光电龙芯 # 
## 龙芯2k0300开发板环境配置：

## 1. 配置wifi和ssh： ##
vi /etc/rc.local
配置连接wifi热点和密码

## 2. 编译内核 ## 
将修改好的内核放入/boot/vmlinuz
执行sync后复位
## 3. 编译龙芯架构opencv ##
修改/dev/ld.conf.d/opencv.conf，设置opencvlib路径
执行ldconfig
## 4. 通过cmakelist交叉编译执行二进制文件即可




   
