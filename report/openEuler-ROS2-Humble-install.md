# 在 openEuler 24.03 LTS 上安装 ROS2 Humble
## 系统环境
- 宿主机 Ubuntu 22.03 LTS 
- QEMU openEuler x86_64 虚拟机, QEMU verision 9.0.1
- 镜像 [openEuler 24.03 LTS](https://repo.openeuler.org/openEuler-24.03-LTS/virtual_machine_img/x86_64/openEuler-24.03-LTS-x86_64.qcow2.xz)

## ROS2 安装

修改 `/etc/yum.repos.d/openEulerROS.repo` 文件，改为以下内容

```
[openEulerROS-humble]
name=openEulerROS-humble
baseurl=http://121.36.84.172/dailybuild/EBS-openEuler-24.03-LTS/EBS-openEuler-24.03-LTS/EPOL/multi_version/ROS/humble/x86_64/
enabled=1
gpgcheck=0

[openEulerROS-humble-source]
name=openEulerROS-humble-source
baseurl=http://121.36.84.172/dailybuild/EBS-openEuler-24.03-LTS/EBS-openEuler-24.03-LTS/EPOL/multi_version/ROS/humble/source
enabled=1
gpgcheck=0

```
安装 ros
```shell
dnf install "ros-humble-*" --skip-broken
```

## 测试

# 以小乌龟（ros-humble-turtlesim）为例
dnf install ros-humble-turtlesim