# 在 openEuler 24.03 LTS 上安装 ROS2 Humble
## 系统环境
- 宿主机 Ubuntu 22.03 LTS 
- QEMU openEuler x86_64 虚拟机， QEMU verision 9.0.1
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
安装所有`ros-humble`包
```shell
sudo dnf install "ros-humble-*" 
```
在安装过程中，由于某些 ROS 软件包依赖于 `ros-humble-admittance-controller` 包，而该包与 `ros-humble-generate-parameter-library-example` 包之间存在冲突，这导致了安装过程中的问题，所以暂时排除了冲突的包。
```shell
sudo dnf install "ros-humble-*" --exclude=ros-humble-admittance-controller --exclude=ros-humble-generate-parameter-library-example --skip-broken 
```
安装完成，添加环境变量
```shell
echo "source /opt/ros/humble/setup.sh" >> ~/.bashrc
source ~/.bashrc
```
## 测试

# 以小乌龟（ros-humble-turtlesim）为例
```
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```
![!\[alt text\](image.png)](img/image.png)