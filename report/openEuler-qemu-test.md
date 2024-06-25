# 在wsl上使用QEMU安装测试X86架构/ARM/RISCV架构的openEuler-24.03-LTS
## 系统环境
- WSL2 (Ubuntu 22.04.3 LTS) 
- QEMU verison >= 8.1 # 如果使用`UEFI`引导 ，需使用 8.1 版本以上的 QEMU。
## 安装虚拟化组件
### 安装步骤


1. 安装QEMU组件

见[qemu install 指南](./qemu-install.md)

2. 验证安装是否成功

查看内核是否支持KVM虚拟化，即查看/dev/kvm和/sys/module/kvm文件是否存在，命令和回显如下：

```shell
$ ls /dev/kvm
/dev/kvm

$ ls /sys/module/kvm
parameters uevent
```
## 准备镜像

在[openEuler Repo
](http://repo.openeuler.org/openEuler-24.03-LTS/virtual_machine_img/)下载`aarch64`, `riscv64`, `x86_64`的镜像文件并解压，其中riscv64需要额外下载启动固件和官方虚拟机启动脚本。

```shell
virtual_machine_img/
├── aarch64
│   └── openEuler-24.03-LTS-aarch64.qcow2
├── riscv64
│   ├── RISCV_VIRT_CODE.fd
│   ├── RISCV_VIRT_VARS.fd
│   ├── fw_dynamic_oe_2403_penglai.bin
│   ├── openEuler-24.03-LTS-riscv64.qcow2
│   ├── start_vm.sh
│   └── start_vm_penglai.sh
└── x86_64
    └── openEuler-24.03-LTS-x86_64.qcow2
```
## 准备虚拟机网络
...待补充

## QEMU 启动 openEuler for x86_64   

- 下载镜像

```shell
$ wget https://repo.openeuler.org/openEuler-24.03-LTS/virtual_machine_img/x86_64/openEuler-24.03-LTS-x86_64.qcow2.xz
$ xz -d openEuler-24.03-LTS-x86_64.qcow2.xz
$ wget https://repo.openeuler.org/openEuler-24.03-LTS/OS/x86_64/images/pxeboot/initrd.img
$ wget https://repo.openeuler.org/openEuler-24.03-LTS/OS/x86_64/images/pxeboot/vmlinuz
```
- 启动虚拟机
```shell
$ qemu-system-x86_64 \
  -name openEulerVM-x86_64 \
  -m 4G \
  -smp 4,sockets=2,cores=2,threads=1 \
  -cpu host \
  -drive file=openEuler-24.03-LTS-x86_64.qcow2,id=hd0,format=qcow2 \
  -append 'console=ttyAMA0' \
  -enable-kvm
```
进入系统~
```shell
Welcome to 6.6.0-28.0.0.34.oe2403.x86_64

System information as of time:  Sat Jun 22 02:01:27 AM UTC 2024

System load:    0.14
Memory used:    1.7%
Swap used:      0.0%
Usage On:       5%
IP address:     10.0.2.15
Users online:   1


[root@localhost ~]# uname -a
Linux localhost.localdomain 6.6.0-28.0.0.34.oe2403.x86_64 #1 SMP Mon May 27 22:22:46 CST 2024 x86_64 x86_64x
```
## QEMU 启动 openEuler for aarch64   

下载镜像

```shell
$ wget https://repo.openeuler.org/openEuler-24.03-LTS/virtual_machine_img/aarch64/openEuler-24.03-LTS-aarch64.qcow2.xz
$ xz -d openEuler-24.03-LTS-aarch64.qcow2.xz
$ wget https://repo.openeuler.org/openEuler-24.03-LTS/OS/aarch64/images/pxeboot/initrd.img
$ wget https://repo.openeuler.org/openEuler-24.03-LTS/OS/aarch64/images/pxeboot/vmlinuz
```
启动虚拟机
```shell
$ qemu-system-aarch64 \
  -M virt -cpu cortex-a57 \
  -smp 8 -m 4G \
  -hda openEuler-24.03-LTS-aarch64.qcow2.xz \
  -kernel vmlinuz \
  -initrd initrd.img \
  -nic user,model=e1000,hostfwd=tcp::2222-:22 \
  -append 'root=/dev/vda2 console=ttyAMA0'
```
进入系统~
```shell
Welcome to 6.6.0-28.0.0.34.oe2403.aarch64

System information as of time:  Fri Jun 21 04:18:58 AM UTC 2024

System load:    0.08
Memory used:    2.0%
Swap used:      0.0%
Usage On:       5%
IP address:     10.0.2.15
Users online:   1


[root@localhost ~]# uname -a
Linux localhost 6.6.0-28.0.0.34.oe2403.aarch64 #1 SMP Mon May 27 22:43:49 CST 2024 aarch64 aarch64 aarch64 GNU/Linux
```

## QEMU 启动 openEuler for riscv64   
- 下载镜像

```shell
$ wget https://repo.openeuler.org/openEuler-24.03-LTS/virtual_machine_img/riscv64/openEuler-24.03-LTS-riscv64.qcow2.xz
$ xz -d openEuler-24.03-LTS-riscv64.qcow2.xz
$ wget https://repo.openeuler.org/openEuler-24.03-LTS/OS/x86_64/images/pxeboot/initrd.img
$ wget https://repo.openeuler.org/openEuler-24.03-LTS/OS/x86_64/images/pxeboot/vmlinuz
```


- 启动虚拟机
```shell
$ bash start_vm.sh
```
进入系统~
```shell
openEuler 24.03 (LTS)
Kernel 6.6.0-27.0.0.31.oe2403.riscv64 on an riscv64

localhost login: root
Password:


Welcome to 6.6.0-27.0.0.31.oe2403.riscv64

System information as of time:  Sat Apr 20 02:23:36 PM UTC 2024

System load:    1.37
Memory used:    1.9%
Swap used:      0.0%
Usage On:       4%
IP address:     10.0.2.15
Users online:   1


[root@localhost ~]# uname -a
Linux localhost.localdomain 6.6.0-27.0.0.31.oe2403.riscv64 #1 SMP Fri May 24 21:52:58 CST 2024 riscv64 riscv64 riscv64 GNU/Linux
[root@localhost ~]#
```
## 参考

- [1] [openEuler社区](https://docs.openeuler.org/zh/docs/24.03_LTS/docs/Virtualization/%E8%AE%A4%E8%AF%86%E8%99%9A%E6%8B%9F%E5%8C%96.html)