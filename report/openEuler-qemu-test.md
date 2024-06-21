# 在wsl上使用QEMU安装测试X86架构/ARM/RISCV架构的openEuler-24.03-LTS
## 系统环境
- WSL2 (Ubuntu 22.04.3 LTS) 
- 
## 安装虚拟化组件
### 安装步骤
1. 安装QEMU组件
```shell
sudo apt install qemu qemu-kvm virt-manager bridge-utils
sudo apt install qemu-system-riscv64
sudo apt install qemu-system-aarch64
```
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

## 准备引导固件
### 概述
针对不同的架构，引导的方式有所差异。x86支持UEFI（Unified Extensible Firmware Interface）和BIOS方式启动，AArch64仅支持UEFI方式启动。openEuler默认已安装BIOS启动对应的引导文件，不需要用户额外操作。所以这里仅介绍UEFI启动方式的安装方法。

统一的可扩展固件接口UEFI是一种全新类型的接口标准，用于开机自检、引导操作系统的启动，是传统BIOS的一种替代方案。EDK II是一套实现了UEFI标准的开源代码，在虚拟化场景中，通常利用EDK II工具集，通过UEFI的方式启动虚拟机。使用EDK II工具需要在虚拟机启动之前安装对应的软件包 ，本节介绍EDK II的安装方法。

### 安装方法
如果使用UEFI方式引导，需要安装工具集EDK II，AArch64架构对应的安装包为edk2-aarch64，x86架构对应的安装包为edk2-ovmf。这里以AArch64架构为例，给出具体的安装方法，x86架构仅需将edk2-aarch64替换为edk2-ovmf。

```shell
sudo apt install qemu-efi-aarch64   # UEFI firmware for 64-bit ARM virtual machines
sudo apt install ovmf               # UEFI firmware for 64-bit x86 virtual machines

sudo apt install qemu-kvm libvirt-daemon-system libvirt-clients bridge-utils virtinst virt-manager
```
## x86_64 for QEMU 
- 下载镜像

```shell
$ wget https://repo.openeuler.org/openEuler-24.03-LTS/virtual_machine_img/x86_64/openEuler-24.03-LTS-x86_64.qcow2.xz
$ xz -d openEuler-24.03-LTS-x86_64.qcow2.xz
$ wget https://repo.openeuler.org/openEuler-24.03-LTS/OS/x86_64/images/pxeboot/initrd.img
$ wget https://repo.openeuler.org/openEuler-24.03-LTS/OS/x86_64/images/pxeboot/vmlinuz
```
- 启动虚拟机
```shell
$ bash start_vm.sh
```

## aarch64 for QEMU 
下载镜像

```shell
$ wget https://repo.openeuler.org/openEuler-24.03-LTS/virtual_machine_img/aarch64/openEuler-24.03-LTS-aarch64.qcow2.xz
$ xz -d openEuler-24.03-LTS-aarch64.qcow2.xz
$ wget https://repo.openeuler.org/openEuler-24.03-LTS/OS/aarch64/images/pxeboot/initrd.img
$ wget https://repo.openeuler.org/openEuler-24.03-LTS/OS/aarch64/images/pxeboot/vmlinuz
```
启动虚拟机
```shell
$ bash start_vm.sh
```
进入系统~
```shell
Welcome to 6.6.0-28.0.0.34.oe2403.aarch64

System information as of time:  Thu Jun 20 01:50:13 AM UTC 2024

System load:    0.28
Memory used:    2.0%
Swap used:      0.0%
Usage On:       5%
IP address:     10.0.2.15
Users online:   1


[root@localhost ~]# uname -a
Linux localhost 6.6.0-28.0.0.34.oe2403.aarch64 #1 SMP Mon May 27 22:43:49 CST 2024 aarch64 aarch64 aarch64 GNU/Linux
```

## riscv64 for QEMU 
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

## 参考

- [1] [openEuler社区](https://docs.openeuler.org/zh/docs/24.03_LTS/docs/Virtualization/%E8%AE%A4%E8%AF%86%E8%99%9A%E6%8B%9F%E5%8C%96.html)