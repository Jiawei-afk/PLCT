# QEMU 下载与编译

## 获取 QEMU 源码

我们可以前往 qemu 的官网下载对应版本的源码：

```shell
$ wget https://download.qemu.org/qemu-9.0.1.tar.xz
$ tar -xf qemu-9.0.1.tar.xz
```
也可以直接从 GitHub 上获取：

```shell
$ git clone git@github.com:qemu/qemu.git
```


## 编译 QEMU

首先安装一些必备的依赖

```shell
$ sudo apt -y install ninja-build build-essential zlib1g-dev pkg-config libglib2.0-dev binutils-dev libpixman-1-dev libfdt-dev
```
创建 build 目录并配置对应的编译选项
```
$ mkdir build && cd build
$ ../configure --target-list=aarch64-softmmu,aarch64-linux-user,x86_64-softmmu,x86_64-linux-user,riscv64-softmmu,riscv64-linux-user --enable-kvm --enable-spice --enable-guest-agent --enable-libusb --enable-usb-redir --enable-slirp
```

这里我们手动指定了这几个编译选项

- -enable-kvm：开启 kvm 支持

- -target-list=<架构名>：指定要编译的 CPU 架构，这里我们配置为 x86_64、aarch64、riscv64
- -enable-debug：能够对 Qemu 进行调试

make

```
sudo make -j$(nproc)
sudo make install
```

## 验证
```
qemu-system-aarch64 --version
qemu-aarch64 --version

qemu-system-x86_64 --version
qemu-x86_64 --version

qemu-system-riscv64 --version
qemu-riscv64 --version
```
以**aarch64**为例

- qemu-aarch64 是用户模式的模拟器（更精确的表述应该是系统调用模拟器）

- qemu-system-aarch64 是系统模拟器，可以模拟出整个机器并运行操作系统

- qemu-aarch64 仅可用来运行二进制文件，因此你可以交叉编译完例如hello world之类的程序然后交给 qemu-aarch64 来运行，简单而高效。

而 qemu-system-aarch64 则需要把hello world程序下载到客户机操作系统能访问到的硬盘里才能运行。

## 调试 QEMU
QEMU 允许我们通过 `-s` 或是 `-gdb tcp::1234` 这样的附加参数来调试虚拟机（比如说调试 Linux kernel），但有的时候我们想要直接调试 QEMU 本体（比如说调试一些自己写的模拟设备），这个时候就需要我们将 Host 上的 QEMU 进程作为待调试对象。

由于 QEMU 本质上也是运行在宿主机上的一个进程，因此我们只需要直接找到其对应的 pid 便能直接使用 `gdb attach` 进行调试。

