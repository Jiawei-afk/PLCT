# irobot ros2 perf eval package 的测试和评估
本文参考[ros2-performance的官方说明](https://github.com/irobot-ros/ros2-performance/blob/rolling/README.md)进行测试和评估，详细内容参照链接
## env
- ROS2 Humble
- ubuntu 22.04 LTS

## build
请确保在编译前，机器支持Python3，CMake 和 colon

由于该包是针对ROS2 Rolling 开发的，其中涉及到`rclcpp::experimental::EventsExecutor`的内容，在最新的`rclcpp`中，已经删除了`rclcpp::experimental`下的`EventsExecutor`内容，需要hack操作。
改动如下
```shell
diff --git a/performance_test/src/executors.cpp b/performance_test/src/executors.cpp
index 5e58f24..822e6c9 100644
--- a/performance_test/src/executors.cpp
+++ b/performance_test/src/executors.cpp
@@ -50,12 +50,14 @@ std::shared_ptr<rclcpp::Executor> make_executor(ExecutorType type)
     case ExecutorType::STATIC_SINGLE_THREADED_EXECUTOR:
       executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
       break;
-    case ExecutorType::EVENTS_EXECUTOR:
-      executor = std::make_shared<rclcpp::experimental::executors::EventsExecutor>();
-      break;
     case ExecutorType::MULTI_THREAD_EXECUTOR:
       executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
       break;
+    default:
+      throw std::runtime_error("Unsupported ExecutorType");
   }

   return executor;
```
build cmd
```shell
mkdir -p ~/performance_ws/src
cd ~/performance_ws/src
git clone https://github.com/irobot-ros/ros2-performance
cd ros2-performance
git submodule update --init --recursive
cd ../..
colcon build
```
## Run
### 测试过程
[irobot_benchmark package](https://github.com/irobot-ros/ros2-performance/tree/rolling/irobot_benchmark)包含用于评估的图形拓扑的主要应用和示例。 
```shell
source ~/performance_ws/install/setup.bash
cd ~/performance_ws/install/irobot_benchmark/lib/irobot_benchmark
./irobot_benchmark topology/sierra_nevada.json
```
### 输出结果
结果将被打印到屏幕上，并保存在目录`./sierra_nevada_log` 中。
```
topology_json_list: topology/sierra_nevada.json
system_executor: StaticSingleThreadedExecutor
node_type: Node
ipc: on
ros_params: on
name_threads: on
duration_sec: 5 seconds
resources_sampling_per_ms: 1000
csv_out: off
tracking.is_enabled: off

Start test!
[ResourceUsageLogger]: Logging to sierra_nevada_log/resources.txt
[INFO] [1721640504.151860039] [montreal]: PerformanceNode montreal created with executor id 0
[INFO] [1721640504.162740705] [montreal]: Publisher to amazon created
[INFO] [1721640504.163350891] [montreal]: Publisher to nile created
[INFO] [1721640504.163576278] [montreal]: Publisher to ganges created
[INFO] [1721640504.163953764] [montreal]: Publisher to danube created
[INFO] [1721640504.165128763] [lyon]: PerformanceNode lyon created with executor id 0
[INFO] [1721640504.165550611] [lyon]: Publisher to tigris created
[INFO] [1721640504.166943906] [lyon]: Subscription to amazon created
[INFO] [1721640504.168116408] [hamburg]: PerformanceNode hamburg created with executor id 0
[INFO] [1721640504.168508713] [hamburg]: Publisher to parana created
[INFO] [1721640504.169560105] [hamburg]: Subscription to nile created
[INFO] [1721640504.170672209] [hamburg]: Subscription to tigris created
[INFO] [1721640504.170996034] [hamburg]: Subscription to ganges created
[INFO] [1721640504.171953770] [hamburg]: Subscription to danube created
[INFO] [1721640504.173186376] [osaka]: PerformanceNode osaka created with executor id 0
[INFO] [1721640504.173629323] [osaka]: Publisher to salween created
[INFO] [1721640504.174641513] [osaka]: Subscription to parana created
[INFO] [1721640504.175882305] [mandalay]: PerformanceNode mandalay created with executor id 0
[INFO] [1721640504.176322268] [mandalay]: Publisher to missouri created
[INFO] [1721640504.177319638] [mandalay]: Subscription to salween created
[INFO] [1721640504.177632436] [mandalay]: Subscription to danube created
[INFO] [1721640504.178948945] [ponce]: PerformanceNode ponce created with executor id 0
[INFO] [1721640504.179172541] [ponce]: Publisher to mekong created
[INFO] [1721640504.179403122] [ponce]: Publisher to congo created
[INFO] [1721640504.180381438] [ponce]: Subscription to missouri created
[INFO] [1721640504.180667976] [ponce]: Subscription to danube created
[INFO] [1721640504.180927476] [ponce]: Subscription to volga created
[INFO] [1721640504.182364170] [barcelona]: PerformanceNode barcelona created with executor id 0
[INFO] [1721640504.182592569] [barcelona]: Publisher to lena created
[INFO] [1721640504.182946797] [barcelona]: Subscription to mekong created
[INFO] [1721640504.184430891] [georgetown]: PerformanceNode georgetown created with executor id 0
[INFO] [1721640504.184735561] [georgetown]: Publisher to volga created
[INFO] [1721640504.185140149] [georgetown]: Subscription to lena created
[INFO] [1721640504.186820722] [geneva]: PerformanceNode geneva created with executor id 0
[INFO] [1721640504.187128903] [geneva]: Publisher to arkansas created
[INFO] [1721640504.187566833] [geneva]: Subscription to congo created
[INFO] [1721640504.187957432] [geneva]: Subscription to danube created
[INFO] [1721640504.188300740] [geneva]: Subscription to parana created
[INFO] [1721640504.189912177] [arequipa]: PerformanceNode arequipa created with executor id 0
[INFO] [1721640504.190257755] [arequipa]: Subscription to arkansas created
[INFO] [1721640504.190481695] [ros2-performance]: Waiting for discovery
[INFO] [1721640504.190728805] [ros2-performance]: Starting to spin

Subscriptions stats:
node           topic          size_b    received_msgs  late_msgs too_late_msgs  lost_msgs mean_us   sd_us     min_us    max_us    freq_hz   throughput_Kb_per_sec
lyon           amazon         36        501            0         0              0         153       113       19        1163      100       3.52207
hamburg        danube         8         501            0         0              0         90        64        21        1019      100       0.782579
hamburg        ganges         16        501            0         0              0         72        67        11        961       100       1.5653
hamburg        nile           16        501            0         0              0         110       83        17        931       100       1.56531
hamburg        tigris         16        501            0         0              0         126       89        19        711       100       1.56461
osaka          parana         12        501            0         0              0         155       109       21        823       100       1.17274
mandalay       danube         8         501            0         0              0         106       74        26        1182      100       0.782602
mandalay       salween        48        50             0         0              0         150       109       40        736       10        0.468818
ponce          danube         8         501            0         0              0         116       78        29        1202      100       0.782603
ponce          missouri       10000     50             0         0              0         133       98        42        705       10        97.6705
ponce          volga          8         10             0         0              0         42        19        14        67        2         0.015625
barcelona      mekong         100       10             0         0              0         82        38        27        124       2         0.195313
georgetown     lena           50        50             0         0              0         166       105       22        334       10        0.488295
geneva         congo          16        50             0         0              0         89        52        21        181       10        0.156248
geneva         danube         8         501            0         0              0         126       82        32        1210      100       0.782603
geneva         parana         12        501            0         0              0         185       128       26        901       100       1.17275
arequipa       arkansas       16        50             0         0              0         129       105       21        336       10        0.156259

Publishers stats:
node           topic          size_b    received_msgs  late_msgs too_late_msgs  lost_msgs mean_us   sd_us     min_us    max_us    freq_hz   throughput_Kb_per_sec
montreal       amazon         36        0              0         0              0         84        58        10        297       100       3.5214
montreal       danube         8         0              0         0              0         44        24        11        118       100       0.782435
montreal       ganges         16        0              0         0              0         15        8         6         57        100       1.56501
montreal       nile           16        0              0         0              0         46        28        9         121       100       1.56504
lyon           tigris         16        0              0         0              0         66        48        9         174       100       1.56442
hamburg        parana         12        0              0         0              0         80        55        10        217       100       1.1726
osaka          salween        48        0              0         0              0         41        39        12        294       10        0.46876
mandalay       missouri       10000     0              0         0              0         66        48        20        345       10        97.6592
ponce          congo          16        0              0         0              0         42        25        9         81        10        0.156248
ponce          mekong         100       0              0         0              0         42        20        14        66        2         0.195312
barcelona      lena           50        0              0         0              0         54        29        10        100       10        0.488285
georgetown     volga          8         0              0         0              0         20        9         6         32        2         0.0156249
geneva         arkansas       16        0              0         0              0         61        51        9         171       10        0.156254

System total:
received_msgs  mean_us   late_msgs late_perc too_late_msgs  too_late_perc  lost_msgs lost_perc
5280           124       0         0         0              0              0         0
```
![alt text](image-2.png)
### 分析

#### 框架配置参数：
topology_json_list: 使用 topology/sierra_nevada.json 文件定义系统拓扑。
system_executor: 使用静态单线程执行器 (StaticSingleThreadedExecutor)。
node_type: 节点类型为普通的 ROS 2 节点 (Node)。
ipc: 启用节点间的进程间通信 (IPC)。
ros_params: 启用 ROS 参数。
name_threads: 启用线程命名。
duration_sec: 测试持续时间为 5 秒。
resources_sampling_per_ms: 每毫秒采样资源数据 1000 次。
csv_out: 不生成 CSV 输出。
tracking.is_enabled: 跟踪未启用。
#### 测试结果分析：
订阅统计和发布统计显示了每个节点对各个主题的消息处理情况，包括消息大小、接收数量、延迟、丢失等详细信息。
系统总览部分显示了整体接收消息数量和相关的性能指标。
## 扩展性能框架并测试自己的系统
`irobot_benchmark/topology`包含一些可用于定义系统的 json 文件示例。
如果要创建自己的 "JSON topology"，请按照[如何创建新拓扑的说明](https://github.com/irobot-ros/ros2-performance/blob/rolling/performance_test_factory/create_new_topology.md)进行操作。 如果要在topology中使用自定义 ROS 2 消息接口，则应查看[performance_test_plugin_cmake](https://github.com/irobot-ros/ros2-performance/blob/rolling/performance_test_plugin_cmake)
### 框架结构
performance_test：此包提供了类，该类提供了用于轻松添加发布者、订阅、客户端和服务的 API，并监视通信的性能。 此外，该类允许同时启动多个节点，同时确保它们相互发现，并监控整个系统的性能。 此外，此 pacakge 包含用于可视化应用程序性能的脚本。performance_test::PerformanceNodeperformance_test::System
performance_metrics：提供工具来测量和记录 ROS 2 系统中的各种性能相关指标。
performance_test_msgs：此包包含包直接用于衡量性能的基本接口定义。performance_test
performance_test_factory：此包提供了可用于根据运行时提供的一些参数创建具有特定发布者和订阅的对象的类：这可以通过 JSON 文件或命令行选项来完成。 可以在这些节点中使用的接口（msg 和 srv）必须在所谓的 .performance_test_factory::TemplateFactoryperformance_test::PerformanceNodeperformance_test_factory_plugins
performance_test_plugin_cmake：此包提供了用于从接口定义生成工厂插件的 CMake 函数。
irobot_interfaces_plugin：此软件包提供 iRobot 系统拓扑中使用的所有接口。performance_test_factory_plugin
irobot_benchmark：此软件包提供了我们的主要基准测试应用程序。 此可执行文件可以加载一个或多个 json 拓扑，并从每个拓扑中创建一个在特定进程中运行的 ROS2 系统。 它还包含示例 json 拓扑。
composition_benchmark：此软件包包含可用于分析 ROS 2 组合概念的应用程序和工具。


