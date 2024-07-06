#  create a ROS 2 package containing both a server and a client in Python
> - 新建功能包，设计一个名为 perform_math_operations 的服务接口，该接口用于执行基本的数学运算，包括加法、减法、乘法和除法
> - 服务的请求为两个数num和需要操作的类型（加法、减法、乘法和除法，服务的返回为计算结果
> - 分别编写服务端节点和客户端节点实现上面的功能
> - 客户端允许在启动时以参数的形式输入两个num的值和需要操作的类型， 当接收到服务返回的结果后，将结果以话题的形式发布出来
## 0. env
- 宿主机 ubuntu 22.04 LTS
- ROS2 humble

## 1. create custom inferfaces
```shell
ros2 pkg create --build-type ament_cmake perform_math_operations
```

创建目录
```shell
cd perform_math_operations
mkdir msg srv
```

add file msg/Res.msg  # ROS 2 的消息名称必须以大写字母开头，并且只能包含字母和数字。

```shell
float64 num
```

add file `srv/Op.srv`

```shell
float64 num1
string operation
float64 num2
---
float64 result
```
add content to `CMakeLists.txt`
```bash
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Res.msg"
  "srv/Op.srv"
)
```

add content to `package.xml`

```xml
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

build package
```
colcon build --packages-select perform_math_operations
```
test interface
```
. install/setup.bash # activate env 
ros2 interface show perform_math_operations/msg/Res
```

echo

```
float64 num
```

done!
## 2. create package

在终端中运行以下命令来创建新的 `ROS2` 包

```sh
ros2 pkg create py_math --build-type ament_python --dependencies rclpy std_msgs
```

`package.xml`


```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>py_math</name>
  <version>0.0.0</version>
  <description>your description</description>
  <maintainer email="your mail">steve</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```
修改`setup.py`，在`entry_points`插入节点`server`和`client`

```python
entry_points={
    'console_scripts': [
        'service = py_math.service:main',
        'client = py_math.client:main',
    ],
},
```

创建 service.py 文件

```python
import rclpy
from rclpy.node import Node
from perform_math_operations.srv import Op

class MathOperationServer(Node):

    def __init__(self):
        super().__init__('math_operation_server')
        self.srv = self.create_service(Op, 'perform_math_operation', self.perform_operation_callback)
    
    def perform_operation_callback(self, request, response):
        if request.operation == '+':
            response.result = request.num1 + request.num2
        elif request.operation == '-':
            response.result = request.num1 - request.num2
        elif request.operation == 'x':
            response.result = request.num1 * request.num2
        elif request.operation == '/':
            if request.num2 != 0:
                response.result = request.num1 / request.num2
            else:
                response.result = float('nan')  # Return NaN if dividing by zero
        else:
            response.result = float('nan')  # Return NaN for unknown operation
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MathOperationServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
创建 client.py

```python
import sys
import rclpy
from rclpy.node import Node
from perform_math_operations.srv import Op
from std_msgs.msg import String

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Op, 'perform_math_operation')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Op.Request()
        self.publisher = self.create_publisher(String, 'operation_result', 10)

    def send_request(self, num1, num2, operation):
        self.req.num1 = num1
        self.req.num2 = num2
        self.req.operation = operation
        return self.cli.call_async(self.req)

def main():
    rclpy.init()
    minimal_client = MinimalClientAsync()
    # Check command line arguments
    if len(sys.argv) != 4:
        print("Usage: ros2 run math_operations_service math_operations_service_client <num1> <operation> <num2>")
        return

    num1 = float(sys.argv[1])
    operation = sys.argv[2]
    num2 = float(sys.argv[3])

    # Send request
    future = minimal_client.send_request(num1, num2, operation)
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()

    # Publish result
    result_msg = String()
    result_msg.data = f'Result of {operation} operation: {num1} {operation} {num2} = {response.result}'
    minimal_client.publisher.publish(result_msg)
    minimal_client.get_logger().info('Result published: "%s"' % result_msg.data)

    # Clean up
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
安装colcon

```shell
sudo apt install python3-colcon-common-extensions
```

构建包 & 设置环境变量

```shell
colcon build
source install/setup.bash
```

在两个终端分别启动`server`和`client`
```sh
ros2 run py_math server    
ros2 run py_math client <a> <operation> <b> 
```
例如
```sh
ros2 run py_math client 5 + 3
```

测试
```
steve@steve-Lenovo-Legion-Y70002021:~/ros2_ws/src$ ros2 run py_math client 1 + 3 
[INFO] [1720963711.978652635] [minimal_client_async]: Result published: "Result of + operation: 1.0 + 3.0 = 4.0"
steve@steve-Lenovo-Legion-Y70002021:~/ros2_ws/src$ ros2 run py_math client 1 - 3 
[INFO] [1720963715.992976927] [minimal_client_async]: Result published: "Result of - operation: 1.0 - 3.0 = -2.0"
steve@steve-Lenovo-Legion-Y70002021:~/ros2_ws/src$ ros2 run py_math client 1 x 3 
[INFO] [1720963719.191213990] [minimal_client_async]: Result published: "Result of x operation: 1.0 x 3.0 = 3.0"
steve@steve-Lenovo-Legion-Y70002021:~/ros2_ws/src$ ros2 run py_math client 1 / 3 
[INFO] [1720963722.866190330] [minimal_client_async]: Result published: "Result of / operation: 1.0 / 3.0 = 0.3333333333333333"
```

最终的目录结构
```
py_math/
├── LICENSE
├── package.xml
├── py_math
│   ├── client.py
│   ├── __init__.py
│   └── service.py
├── resource
│   └── py_math
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```
参考
- https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html
- https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html
- https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html
