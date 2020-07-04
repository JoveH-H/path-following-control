```
              __  __       ___     ____           _                             __           __
   ___  ___ _/ /_/ /  ____/ _/__  / / /__ _    __(_)__  ___ ____________  ___  / /________  / /
  / _ \/ _ `/ __/ _ \/___/ _/ _ \/ / / _ \ |/|/ / / _ \/ _ `/___/ __/ _ \/ _ \/ __/ __/ _ \/ / 
 / .__/\_,_/\__/_//_/   /_/ \___/_/_/\___/__,__/_/_//_/\_, /    \__/\___/_//_/\__/_/  \___/_/  
/_/                                                   /___/                                    
```
# path-following-control
![Python CI](https://github.com/JoveH-H/path-following-control/workflows/Python%20CI/badge.svg)
[![License](https://img.shields.io/badge/license-Apache%202-blue.svg)](./LICENSE)
[![Release](https://img.shields.io/badge/release-v1.0.0-brightgreen.svg)](https://github.com/JoveH-H/path-following-control/releases/tag/v1.0.0)
[![Author](https://img.shields.io/badge/author-Jove-orange.svg)](https://github.com/JoveH-H)

这是一个关于路径跟踪控制实现的项目。

## 发布版本

| 版本 | 描述 |
| --- | --- |
| `v1.0.0` | `初步的模型运动及其路径跟踪的横向控制` |

## 功能模块

1. [模型(model)](./model/README.md)：用于控制的模型
    - [质点模型](./model/particle.py)

2. [控制器(controller)](./controller/README.md)：常见的控制器
    - [位置式PID控制器](./controller/positional_pid.py)
    - [增量式PID控制器](./controller/incremental_pid.py)
    - [纯跟踪控制器](./controller/pure_pursuit.py)
    - [Stanley控制器](./controller/stanley.py)
    - [线性二次型调节控制器](./controller/lqr.py)

3. [例子(examples)](./examples/README.md)：关于模型运动及其路径跟踪横向控制的例子
    - [质点模型运动](./examples/particle_motion.py)
    - [位置式PID控制](./examples/positional_pid_control.py)
    - [增量式PID控制](./examples/incremental_pid_control.py)
    - [纯跟踪控制](./examples/pure_pursuit_control.py)
    - [Stanley控制](./examples/stanley_control.py)
    - [线性二次型调节控制](./examples/lqr_control.py)

## 运行环境
> Python 3.x

> 相关依赖及其指定版本请参考 [requirements.txt](./requirements.txt)

## 使用说明
1. 安装Python依赖
```shell
$ pip3 install -r requirements.txt
```

2. 执行对应的例子文件即可。
```shell
$ python3 examples/particle_motion.py
```

## 问题
欢迎以 [GitHub Issues](https://github.com/JoveH-H/path-following-control/issues) 的形式提交问题和bug报告。

谢谢!