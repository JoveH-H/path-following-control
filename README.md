```
                 __  __          ____      ____              _                                    __             __
    ____  ____ _/ /_/ /_        / __/___  / / /___ _      __(_)___  ____ _      _________  ____  / /__________  / /
   / __ \/ __ `/ __/ __ \______/ /_/ __ \/ / / __ \ | /| / / / __ \/ __ `/_____/ ___/ __ \/ __ \/ __/ ___/ __ \/ / 
  / /_/ / /_/ / /_/ / / /_____/ __/ /_/ / / / /_/ / |/ |/ / / / / / /_/ /_____/ /__/ /_/ / / / / /_/ /  / /_/ / /  
 / .___/\__,_/\__/_/ /_/     /_/  \____/_/_/\____/|__/|__/_/_/ /_/\__, /      \___/\____/_/ /_/\__/_/   \____/_/   
/_/                                                              /____/                                            
```
# path-following-control
[![License](https://img.shields.io/badge/license-Apache%202-blue)](./LICENSE)
[![Author](https://img.shields.io/badge/author-Jove-orange.svg)](https://github.com/JoveH-H)

这是一个关于路径跟踪控制实现的项目。

## 发布版本

| 版本 | 描述 |
| --- | --- |
| `-` | `-` |

## 功能模块

1. [模型(model)](./model/README.md)：用于控制的模型
    - [质点模型](./model/particle.py)

2. [控制器(controller)](./controller/README.md)：常见的控制器
    - [位置式PID控制器](./controller/positional_pid.py)

3. [例子(examples)](./examples/README.md)：关于模型运动及其路径跟踪控制的例子
    - [质点模型运动](./examples/particle_motion.py)
    - [质点模型位置式PID控制](./examples/particle_positional_pid_control.py)

## 运行环境
> Python 3.7

> sys + math + numpy + scipy + matplotlib + copy

## 使用说明
直接执行对应的例子文件即可。
```shell
$ python3 examples/particle_motion.py
```

## 问题
欢迎以 [GitHub Issues](https://github.com/JoveH-H/path-following-control/issues) 的形式提交问题和bug报告。

谢谢!