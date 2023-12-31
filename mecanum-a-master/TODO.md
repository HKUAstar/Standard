# Work To be done & optimized

- **待完成**：不完成可能导致整车功能不完整
- **待优化**：对已有的功能进行性能提高/模块解耦/可维护性增强
- **待添加**：不紧急的/锦上添花的功能

**==标为黄色高亮的代表紧急程度高。==**

## assorted

- [ ] 由于我们读写和传递的数据结构都不大，基本不会发生读写时任务切换的情况。典型的数据读写时间都是~μs，故没有对数据访问的接口添加互斥锁或关闭全局中断。后续有需求（如大量数据复制）可以添加。可以新增一个bsp_mutex或者module层的ds，提供相应支持。实际上freertos提供了一些供线程（任务）间进行数据交互的类型和函数，请查阅对应文档。

## BSP

### 待完成



### 待优化

#### bsp_pwm

- [ ] 是否允许修改预分频计数器？

### 待添加

#### bsp_spi

- [x] 待测试（BMI088，预计1.20前完成）

#### bsp_iic

- [x] 待测试（OLED，IST8310，预计1.20前完成）

#### bsp_gpio

- [x] 增加GPIO引脚的控制（有待商榷是否需要单独添加，HAL实际上已经提供较好的封装）

#### bsp_usb

- [ ] 增加usb的支持，用于虚拟串口或和PC进行高速通信

#### bsp_blueteetch

- [ ] 增加蓝牙功能，方便调试和测试

#### bsp_wifi

- [ ] 增加无线网络功能，方便调试和测试

#### bsp_log

- [ ] 在vscode中添加rtt viewer的查看窗口，提供一键启用的task.json 




---




## Module

### 待完成

- [ ] 给所有模块增加Daemon 模块以提供离线和异常检测

- [ ] 为键鼠/遥控器/ps手柄/视觉上位机等各种控制器提供一套统一的接口，把发来数据转化为标准的控制数据，包括底盘速度云台角度等等

- [ ] 给每个模块增加调试的条件编译，并增加bsp log的输出。或直接在运行时添加log等级，输出不同的信息。

#### buzzer

- [ ] 使用bsp_pwm添加buzzer模块

- [ ] 添加初始化完成时的音乐播放

#### ==servo_motor==

舵机模块，需要预先定义90/180/360连续旋转的电机类型，并且能够设定max和min位置。

- [x] 编写舵机模块（待测试和优化）

#### imu

- [x] 增加角速度的反馈，并且能够获取加速度值（目前看来修改反馈数据类型定义即可）

#### refereeUI

- [x] 提供UI绘制封装
- [x] 绘制电容剩余容量/当前底盘状态/当前云台状态/底盘位置/射表
- [ ] 绘制视觉识别UI/识别状态/击打状态

#### ==master_machine==

- [ ] 增加IMU数据的时间戳
- [ ] 增加加速度计数据
- [ ] 重构seasky protocol的接口
- [ ] 增加数据未更新的处理


### 待优化

#### buzzer

> 是否需要在module层就和**daemon**模块配合？

- [ ] 增加错误或异常提示音
- [ ] 增加功能提示音

#### led

>  是否需要和**daemon**模块配合？这同时会影响到module层的led_task，放在这一并解决。

- [ ] 增加错误或异常流水灯
- [ ] 增加功能点灯

#### BMI088

需要重写部分数据结构，并在bsp_spi完成之后移植到新的bsp上。（等待bsp_spi的测试）

- [x] 重构imu模块

#### remote_control

- [x] 将键盘数据解析替换为位域操作（已完成，待测试）

#### referee

- [ ] 优化三个裁判系统模块的组织关系

#### message_center

- [ ] 增加队列剩余信息和数据时间戳的支持

#### can_comm

- [ ] 增加can_comm数据未更新的处理

#### controller

- [x] 增加前馈数据
- [ ] 将PID的初始化改写为PIDRegister的形式,在controller统一分配内存.

#### user_lib

- [ ] 将所有通用的计算函数和常用函数汇集在此

#### dji_motor

- [ ] 增加3508和2006的开环零位校准函数
- [ ] 为实例增加低通滤波系数变量，使不同电机有不同的配置
- [ ] 正反转标志位设置,需要修改反馈量和pid计算

#### LKmotor

- [ ] 正反转标志位设置,需要修改反馈量和pid计算

#### HTmotor

- [ ] 正反转标志位设置,需要修改反馈量和pid计算

### 待添加

#### unicomm

- [ ] 完成初版构建

#### step_motor

- [ ] 增加步进电机模块

#### referee_communication

- [x] 增加裁判系统多机通信功能

#### controller

- [ ] 增加扰动观测器，可能需要新增模块
- [ ] 增加模型控制器，可能需要新增模块

#### ws2816

- [ ] 通过bsp_pwm添加支持







---





## APP

### 待完成

- [ ] 增加调试的条件编译，使得在没有连接其他应用时也可以假装有那些应用而正常调试运行

#### ==robot_cmd==

- [ ] 键鼠控制
- [x] 双板兼容（待测试）

#### ==chassis==

- [ ] 根据裁判系统的功率数据和超级电容，进行输出限幅
- [x] 双板兼容（待测试）

#### ==shoot==

- [ ] 增加卡弹检测和反转
- [ ] 弹仓盖控制（需待servo_motor完成）
- [ ] 42mm发射机构兼容



### 待优化

#### robot_cmd

- [x] 解耦各个应用的运行模式
- [ ] 优化消息发布和接收性能

#### gimbal

- [ ] 增加底盘速度前馈控制

#### chassis

- [ ] 根据电机的实际速度计算底盘的真实运动
- [ ] 若为双板，根据IMU的数据对电机实际速度进行融合

#### shoot

