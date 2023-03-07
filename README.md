## 文件夹简介
`cmake` : cmake 文件\
`common`: 状态估计及触地检测, 贝塞尔曲线轨迹生成, 步态生成, 机器人的动力学模型\
`config`: cheetah配置文件\
`documentation`: 模拟器和机器人本体的getting started文档\
`lcm_types`: 自定义的LCM消息类型\
`resources`: 模拟器用到的CAD文件\
`robot`: 机器人的底层SPI和SBUS通信, 部分进程是硬实时的\
`scripts`: 机器人初始化和LCM消息生成脚本\
`sim`: 模拟器\
`third-party` : 第三方库\
`user`: 控制器和状态机\

## user内容
`Example_Leg_InvDyn`: 腿的逆动力学示例, 即上文说的接触腿的力控制器\
`JPos_Controller`: 关节位置控制示例， 即上文所说的摆动腿轨迹跟踪控制器, 值得注意的是, 这里使用了三角函数成作为轨迹生成器, 而正式使用的是贝塞尔曲线。

`MIT_Controller`:是最核心最复杂的控制器, 分为:
`FSM`: 有限状态机, 负责机器人各个状态: 站起, 站定时平衡, 腿末端阻抗, 腿被动, 腿关节PD+前馈, 各种步态的切换及控制.
`BlanceController`: QP平衡控制器
`convexMPC`: 上文提到的凸MPC控制器

-------------------------------------------------——————————------——————-----------——————————————————————————————————————————————————————
## Cheetah-Software
This repository contains the Robot and Simulation software project.  For a getting started guide, see the documentation folder.

The common folder contains the common library with dynamics and utilities
The resources folder will contain data files, like CAD of the robot used for the visualization
The robot folder will contain the robot program
The sim folder will contain the simulation program. It is the only program which depends on QT.
The third-party will contain *small* third party libraries that we have modified. This should just be libsoem for Cheetah 3, which Pat modified at one point.

## Basic definetion
In `Quadruped.h`, a data structure containing parameters for quadruped robot is defined. This file contains the Quadruped class.  This stores all the parameters for a quadruped robot.  There are utility functions to generate Quadruped objects for Cheetah 3 (and eventually mini-cheetah). There is a buildModel() method which can be used to create a floating-base dynamics model of the quadruped.


## Build
To build all code:
```
mkdir build
cd build
cmake ..
./../scripts/make_types.sh
make -j4
```

If you are building code on your computer that you would like to copy over to the mini cheetah, you must replace the cmake command with
```
cmake -DMINI_CHEETAH_BUILD=TRUE
```
otherwise it will not work.  If you are building mini cheetah code one the mini cheetah computer, you do not need to do this.

This build process builds the common library, robot code, and simulator. If you just change robot code, you can simply run `make -j4` again. If you change LCM types, you'll need to run `cmake ..; make -j4`. This automatically runs `make_types.sh`.

To test the common library, run `common/test-common`. To run the robot code, run `robot/robot`. To run the simulator, run `sim/sim`.

Part of this build process will automatically download the gtest software testing framework and sets it up. After it is done building, it will produce a `libbiomimetics.a` static library and an executable `test-common`.  Run the tests with `common/test-common`. This output should hopefully end with

```
[----------] Global test environment tear-down
[==========] 18 tests from 3 test cases ran. (0 ms total)
[  PASSED  ] 18 tests.
```
## Run simulator
To run the simulator:
1. Open the control board
```
./sim/sim
```
2. In the another command window, run the robot control code
```
./user/${controller_folder}/${controller_name} ${robot_name} ${target_system}
```
Example)
```
./user/JPos_Controller/jpos_ctrl 3 s
```
3: Cheetah 3, m: Mini Cheetah
s: simulation, r: robot

## Run Mini cheetah
1. Create build folder `mkdir mc-build`
2. Build as mini cheetah executable `cd mc-build; cmake -DMINI_CHEETAH_BUILD=TRUE ..; make -j`
3. Connect to mini cheetah over ethernet, verify you can ssh in
4. Copy program to mini cheetah with `../scripts/send_to_mini_cheetah.sh`
5. ssh into the mini cheetah `ssh user@10.0.0.34`
6. Enter the robot program folder `cd robot-software-....`
7. Run robot code `./run_mc.sh` 



## Dependencies:
- Qt 5.10 - https://www.qt.io/download-qt-installer
- LCM - https://lcm-proj.github.io/ (Please make it sure that you have a java to let lcm compile java-extension together)
- Eigen - http://eigen.tuxfamily.org
- `mesa-common-dev`
- `freeglut3-dev`
- `libblas-dev liblapack-dev`

To use Ipopt, use CMake Ipopt option. Ex) cmake -DIPOPT_OPTION=ON ..
