## set on UP-xtreme
//Ubutnu 20.04 -> Ubuntu 18.04
1. in Arclab-Cheetah-Software/sim/CMakeList.txt

change
```
set(CMAKE_PREFIX_PATH /home/linjh/Qt/5.14.2/gcc_64)
set(QT5Core_DIR /home/linjh/Qt/5.14.2/gcc_64/lib/cmake/Qt5Core)
set(QT5Widgets_DIR /home/linjh/QT/5.14.2/gcc_64/lib/cmake/Qt5Widgets)
set(QT5Gamepad_DIR /home/linjh/QT/5.14.2/gcc_64/lib/cmake/Qt5Gamepad)
```
to
```
set(CMAKE_PREFIX_PATH /home/linupx/Qt5.14.2/5.14.2/gcc_64)
set(QT5Core_DIR /home/linupx/Qt5.14.2/5.14.2/gcc_64/lib/cmake/Qt5Core)
set(QT5Widgets_DIR /home/linupx/QT5.14.2/5.14.2/gcc_64/lib/cmake/Qt5Widgets)
set(QT5Gamepad_DIR /home/linupx/QT5.14.2/5.14.2/gcc_64/lib/cmake/Qt5Gamepad)
```

2. in /third-party/osqp/CMakelist.txt
search `-lrt` and insert `-lpthread` after it
```
set(CMAKE_C_STANDARD_LIBRARIES "${CMAKE_C_STANDARD_LIBRARIES} -lrt -ldl -lpthread")
```

3. in /scripts, run `./make_types.sh`, if there are some warnings and errors, don't worry
4. run `cp third-party/N100IMU/IMU_LCMt.lcm lcm-types/cpp/`
