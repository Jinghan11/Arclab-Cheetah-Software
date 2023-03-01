# Running Arclab Cheetah
First turn on computer power.  It takes around 2 minutes for the computer to boot.
SSH into the computer with `ssh user@xx.xx.xx.xx`. You must set your computer's IP address to something like `10.0.0.xxx` for this to work.

Next, set up your computer for LCM.  In the scripts folder, run `./config_network_lcm.sh -I ` followed by the name of your network adapter you're connecting with. If you want, you can edit the script to add a shortcut for your specific network adapter. For Arclab Cheetah Software, the following shortcut has been added.
```
if [ "$1" == "lin" ]; then
    sudo ifconfig wlp0s20f3 multicast
    sudo route add -net 10.0.0.0 netmask 255.0.0.0 dev wlp0s20f3
```
`wlp0s20f3` here is my NIC name on Arclab Cheetah. You can use `ifconfig`to find the name of your NIC on your computer. 

To build code which can run on the mini cheetah. After your first make, you can use `cmake ..` in subsitute, the config will keep in the future.
- `mkdir mc-build && cd mc-build`
- `cmake -DMINI_CHEETAH_BUILD=TRUE ..`
- `make -j8`
  


To copy the code to the robot, you need to set your controller path IP address in `../scripts/send_to_mini_cheetah.sh`. Then, you can run it and check the file in your computer via SSH.

If you would like to open LCM spy, you can do this by running `../script/launch_lcm_spy.sh`.  If you receive an error about Java, try running `../scripts/launch_lcm_spy_jdk_fix.sh`, which has modified launch arguments to support newer versions of the JVM.


On the mini cheetah, you will find a folder in the home directory name `robot-software` and the date.  

To run the robot code, enter the build folder and run 
- `./run_mc.sh jpos_ctrl`
  
Remind that the NIC name and multicast address should be changed firstly.


===========================================================
The following has not set up yet in Arclab Cheetah. (In progress)

The robot controller currently works with:

- State estimate (orientation only), access with `_stateEstimate`
- Leg control (torque feedforward, force feedforward, PD control cartesian, PD control joint), access through LegCommand.  Note that the leg command is zeroed for you on each iteration.
- Leg data (joint position/velocity, cartesian position/velocity in hip frame, jacobian in hip frame)
- Gamepad data
- Main Visualization (this is just a single cheetah model, the rest is still being implemented)
- Control parameters (set in the simulator gui)
- Configuration files (using the `THIS_COM`, or relative paths to the build folder)


It does not work with
- Full state estimator (position, velocity)
- Full visualization data
- Cheater state when running on the robot

Current LCM streams
- Raw spi data
- Raw vectornav data
- gamepad
- main visualization

Currently missing LCM streams
- Generic leg data
- State estimate
- Probably others I am forgetting

