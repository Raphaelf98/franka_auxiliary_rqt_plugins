# Franka reset button
 This rqt plugin integrates with franka_ros and will subscribe to franka error states (/franka_state_controller/franka_states).
 If an error state or a user stop event occurs a recovery button is available that sends a recovery message to the error_recovery (/franka_control/error_recovery/goal) topic.
# install
Source catkin workspace and run:
```
cd catkin_ws/src/
git clone https://git.tu-berlin.de/rbo/robotics/franka_auxiliary_rqt_plugins.git

```
After package is cloned run:
```
catkin_make
```

# run
The plugin can be run with the following commands:
```
source devel/setup.bash
```
and launch rqt with:
```
rqt
```
Inside the rqt user interface go to Plugins, Franka Auxiliary Plugins and open Reset Button.
