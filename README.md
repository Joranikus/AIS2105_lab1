# PID Controller Node

## Starting Node

### Default Parameters 

> ros2 run pid_controller pid_controller_node

### Launch with custom Parameters

> ros2 run pid_controller pid_controller_node --ros-args -p p:=69 -p i:=69 -p d:=69 -p reference:=69

## Changing Parameters

### Set parameters while Node running

> ros2 param set /pid_controller_node p 69
> ros2 param set /pid_controller_node i 69
> ros2 param set /pid_controller_node d 69
> ros2 param set /pid_controller_node reference 69

## Information

### Parameters

> p (double)
> i (double)
> d (double)
> reference (double)

### Publishing Topics

> voltage (std_msgs/msg/Float64)

### Subscribed Topics

> measured_angle (std_msgs/msg/Float64)