# PID Controller Node

## Starting Node

### Default Parameters 

```bash
ros2 run pid_controller pid_controller_node
```

### Launch with custom Parameters

```bash
ros2 run pid_controller pid_controller_node --ros-args -p p:=69 -p i:=69 -p d:=69 -p reference:=69
```

## Changing Parameters

### Set parameters while Node running

```bash
ros2 param set /pid_controller_node p 69
```

```bash
ros2 param set /pid_controller_node i 69
```

```bash
ros2 param set /pid_controller_node d 69
```

```bash
ros2 param set /pid_controller_node reference 69
```

## Information

### Parameters

```bash
p (double)
```

```bash
i (double)
```

```bash
d (double)
```

```bash
reference (double)
```

### Publishing Topics

```bash
voltage (std_msgs/msg/Float64)
```

### Subscribed Topics

```bash
measured_angle (std_msgs/msg/Float64)
```