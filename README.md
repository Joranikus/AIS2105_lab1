# PID Controller Node

## Starting Node

### Default Parameters 

```bash
ros2 run pid_controller pid_controller_node
```

### Launch with custom Parameters

```bash
ros2 run pid_controller pid_controller_node --ros-args -p p:=69.0 -p i:=69.0 -p d:=69.0 -p reference:=69.0
```

## Changing Parameters

### Set parameters while Node running

```bash
ros2 param set /pid_controller_node p 69.0
```

```bash
ros2 param set /pid_controller_node i 69.0
```

```bash
ros2 param set /pid_controller_node d 69.0
```

```bash
ros2 param set /pid_controller_node max_votage 69.0
```

```bash
ros2 param set /pid_controller_node min_voltage 69.0
```

```bash
ros2 param set /pid_controller_node reference 69.0
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
max_voltage (double)
```

```bash
max_voltage (double)
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

# Joint Simulator Node

## Starting Node

### Default Parameters 

```bash
ros2 run joint_simulator joint_simulator_node
```

### Launch with custom Parameters

```bash
ros2 run joint_simulator joint_simulator_node --ros-args -p K:=200.0 -p T:=0.2 -p noise:=0.1
```

## Changing Parameters

### Set parameters while Node running

```bash
ros2 param set /joint_simulator_node K 69.0
```

```bash
ros2 param set /joint_simulator_node T 69.0
```

```bash
ros2 param set /joint_simulator_node noise 69.0
```

## Information

### Parameters

```bash
K (double)
```

```bash
T (double)
```

```bash
noise (double)
```


### Publishing Topics

```bash
measured_angle (std_msgs/msg/Float64)
```

### Subscribed Topics

```bash
voltage (std_msgs/msg/Float64)
```
