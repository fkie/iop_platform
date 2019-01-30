This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## fkie_iop_platform_mode: PlatformMode

Register the service and reports parameterised values to control.

#### Parameter:

_platform_mode (int_, (Default: 0)

> Current mode: 0: "Standard_Operating", 1: "Training", 2: "Maintenance"

_supported_modes (list_, (Default: [0])

> List of supported modes.

#### Publisher:

_platform_mode (std_msgs::UInt8)_, latched

> Current platform mode

#### Subscriber:

_set_platform_mode (std_msgs::UInt8)_

> Change current platform mode. The service forwards the value without checks.

