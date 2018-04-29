This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## iop_platform_state_fkie: PlatformState

Supports state changes of whole robot. On set emergency all components with management service are set into emergency state.

#### Parameter:

_init_platform_state (int_, (Default: 1)

> Default state on start: 0: "initialize", 1: "operational", 2: "shutdown", 3: "system_abort", 4: "emergency", 5: "render_useless"

_supported_states ([string]_, (Default: ["OPERATIONAL", "EMERGENCY"])

> A list with supported states. Possible entries: "initialize", "operational", "shutdown", "system_abort", "emergency", "render_useless"

#### Publisher:

_set_platform_state (std_msgs::UInt8)_

> Publishes the state to set to ROS.

_set_platform_state_str (std_msgs::String)_

> Like `set_platform_state` with state as string.

#### Subscriber:

_robot_platform_state (std_msgs::UInt8)_

> ROS componets can report the current state of the robot by publishing the state to this topic. This state will be only reported to control client!

_robot_platform_state_str (std_msgs::String)_

> The same as `robot_platform_state` with string type.


## iop_platform_state_fkie: PlatformStateClient

This service can be used by other service to query or set the state of the platfrom.

#### Parameter:

_hz (double_ Default: 0.0)

> Specifies the rate at which state is reported from robot, in Hertz. By 0 tries to create an event to get states on change.

#### Publisher:

_platform_state (std_msgs::UInt8)_

> Publishes reported state of robot to ROS.

_platform_state_str (std_msgs::String)_

> Like `platform_state_str` with state as string.

#### Subscriber:

_cmd_platform_state (std_msgs::UInt8)_

> Forwards the commands to change the state to the robot. You need a granded access to be able to change the state!

_cmd_platform_state_str (std_msgs::String)_

> Like `cmd_platform_state_str` with state as string.
