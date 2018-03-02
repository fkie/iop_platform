See [iop_core](https://github.com/fkie/iop_core/blob/master/README.md) for use instructions.

# Interfaces

List of service plugins in this repository:

[iop_client_digital_resource_fkie: DigitalResourceClient](#iop_client_digital_resource_fkie-digitalresourceclient)  
[iop_digital_resource_discovery_fkie: DigitalResourceDiscovery](#iop_digital_resource_discovery_fkie-digitalresourcediscovery)  
[iop_digital_resource_discovery_fkie: DigitalResourceDiscoveryClient](#iop_digital_resource_discovery_fkie-digitalresourcediscoveryclient)  
[iop_health_monitor_fkie: HealthMonitor](#iop_health_monitor_fkie-healthmonitor)  
[iop_platform_mode_fkie: PlatformMode](#iop_platform_mode_fkie-platformmode)  
[iop_platform_state_fkie: PlatformState](#iop_platform_state_fkie-platformstate)  
[iop_platform_state_fkie: PlatformStateClient](#iop_platform_state_fkie-platformstateclient)  
[iop_unsolicited_heartbeat_fkie: UnsolicitedHeartbeat](#iop_unsolicited_heartbeat_fkie-unsolicitedheartbeat)  


## iop_client_digital_resource_fkie: DigitalResourceClient

Plugin service for OCU clients. It connects to the DigitalResourceDiscovery service on the robot and publishes the reported endpoints. Implements Slave interface.

#### Parameter:

> None

#### Publisher:

_digital_endpoints (std_msgs::DigitalResourceEndpoints)_, latched

> Publishes the discovered endpoints.

#### Subscriber:

> None


## iop_digital_resource_discovery_fkie: DigitalResourceDiscovery

Service to manage the digital resource endpoint of a robot. This service should be run inside of a platform node and only once in a subsystem.

#### Parameter:

> None

#### Publisher:

> None

#### Subscriber:

> None


## iop_digital_resource_discovery_fkie: DigitalResourceDiscoveryClient

This client offers methods to register digital endpoints by DigitalResourceDiscovery service. This is used e.g. by DigitalVideo service.

#### Parameter:

> None

#### Publisher:

> None

#### Subscriber:

> None


## iop_health_monitor_fkie: HealthMonitor

Currently without function, register only the service.

#### Parameter:

> None

#### Publisher:

> None

#### Subscriber:

> None


## iop_platform_mode_fkie: PlatformMode

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


## iop_unsolicited_heartbeat_fkie: UnsolicitedHeartbeat

Automatically generate periodic Report Heartbeat Pulse messages.

#### Parameter:

_hz (double_ Default: 1.0)

> Specifies the rate at which the unsolicited Report Heartbeat Pulse message is send, in Hertz. 0 disables the send.

#### Publisher:

> None

#### Subscriber:

> None


