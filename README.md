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

Currently without function, register only the service and reports that it is operational.

#### Parameter:

> None

#### Publisher:

> None

#### Subscriber:

> None


## iop_platform_state_fkie: PlatformStateClient

This service can be used by other service to query or set the state of the platfrom.

#### Parameter:

> None

#### Publisher:

> None

#### Subscriber:

> None



