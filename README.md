See [iop_core](https://github.com/fkie/iop_core/blob/master/README.md) for use instructions.

# Interfaces

List of service plugins in this repository:
```
iop_client_digital_resource_fkie: DigitalResourceClient
iop_digital_resource_discovery_fkie: DigitalResourceDiscovery
iop_digital_resource_discovery_fkie: DigitalResourceDiscoveryClient
iop_health_monitor_fkie: HealthMonitor
iop_platform_mode_fkie: PlatformMode
iop_platform_state_fkie: PlatformState
iop_platform_state_fkie: PlatformStateClient
```

## iop_client_digital_resource_fkie: DigitalResourceClient

Plugin service for OCU clients. Implements Slave interface.

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

Currently without function, register only the service.

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

# To be continued...


