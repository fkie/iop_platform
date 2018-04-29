This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## iop_client_digital_resource_fkie: DigitalResourceClient

Plugin service for OCU clients. It connects to the DigitalResourceDiscovery service on the robot and publishes the reported endpoints. Implements Slave interface.

#### Parameter:

> None

#### Publisher:

_digital_endpoints (std_msgs::DigitalResourceEndpoints)_, latched

> Publishes the discovered endpoints.

#### Subscriber:

> None
