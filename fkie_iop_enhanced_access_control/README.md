This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).

## fkie_iop_enhanced_access_control: EnhancedAccessControl

The EnhancedAccessControl service extends Access Control to allow for handoff of control from one client to another.

#### Parameter:

_decision_timeout (int_, (Default: 60)

> The request timeout is the amount of time that must pass from when this service first requests a handoff from the current controlling client before the requester is notified that the handoff failed due to a timeout.

_request_timeout (int_, (Default: 10)

> Clients must re-request handoff to prevent being denied handoff request when the timeout expires. A value of zero indicates this feature is disabled.

#### Publisher:

> None

#### Subscriber:

> None
