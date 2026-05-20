This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## fkie_iop_handoff: HandoffController

The Handoff Controller service runs on an entity such as an OCU that support handing off control.

#### Parameter:

_enhanced_timeout (int_, (Default: 10)

> Timeout in seconds.

_auto_request (bool_, (Default: false])

> Requests automatically handoff on INSUFFICIENT_AUTHORITY.

_auto_authority (int_, (Default: 255])

> Authority for auto requests.

_auto_explanation (str_, (Default: ""])

> Explanation text for auto requests.


#### Publisher:

_handoff_remote_response (fkie_iop_msgs::HandoffResponse)_

> Publishes the remote response as result to own requests.

_handoff_remote_request (handoff_remote_request)_

> Publishes the remote requests to local ROS-OCU.

#### Subscriber:

_handoff_own_request (fkie_iop_msgs::HandoffRequest)_

> Listen for handoff requests from ROS-OCU.

_handoff_own_response (fkie_iop_msgs::HandoffResponse)_

> Listen for response from ROS-OCU to remote requests.

