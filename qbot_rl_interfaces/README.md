# qbot_rl_interfaces

Message and action definitions shared by `qbot_rl_deploy` nodes.


## `ObservationStamped.msg`

Published by `observation_builder`, consumed by `navigate_action_server`.

Carries the flattened observation vector plus self-describing layout metadata: component names, start indices, lengths, and per-component source timestamps, alongside the overall message header.

The message is self-describing rather than a bare `float32[]` so a consumer can assert the observation layout matches what the policy model expects at startup, rather than silently misaligning if `observation_builder`'s component order ever deviates from the trained policy's expected input order. Per-component timestamps also enable per-component staleness/latency checks independent of the overall message timestamp.


## `PolicyNavigate.action`

Used between `navigate_action_client` (or any custom client) and `navigate_action_server`. The goal carries the client-specified termination conditions (timeout and success radius). Feedback reports running time, distance to goal, and heading error. The result reports success, a status message, and final distance to goal.

`timeout_sec` and `success_radius_m` are client-specified per call.

## License

Apache-2.0