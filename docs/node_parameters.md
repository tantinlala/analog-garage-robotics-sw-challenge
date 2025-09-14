# Estop Monitor Parameters

- trigger_time_ms - Number of milliseconds after starting up at which to publish an estop triggered message (the estop node will have published an estop cleared message immediately on startup)

# Proximity Sensor Parameters

- distance_series - A list of distances that the simulated proximity sensor will publish in sequence.
- sample_time_ms - Period at which to publish the next proximity distance in the list.

# Speed Limiter Parameters

The stop_boundary, slow_boundary, and hysteresis parameters are used to enforce the following rules in the [state machine](speed_limiter_states.md):

## If the present state is "STOP"

x < (stop_boundary + hysteresis)
- Next state is "STOP"

(stop_boundary + hysteresis) <= x < (slow_boundary + hysteresis)
- Next state is "SLOW"

(slow_boundary + hysteresis) <= x
- Next state is "FULL_SPEED"

## If the present state is "SLOW"

x <= stop_boundary
- Next state is "STOP"

stop_boundary < x < (slow_boundary + hysteresis)
- Next state is "SLOW"

(slow_boundary + hysteresis) <= x
- Next state is "FULL_SPEED"

## If the present state is "FULL_SPEED"

x <= stop_boundary
- Next state is "STOP"

stop_boundary < x <= slow_boundary
- Next state is "SLOW"

slow_boundary < x
- Next state is "FULL_SPEED"
