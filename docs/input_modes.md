# Input Modes
As of version ###, ODrive now intercepts the incoming commands and can apply filters to them.  The old protocol values `pos_setpoint`, `vel_setpoint`, and `current_setpoint` are still used internally by the closed-loop cascade control, but the user cannot write to them directly.  This allows us to condense the number of ways the ODrive accepts motion commands.  The new commands are:

* `<axis>.controller.config.input_mode`
* `<axis>.controller.input_pos`
* `<axis>.controller.input_vel`
* `<axis>.controller.input_current`

The Input Modes currently valid are:
* `INPUT_MODE_INACTIVE`
* `INPUT_MODE_PASSTHROUGH`
* `INPUT_MODE_VEL_RAMP`
* `INPUT_MODE_POS_FILTER`
* `INPUT_MODE_MIX_CHANNELS`
* `INPUT_MODE_TRAP_TRAJ`
* `INPUT_MODE_CURRENT_RAMP`
* `INPUT_MODE_MIRROR`

---

## INPUT_MODE_INACTIVE
Disable inputs.  Setpoints retain their last value.

## INPUT_MODE_PASSTHROUGH
Pass `input_xxx` through to `xxx_setpoint` directly.

### Valid Inputs:
* `input_pos`
* `input_vel`
* `input_current`

### Valid Control modes:
* `CTRL_MODE_VOLTAGE_CONTROL`
* `CTRL_MODE_CURRENT_CONTROL`
* `CTRL_MODE_VELOCITY_CONTROL`
* `CTRL_MODE_POSITION_CONTROL`

## INPUT_MODE_VEL_RAMP
Ramps a velocity command from the current value to the target value.

### Configuration Values:
* `<axis>.controller.config.vel_ramp_rate` [cpr/sec]
* `<axis>.controller.config.inertia` [A/(count/s^2))]

### Valid inputs:
* `input_vel`

### Valid Control Modes:
* `CTRL_MODE_VELOCITY_CONTROL`

## INPUT_MODE_POS_FILTER
Implements a 2nd order position tracking filter.  Inteded for use with step/dir interface, but can also be used with position-only commands.

![POS Filter Response](secondOrderResponse.PNG)
Result of a step command from 1000 to 0

### Configuration Values:
* `<axis>.controller.config.input_filter_bandwidth`
* `<axis>.controller.config.inertia`

### Valid inputs:
* `input_pos`

### Valid Control modes:
* `CTRL_MODE_POSITION_CONTROL`

## INPUT_MODE_MIX_CHANNELS
Not Implemented.


## INPUT_MODE_TRAP_TRAJ
Implementes an online trapezoidal trajectory planner.

![Trapezoidal Planner Response](TrapTrajPosVel.PNG)

### Configuration Values:
* `<axis>.trap_traj.config.vel_limit`
* `<axis>.trap_traj.config.accel_limit`
* `<axis>.trap_traj.config.decel_limit`
* `<axis>.controller.config.inertia`

### Valid Inputs:
* `input_pos`

### Valid Control Modes:
* `CTRL_MODE_POSITION_CONTROL`

## INPUT_MODE_CURRENT_RAMP
Ramp a current command from the current value to the target value.

### Configuration Values:
* `<axis>.controller.config.current_ramp_rate`

### Valid Inputs:
* `input_current`

### Valid Control Modes:
* `CTRL_MODE_CURRENT_CONTROL`

## INPUT_MODE_MIRROR
Implements "electronic mirroring".  This is like electronic camming, but you can only mirror exactly the movements of the other motor, according to a fixed ratio

[![](http://img.youtube.com/vi/D4_vBtyVVzM/0.jpg)](http://www.youtube.com/watch?v=D4_vBtyVVzM "Example Mirroring Video")

### Configuration Values
* `<axis>.controller.config.axis_to_mirror`
* `<axis>.controller.config.mirror_ratio`

### Valid Inputs
* None.  Inputs are taken directly from the other axis encoder estimates

### Valid Control modes
* `CTRL_MODE_POSITION_CONTROL`
