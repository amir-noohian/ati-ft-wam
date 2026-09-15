# Regular-Jacobian null-space assistance (new case)

The original `ft_compensation` and `ft_link4_compensation` programs remain available.
The new `ft_link4_nullspace` executable is for a 7-DOF WAM only. It adds sensor-based
force/moment assistance through a regular Jacobian null-space projector. It does
not capture or hold any end-effector position or orientation target. The operator
can physically guide the end effector while gravity compensation remains active.

```sh
cmake -S . -B build
cmake --build build --target ft_link4_nullspace -j2
./build/bin/ft_link4_nullspace
```

Run it yourself in the robot terminal. Follow libbarrett startup prompts and keep
the sensor unloaded/still during tare. Assistance starts disabled. `f` + Enter
toggles it; `q` + Enter exits. Sensor position and full F/T in base axes use the
same PrintToStream diagnostics as the previous case.

## Configuration

`config/link4_sensor.conf` selects the mounting, shared with the prior case.
`config/link4_nullspace.conf` controls only this new case. Both load at startup;
restart after edits, without rebuilding.

- `task = "pose"` (default): full six-row end-effector Jacobian; normally one
  null-space dimension. This is the default assumption for keeping the added
  assistance out of both position and orientation Jacobian directions.
- `task = "position"`: only the first three (linear) rows; normally four
  null-space dimensions. The assistance may influence orientation.
- `gain = 1.0`, `sign = 1.0`, `filter_hz = 10.0`.
- `torque_limit_nm = 4.0`: magnitude limit per joint on added assistance only.
- `slew_limit_nm_s = 5.0`: normal-cycle per-joint rate bound.
- `relative_rank_tolerance = 0.000001`: singular-value rejection threshold.

There is no force or moment deadband, no position-holding controller, and no
additional velocity damping. Configuration values are validated at startup.

## Calculation

1. Compute sensor pose and its 6-by-7 sensor-point Jacobian from live WAM kinematics.
2. Rotate tared force and moment into the Jacobian's world axes, retaining the
   moment about the sensor origin. Apply sign and the 10 Hz filter.
3. Form the unprojected assistance `tau_raw = gain * J_sensor^T * wrench`.
4. Read the live end-effector Jacobian at libbarrett's configured tool origin.
5. Compute `N = I - J_task^+ J_task` using a fixed-size SVD. The implementation
   forms `V_null * V_null^T`, an equivalent orthogonal projector.
6. Form `tau_desired = N * tau_raw`. All seven joints may contribute after projection.
7. Scale the entire torque vector by a common scalar in [0,1] to satisfy amplitude
   and slew bounds. There is no independent component clipping after projection.

No mass matrix or dynamically consistent inverse is used. The projector satisfies
`J_task * tau_assist ≈ 0` algebraically. A torque vector is not a joint velocity:
this does **not** imply zero end-effector acceleration. In general,
`J_task * M^-1 * tau_assist` is nonzero. Physical operator forces, gravity-model
errors, motion-dependent dynamics, and delays can also move the end effector.
This case therefore does not guarantee dynamically isolated assistance.

## Limiting and faults

A common scale preserves the current null-space direction. If that direction
changes too much, or desired assistance falls sharply, there may be no scalar
that also meets the previous-command slew bounds. This conservative limiter is
not a general constrained optimizer; a feasible torque elsewhere in a larger
null space may exist even when this scalar method fails.

In these cases output immediately becomes zero and a fault latches:

1. Nonfinite input.
2. Singular/near-singular selected task: smallest required singular value is at
   most `relative_rank_tolerance` times the largest. The controller does not
   silently add more null directions near a singularity.
3. No feasible common scalar for torque/slew bounds, or nonfinite desired torque.

The console loop reports the fault and disconnects assistance; gravity compensation
continues. `f` resets/re-enables it. Zeroing on a fault or manual disable bypasses
the slew bound. The default full-pose mode may reject straight-arm configurations.
The singularity threshold compares the numerical Jacobian in metres/radians;
it is not a unit-independent physical manipulability measure.

## Validation and remaining limitations

Build and offline tests cover projector symmetry/idempotence, both task choices,
`J_task * N`, regular versus inertia-weighted behavior, singularity rejection,
combined amplitude/slew limits, small wrench inputs, moments, wrist contributions,
fault/reset behavior, and 100 moving configurations per task using the repository's
WAM kinematics. Run assertions-enabled tests from the repository root:

```sh
c++ -std=c++14 -O2 -Iinclude -I/usr/include/eigen3 tests/link4_nullspace_test.cpp -L/usr/local/lib -Wl,-rpath,/usr/local/lib -lbarrett -lconfig++ -lconfig -lgsl -lgslcblas -o /tmp/link4_nullspace_test
/tmp/link4_nullspace_test
```

This is not hardware validation or a closed-loop stability guarantee. It retains
the earlier custom ATI reader: blocking DAQ reads, startup tare only, no sample-age
watchdog, and no attached-load gravity/inertia model. The calibration BasicTransform
is still not applied; the measured moment reference point remains to be verified.
PrintToStream performs console output each execution cycle. Real-time timing and
physical behavior must be evaluated on the robot. There are no new workspace,
position, or speed limits; WAM safety settings remain separate.
