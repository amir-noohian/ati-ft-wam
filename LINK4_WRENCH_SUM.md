# Sensor torque minus end-effector remapping

New executable: `ft_link4_wrench_sum`. The direct and null-space cases are retained.
The existing executable name is retained, but the mapping now subtracts the
returned torque, as requested.

```sh
cmake -S . -B build
cmake --build build --target ft_link4_wrench_sum -j2
./build/bin/ft_link4_wrench_sum
```

Run from your terminal. Assistance starts disabled with gravity compensation on.
Keep the sensor unloaded/still during startup tare. `f` + Enter toggles assistance;
`q` + Enter exits. Continuous sensor position and F/T prints are temporarily
disabled so enable/fault messages remain visible. Uncomment the two monitor-to-
PrintToStream connections in `src/ft_link4_wrench_sum.cpp` to restore them.

## Equations

All wrenches and Jacobians use matching world axes. The sensor wrench moment is
about the sensor origin, and the recovered end-effector wrench moment is about
libbarrett's configured tool origin. Let the signed/filtered sensor wrench be W.

1. `tau_initial = gain * J_sensor^T * W` (seven entries). Explicitly set entries
   5–7 to zero, consistent with a sensor attached after joint 4.
2. `W_ee = (J_ee^T)^+ * tau_initial`. This solves the seven-equation least-squares
   problem `J_ee^T W_ee ~= tau_initial`, including all three zero wrist torques.
   `J_ee` is 6-by-7, so an ordinary matrix inverse does not exist. The appropriate
   torque-to-wrench map is the pseudoinverse of its transpose, not `J_ee^+`.
3. `tau_returned = J_ee^T * W_ee` (all seven joints).
4. `tau_desired = tau_initial - tau_returned`.
5. Apply assistance limits to the combined torque, then send it as the WAM torque
   reference alongside its gravity compensation.

The SVD is `J_ee = U Sigma V^T`, so step 2 is evaluated as
`W_ee = U Sigma^-1 V_6^T tau_initial` when the task has full row rank.
Generally, no end-effector wrench reproduces the initial seven joint torques
exactly: the least-squares residual lies in the regular Jacobian null space.
The returned torque may have nonzero wrist entries; these are NOT zeroed.

Equivalently, `tau_desired = (I - J_ee^T (J_ee^T)^+) tau_initial`.
The end-effector-representable component is removed. Before output limits this
is equivalent to the full-pose regular null-space projector `(I - J_ee^+ J_ee)`.
It does not guarantee zero end-effector acceleration because no inertia matrix
is used. There is no end-effector holding controller.

## Configuration and limits

- Mounting: `config/link4_sensor.conf`, shared with the other cases.
- This case's settings: `config/link4_wrench_sum.conf`.
- Gain 1.0, sign +1, 10 Hz filter on all six components, no deadband.
- Combined assistance limit: 4 Nm per joint. Normal-cycle slew bound: 5 Nm/s.
- Full six-axis end-effector wrench only; no position-only task selector.
- SVD rejects `sigma_min <= 1e-6 * sigma_max` by default.

Limits clip the desired combined torque to ±4 Nm per joint and then ramp each
joint independently at up to 5 Nm/s. Direction changes and force release no longer
fault merely because a common scalar cannot satisfy all slew bounds. During
limiting, the applied vector can differ in direction from the requested residual and
lose its null-space property.
Fault 1 is nonfinite input; fault 2 is a singular/near-singular Jacobian or invalid
remapping; fault 3 is invalid limiter input/output. Faults latch until reset;
the console disconnects assistance, and `f` can reset/re-enable it. Manual disable
and fault shutdown drop immediately to zero, bypassing the slew bound. Gravity
compensation stays active. Settings load at startup; restart after configuration
edits without rebuilding.

## Validation and limitations

Offline tests verify the seven-equation least-squares result, subtraction and the pre-limit null-space residual,
nonzero returned wrist torque, range/null-space examples, amplitude and slew
bounds, small inputs, moment input, faults, and mapping at 100 WAM configurations.

```sh
c++ -std=c++14 -O2 -Iinclude -I/usr/include/eigen3 tests/link4_wrench_sum_test.cpp -L/usr/local/lib -Wl,-rpath,/usr/local/lib -lbarrett -lconfig++ -lconfig -lgsl -lgslcblas -o /tmp/link4_wrench_sum_test
/tmp/link4_wrench_sum_test
```

The robot has not been run for this case. Blocking DAQ reads and per-cycle console
printing remain; no sample-age watchdog, attached-load model, or stability proof
is provided. The custom ATI reader still ignores BasicTransform, so the moment
reference origin needs verification. WAM safety settings are separate from the
assistance limit. The shared sensor configuration and prior executables are not
modified by this new case.
