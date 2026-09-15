# Side-mounted link-4 F/T assistance

Build and run from the repository root:

```sh
cmake -S . -B build
cmake --build build --target ft_link4_compensation -j2
./build/bin/ft_link4_compensation
```

The program uses the existing FT9236 calibration and Dev2/ai16:21 channels.
Keep the sensing side unloaded and stationary for startup tare. Assistance
starts disabled. `f` followed by Enter toggles assistance; `q` exits.
Gravity compensation stays enabled when assistance is disabled.

Live diagnostics use libbarrett `PrintToStream` each execution cycle, including while assistance is disabled:
- Sensor position relative to the robot base origin, in base axes (x y z), metres.
- Tared measured force in base axes (Fx Fy Fz), newtons.
- Tared measured moment in base axes (Mx My Mz), newton-metres, about the sensor
  origin (not about the base origin).

Move the arm with gravity compensation to inspect these readings before pressing
`f`. The monitor rotates all six measured components without the assistance gain,
filter. It explicitly accounts for any configured
world-to-base transform. Console output uses the same managed `PrintToStream` systems as the other
programs in this repository. These readings retain the startup
tare and attached-load limitations described below.

Edit `config/link4_sensor.conf` to change the sensor mounting.
Set `active_mounting` to `"positive_y"` (current: [0, 0.11, 0.17] m,
xs=z4, ys=x4, zs=y4) or `"negative_x"` ([-0.10, 0, 0.18] m,
xs=z4, ys=y4, zs=-x4). Both profiles are saved under `mountings`.
The selected profile name is printed at startup. `position_m` is
[x4, y4, z4] in metres. `rotation` is a 3-by-3 sensor-to-frame-4 rotation,
written row by row; its columns express the sensor x, y, z axes in frame 4.
The file is read once at startup, independently of the WAM configuration. Restart
after edits; rebuilding is unnecessary. Both assistance and diagnostics use the
same loaded mounting. Missing/malformed settings, nonfinite values, and rotations
that are not orthonormal with determinant +1 are rejected. The default file path
is set by CMake to this repository's config file and printed at startup.

The mounting frame is libbarrett's fourth moving-link DH frame (`kin.link[3]`),
not the frame immediately before joint 4. Verify that this is the frame used
for your measurements. The sensor origin and axes come from the selected mounting profile.

`link4_sensor_kinematics.hpp` uses the configured live link transforms and
constructs a world-expressed geometric Jacobian at the sensor origin. It includes
the mounting lever arm. Columns for joints 5–7 are zero. The rotation maps sensor
vectors into world axes directly; there is no tool-orientation transpose.

`link4_ft_assistance.hpp` uses the full wrench (force and measured moment), a 10 Hz
first-order filter on all six components, no force or moment deadband,
gain 1.0, a 4 Nm limit per
joint, and a 5 Nm/s command slew limit. These are starting parameters, not
hardware-validated settings. Filter state resets before each enable. Nonfinite
inputs produce zero assistance. The sign argument in the new executable is +1,
assuming the sensor measures force applied to the robot. Check the physical sign
with a small push before increasing assistance. Output goes through the WAM's
joint-torque reference controller; its gravity contribution remains separate.

The existing ATI reader provides startup voltage tare. This does not compensate
orientation-dependent gravity or inertia of a handle/plate on the sensing side;
no mass/centre-of-mass model has been supplied. The wrist itself does not load
this externally attached sensor through its normal structural support.

This executable reuses the existing ATI system, whose DAQ reads can block in the
execution loop. It does not add an asynchronous acquisition or stale-data watchdog.
Hardware behavior and real-time timing still need validation.

Hardware-independent kinematics check (assertions must remain enabled):

```sh
c++ -std=c++14 -Iinclude -I/usr/include/eigen3 tests/link4_sensor_kinematics_test.cpp -L/usr/local/lib -Wl,-rpath,/usr/local/lib -lbarrett -lconfig++ -lconfig -lgsl -lgslcblas -o /tmp/link4_sensor_kinematics_test
/tmp/link4_sensor_kinematics_test
```

The test compares linear and angular Jacobian columns to central finite
differences of the sensor pose at ten configurations, checks the mounting axis,
and checks zero wrist columns. It does not command the robot.

Force and moment are rotated into the Jacobian's world axes, with moments kept
about the sensor origin. Assistance is `gain * J_sensor.transpose() * wrench`.
No extra lever-arm moment is added: the sensor-point Jacobian already includes
the mounting offset. Combined force/moment output is limited together, and joints
5–7 still receive zero assistance. The sign setting applies to all six components.

A separate regular-Jacobian null-space case is available as
`ft_link4_nullspace`; see [LINK4_NULLSPACE.md](LINK4_NULLSPACE.md).
