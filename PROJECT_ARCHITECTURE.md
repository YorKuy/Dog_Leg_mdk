# Dog Leg firmware structure

The project follows the same application-wrapper layout as `twitch_chassic`.
CubeMX-generated hardware code stays under `Core`, while application behavior
and control algorithms are kept in separate modules.

## Application layer

- `App_Lib/main.cpp`: firmware entry point, peripheral initialization order and
  HAL callback forwarding.
- `App_Lib/app_chassis.cpp`: chassis objects, operating modes, CAN application
  handling, keyboard logic, power limiting and UI scheduling.
- `App_Lib/app_chassis.h`: C-compatible application lifecycle and callback API.

## Leg control layer

- `Control_Lib/leg.h`: stable public API, input/output structures and Ozone
  tuning variables.
- `Control_Lib/leg.cpp`: public API implementation and 1 kHz mode coordinator.
- `Control_Lib/leg_balance.cpp`: physical pitch SMC, acceleration feedforward,
  physical roll balance, slope state machine and height damping/hold.
- `Control_Lib/leg_output.cpp`: mirrored motor mixing, gas-spring-aware joint
  damping, torque limiting and direction-aware torque slew.
- `Control_Lib/leg_height.cpp`: FAST/PLAYER manual leg-height cascade and its
  continuous feedforward.
- `Control_Lib/leg_internal.h`: private mechanical constants, shared state and
  interfaces used only between the leg implementation modules.

The leg encoder convention is fixed: both `*_MIN_MANG` values are the fully
retracted drawing pose and both `*_MAX_MANG` values are the mechanical extension
limits. The mirrored right encoder decreases while extending, so physical leg
direction must always be normalized with `(angle - MIN) / (MAX - MIN)`.

## Hardware layer

- `Core/Src/main.c`: CubeMX-compatible clock configuration, fallback entry and
  error handler only.
- `Core/Src/*.c`, `Drivers/`: generated peripheral and HAL implementation.

Both CMake and `MDK-ARM/New_Chassis.uvprojx` include the same application and
leg-control modules. Do not place control behavior back into CubeMX-generated
files; add it to the corresponding `App_Lib` or `Control_Lib` module instead.
