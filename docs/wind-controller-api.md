# External wind controllers

Plugin interface **2.1** exposes `registerWindController`,
`unregisterWindController`, and `isWindControllerActive`, with the version 2
`WindController` descriptor. Existing 2.0
listener methods and events retain their layouts and virtual-method order.
Consumers must check the interface version before using the new methods.

Register for FSMP's `MSG_STARTUP` through SKSE messaging during PostLoad,
then pass a `WindController` descriptor to the interface. All three wind
interface methods must be called on the game thread, outside FSMP callbacks.
Registration reserves exclusive ownership; another controller is rejected.
The controller's `isEnabled` callback selects external or native wind before
each frame's actor shelter checks. Registration, removal, and active-state
changes use FSMP's existing simulation synchronization before changing
ownership. Unregister immediately revokes ownership once the running
simulation finishes. Native weather wind refreshes on the next enabled wind
tick.

Keep the original descriptor address as the registration identity, and keep
its context and callback code alive until unregister returns. FSMP copies
the descriptor and display name. Registration does not change the master
wind switch. The Wind menu displays the active controller's name.
`isWindControllerActive` reports selection for the current frame, not
registration alone; the master wind switch can still suppress forces.

## Callbacks

- `isEnabled` runs on the game thread before shelter checks and simulation.
  Return false while disabled or while the external source is unavailable.
  FSMP uses native wind for that frame without removing the registration,
  and automatically returns to external wind once this callback returns true.
- `sampleShelter` runs during actor updates on the game thread. Supply the
  ambient wind vector at the supplied ray origin. FSMP checks the direction
  and performs its existing shelter raycast and attenuation. Return false
  when no direction is available; the previous shelter factor is retained.
- `evaluate` runs once per simulated update, on FSMP's simulation worker.
  It receives the active wind-sensitive systems and movable bones, with
  positions restored to Skyrim world space. Return one force and torque
  per bone. Outputs start at zero; returning false discards the whole output.

There is no additional wind mutex around callbacks. Do not call back into
FSMP APIs or wait for the game thread from any callback.
Do not retain the arrays or pointers passed to `evaluate`. Plugins should
copy their UI settings under their own synchronization before evaluating.
No Bullet headers, object layouts, or private FSMP classes are needed.

The full system list is supplied on each evaluated frame. IDs remain stable
while a system is attached to the world and are not reused within that world.
A reattached or recreated system receives a new ID. Drop per-system state
when its ID disappears; there are no callbacks while simulation or wind is
disabled, so an empty scene may not produce an empty frame immediately.

## Units and response

Positions are in Skyrim units and linear velocities in Skyrim units per
second. `windStrength` is the existing FSMP setting; `unitsPerMeter` exposes
FSMP's unit conversion. System `shelterFactor` is the current smoothed
ambient attenuation. Body `windFactor` is the XML wind sensitivity.
System `position` is the arithmetic centroid of its included movable,
wind-sensitive bodies. Body-array indices are valid only for the current frame.

The controller owns its response model, including any use of these factors.
FSMP does not apply them a second time. Forces and torques use Bullet's
mass/Skyrim-unit/second convention. They are integrated across the update's
substeps; do not multiply force or torque by delta time. An off-center force
can be represented as a central force plus `offset cross (force * linearFactor)` torque,
where `* linearFactor` is component-wise and respects Bullet's locked axes.
Non-finite outputs are ignored for the affected bone.

An enabled external controller replaces native wind generation and force
application, including native gust modulation. A failed `evaluate` contributes
no wind for that update; readiness is checked again on the next frame.
Existing generic pre/post-step listeners continue running;
their unrelated forces, gravity, collisions, and constraints are unchanged.
