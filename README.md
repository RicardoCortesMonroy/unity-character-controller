The order of operations in the kinetic body system is governed by KinematicSystem.cs

This is the FixedUpdate code:

```C#
private void FixedUpdate()
{
    _simulationStartTime = Time.time;

    foreach (var mover in _movers.Values)
    {
        mover.CalculateVelocity();
        mover.Simulate();
        mover.ApplyTransientTransform();
    }
    foreach (var body in _bodies.Values)
    {
        body.UpdateCurrentPositionAndRotation();
        body.CalculateVelocity();
        body.Simulate();
        body.CollisionCheck();

        body.ApplyTransientTransform();
    }
    _fixedFrame++;
}
```

A _body_ is an object in the system whose movement can be controlled externally (e.g. by player input or by AI), but is also subject to forces and collisions with the environment.

A _mover_ is an object in the system that is able to move a body with no resistance. A body may stand on a mover, provided the surface angle is shallow enough, and be "moved" by the mover, maintaining a static position and orientation relative to its point of contact with the mover.

The movers are all simulated before the bodies so that the bodies can perform a valid collision check at the end of the frame.
For example, imagine a body that's standing on a ascending platform. If the body were simulated first, it would end up above the platform, ungrounded, at the end of the simulation. By the time the collision check happens, the platform is still too far underneath the player, and so the player isn't considered grounded. But after, when the mover is simulated, it reaches the bottom of the player. So the player appears grounded, but the system doesn't know.

The general structure of an object's simulation is to first calculate its linear and angular velocity in the next frame, then apply those velocities to their "transient" position and rotation (that is, their position and rotation at the end of the frame), then to apply the transient position and rotation to the object's collider.

Mover simulation:
1. CalculateVelocity:
    * Requests its controller to update its linear and angular velocity. The mechanics of the controller are up to the user to implement.
2. Simulate:
    * Apply linear and angular velocity to transient position and rotation.
3. ApplyTransientTransform:
    * Apply transient position and rotation to the object's collider.

Body simulation:
1. UpdateCurrentPositionAndRotation:
    * Stores the current position and rotation for later use.
2. CalculateVelocity:
    * Gets input from controller and calculates nominal velocity (that is before any obstacles). This will depend: 
      * User input movement
      * Force calculations (e.g. gravity)
      * Ground movement (if the body is standing on a mover). This accounts for both the linear velocity of the mover and the tangential velocity (if it's rotating).
      * Residual movement from a past mover
3. Simulate:
    * Calculates correct orientation from user input.
    * If on a mover, sweeps the capsule along the same delta vector as the mover. The final position is as if the body hadn't moved at all relative to the mover.
    * Sweeps again across the applied velocity calculated in the previous step.
4. CollisionCheck:
    * Checks if collider is overlapping with any object in the environment, and depenetrates the capsule if so.
    * Based on colliders that the body is "touching" (i.e. overlapping within a defined margin), the body will keep track of any colliders it's touching and their normals at the point of contact.
    * Based on the normals of the touching colliders, the body will also determine whether it's on stable ground (which is useful information for the controller).
5. ApplyTransientTransform:
    * Apply transient position and rotation to the object's collider.

"Sweeps" are when we simulate a collider "sweeping" across a given movement vector. If any other colliders are in its path, then the sweep will correct the movement vector in a way that prevents overlap.
