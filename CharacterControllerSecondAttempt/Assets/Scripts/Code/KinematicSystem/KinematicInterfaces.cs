using UnityEngine;

public interface ICharacterController
{
    public void UpdateInputState(ref InputState inputState);
}

public interface IKinematicMoverController
{
    public void UpdateVelocity(ref Vector3 velocity, ref Vector3 angularVelocity);
}
