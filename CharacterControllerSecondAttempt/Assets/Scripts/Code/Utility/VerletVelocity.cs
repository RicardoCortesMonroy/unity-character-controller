using UnityEngine;

public class VerletVelocity
{
    public Vector3 AppliedVelocity { get { return _appliedVelocity; } }
    public Vector3 Acceleration { get { return _acceleration; } set { _acceleration = value; } }

    private Vector3 _appliedVelocity;
    private Vector3 _currentVelocity;
    private Vector3 _previousVelocity;
    private Vector3 _acceleration;

    public VerletVelocity(Vector3? startingVelocity = null)
    {
        if (startingVelocity != null) SetVelocity((Vector3)startingVelocity);
    }

    public void ApplyAcceleration(bool useVerlet = true)
    {
        _previousVelocity = _currentVelocity;
        _currentVelocity += _acceleration * Time.fixedDeltaTime;

        _appliedVelocity = useVerlet ? 0.5f * (_previousVelocity + _currentVelocity) : _currentVelocity;
    }

    public void SetVelocity(Vector3 newVelocity)
    {
        _previousVelocity = newVelocity;
        _currentVelocity = newVelocity;
        _appliedVelocity = newVelocity;
    }

    public void AddImpulse(Vector3 impulse)
    {
        _previousVelocity += impulse;
        _currentVelocity += impulse;
        _appliedVelocity += impulse;
    }

    public void ConstrainVelocity(Vector3 component, float min = -Mathf.Infinity, float max = Mathf.Infinity)
    {
        Vector3 Constrain(Vector3 vector)
        {
            return vector.Constrain(component, min, max);
        }

        _previousVelocity = Constrain(_previousVelocity);
        _currentVelocity = Constrain(_currentVelocity);
        _appliedVelocity = Constrain(_appliedVelocity);
    }

    public void ConstrainSpeed(float maxSpeed)
    {
        _previousVelocity = _previousVelocity.CapMagnitude(maxSpeed);
        _currentVelocity = _currentVelocity.CapMagnitude(maxSpeed);
        _appliedVelocity = _appliedVelocity.CapMagnitude(maxSpeed);
    }
}
