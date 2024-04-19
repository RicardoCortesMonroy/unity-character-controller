using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[RequireComponent(typeof(IKinematicMoverController))]
public class KinematicMover : MonoBehaviour
{
    public readonly static string Tag = "KinematicMover";

    [HideInInspector] public int ColliderInstanceID;

    public Vector3 TransientPosition { get { return _transientPosition; } }
    public Vector3 AngularVelocity { get { return _angularVelocity; } }

    private Vector3 _currentPosition;
    private Vector3 _transientPosition;
    private Quaternion _currentRotation;
    private Quaternion _transientRotation;

    private Vector3 _velocity;
    private Vector3 _angularVelocity; // rad/s

    private IKinematicMoverController _controller;

    private Transform _transform;

    public void OnValidate()
    {
        // Delay call until after inspector refreshes.
        // Unity has an annoying warning where "SendMessage()" can't be called in OnValidate.
        // SendMessage() seems to be called by all sorts of API calls, such as changing the gameobject's tag.

        _transform = transform;
        _controller = GetComponent<IKinematicMoverController>();
        gameObject.tag = Tag;

        _transientPosition = _transform.position;
        _transientRotation = _transform.rotation;

        ColliderInstanceID = GetComponent<Collider>().GetInstanceID();
    }

    private void OnEnable()
    {
        KinematicSystem.EnsureCreation();
        KinematicSystem.AddMover(this);
    }

    private void OnDisable()
    {
        KinematicSystem.RemoveMover(this);
    }

    public void CalculateVelocity()
    {
        _currentPosition = _transientPosition;
        _currentRotation = _transientRotation;

        _controller?.UpdateVelocity(ref _velocity, ref _angularVelocity);
    }

    public void Simulate()
    {
        _transientPosition += Time.fixedDeltaTime * _velocity;
        _transientRotation = Quaternion.Euler(Mathf.Rad2Deg * Time.fixedDeltaTime * _angularVelocity) * _transientRotation;
    }

    public Vector3 GetTangentialVelocity(Vector3 contactPoint)
    {
        Vector3 baseVelocity = _velocity;
        Vector3 angularVelocity = _angularVelocity;
        if (angularVelocity == Vector3.zero) return baseVelocity;

        Vector3 rotationalAxis = angularVelocity.normalized;
        // take the delta position and "flatten" it along the rotational axis
        Vector3 contactToCentre = (contactPoint - _transientPosition).With(rotationalAxis, 0f);
        Vector3 contactToCentreNextFrame = Quaternion.Euler(Time.fixedDeltaTime * Mathf.Rad2Deg * angularVelocity) * contactToCentre;
        Vector3 tangentialVelocity = (contactToCentreNextFrame - contactToCentre) / Time.fixedDeltaTime;

        //error += contactToCentreNextFrame.magnitude - contactToCentre.magnitude;
        //Debug.Log($"Error: {error}");

        return baseVelocity + tangentialVelocity;
    }

    public void ApplyTransientTransform()
    {
        _transform.position = _transientPosition;
        _transform.rotation = _transientRotation;
    }

    public void HandleInterpolation(float simulationStartTime, float deltaTime, bool interpolate)
    {
        if (deltaTime == 0f) return;
        if (interpolate)
        {
            float interpolationFactor = Mathf.Clamp01((Time.time - simulationStartTime) / deltaTime);
            _transform.position = Vector3.Lerp(_currentPosition, _transientPosition, interpolationFactor);
            _transform.rotation = Quaternion.Slerp(_currentRotation, _transientRotation, interpolationFactor);
        }
        else
        {
            ApplyTransientTransform();
        }
    }
}
