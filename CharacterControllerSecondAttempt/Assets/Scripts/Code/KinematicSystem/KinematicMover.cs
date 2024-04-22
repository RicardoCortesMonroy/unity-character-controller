using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.PackageManager;
using UnityEngine;

public struct BoxCastInfo
{
    public Vector3 Origin;
    public Vector3 Direction;
    public float Distance;
    public bool hit;
    public float hitDistance;
}

[RequireComponent(typeof(IKinematicMoverController))]
public class KinematicMover : MonoBehaviour
{

    public readonly static string Tag = "KinematicMover";

    [HideInInspector] public int ColliderInstanceID;

    public Vector3 TransientPosition { get { return _transientPosition; } }
    public Vector3 AngularVelocity { get { return _angularVelocity; } }


    [Tooltip("Continuous Collision Detection")]
    [SerializeField] private bool _useCCD = false;
    public bool UseCCD { get { return _useCCD; } }


    private Vector3 _currentPosition;
    private Vector3 _transientPosition;
    private Quaternion _currentRotation;
    private Quaternion _transientRotation;

    private Vector3 _linearVelocity;
    private Vector3 _angularVelocity; // rad/s

    private IKinematicMoverController _controller;

    private Transform _transform;
    private Collider _collider;

    private Vector3 _boxColliderSize;

    private BoxCastInfo _boxCastInfo;

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

        _collider = GetComponent<Collider>();
        ColliderInstanceID = _collider.GetInstanceID();

        if ( _useCCD )
        {
            var boxCollider = GetComponent<BoxCollider>();
            if ( boxCollider == null )
            {
                Debug.LogError($"CCD can only be used on objects with box colliders");
            }
            _boxColliderSize = Vector3.Scale(transform.lossyScale, boxCollider.size);
        }
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

        _controller?.UpdateVelocity(ref _linearVelocity, ref _angularVelocity);
    }

    // TODO: sweep mover across trajectory to detect any bodies in the way
    // Inform bodies if collided.
    // This sweep prevent fast movers from tunneling through bodies.
    public void SweepForBodies()
    {
        Vector3 sweepVector = Time.fixedDeltaTime * _linearVelocity;
        float sweepDistance = sweepVector.magnitude;
        Vector3 sweepDirection = sweepVector / sweepDistance;

        // Box cast across trajectory using KinematicBody layermask
        bool castHit = Physics.BoxCast(
            center: _currentPosition,
            halfExtents: 0.5f * _boxColliderSize,
            direction: sweepDirection,
            orientation: transform.rotation,
            maxDistance: sweepDistance,
            hitInfo: out RaycastHit hitInfo,
            layerMask: LayerMask.GetMask("KinematicBody")
        );

        Gizmos.color = Color.red;

        _boxCastInfo.Origin = _currentPosition;
        _boxCastInfo.Direction = sweepDirection;
        _boxCastInfo.Distance = sweepDistance; 
        _boxCastInfo.hit = castHit;

        // If hit, update body with hit information
        if (castHit)
        {
            float fractionOfFrameApplied = (sweepDistance - hitInfo.distance) / sweepDistance;
            KinematicBody body = hitInfo.collider.GetComponent<KinematicBody>();
            body?.RegisterMoverPush(_collider, -hitInfo.normal, _linearVelocity, fractionOfFrameApplied);

            _boxCastInfo.hitDistance = hitInfo.distance;
        }
    }

    public void Simulate()
    {
        _transientPosition += Time.fixedDeltaTime * _linearVelocity;
        _transientRotation = Quaternion.Euler(Mathf.Rad2Deg * Time.fixedDeltaTime * _angularVelocity) * _transientRotation;
    }

    public Vector3 GetTangentialVelocity(Vector3 contactPoint)
    {
        Vector3 baseVelocity = _linearVelocity;
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

    private float angleY;

    public void OnDrawGizmos()
    {
        // Draw sweep
        Gizmos.color = Color.white;
        Gizmos.DrawRay(_boxCastInfo.Origin, _boxCastInfo.Distance * _boxCastInfo.Direction);
        GizmoExtensions.DrawWireCube(_boxCastInfo.Origin, 0.5f * _boxColliderSize, transform.rotation);
        GizmoExtensions.DrawWireCube(_boxCastInfo.Origin + _boxCastInfo.Distance * _boxCastInfo.Direction, 0.5f * _boxColliderSize, transform.rotation);

        if (_boxCastInfo.hit)
        {
            // Draw hit
            Gizmos.color = Color.red;
            Gizmos.DrawRay(_boxCastInfo.Origin, _boxCastInfo.hitDistance * _boxCastInfo.Direction);
            GizmoExtensions.DrawWireCube(_boxCastInfo.Origin + _boxCastInfo.hitDistance * _boxCastInfo.Direction, 0.5f * _boxColliderSize, transform.rotation);
        }
    }
}
