using System.Collections;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(KinematicMover))]
public class MovingPlatform : MonoBehaviour, IKinematicMoverController
{
    [SerializeField] List<Vector3> _waypoints = new() { Vector3.zero };
    [SerializeField] float _speed = 2.5f;
    [SerializeField] Vector3 _rotationalAxis = Vector3.up;
    [SerializeField] bool _localAxes = true;
    [SerializeField] float _angularSpeed; //angular speed in deg/s
    [SerializeField] float _angularAcceleration; // deg/s^2


    private List<Vector3> _distinctWaypoints = new();
    private KinematicMover _mover;

    private Vector3 _startPos;
    private int _nextWaypointIndex = 1;

    Vector3 _rotationalAxisWorld;

    private void OnValidate()
    {

        _distinctWaypoints = _waypoints.Distinct().ToList();

        if (!Application.isPlaying && _distinctWaypoints.Count > 0) _startPos = transform.position + _distinctWaypoints[0];

        _rotationalAxis = _rotationalAxis.normalized;
        _rotationalAxisWorld = _localAxes ? transform.TransformDirection(_rotationalAxis) : _rotationalAxis;
    }

    private void Awake()
    {
        if (_distinctWaypoints.Count > 0) _startPos = transform.position + _distinctWaypoints[0];
    }

    public void UpdateVelocity(ref Vector3 velocity, ref Vector3 angularVelocity)
    {
        _angularSpeed += Time.fixedDeltaTime * _angularAcceleration;
        angularVelocity = Mathf.Deg2Rad * _angularSpeed * _rotationalAxisWorld.normalized;

        if (_distinctWaypoints.Count <= 1) return;

        Vector3 target = _startPos + _distinctWaypoints[_nextWaypointIndex];
        Vector3 vectorToTarget = target - transform.position;

        float distanceToTarget = vectorToTarget.magnitude;
        Vector3 direction = vectorToTarget.normalized;

        float effectiveSpeed = _speed;
        float distanceToMoveThisFrame = Time.fixedDeltaTime * _speed;

        if (distanceToTarget < distanceToMoveThisFrame)
        {
            effectiveSpeed *= distanceToTarget / distanceToMoveThisFrame;
            _nextWaypointIndex = (_nextWaypointIndex + 1) % _distinctWaypoints.Count;
        }

        velocity = effectiveSpeed * direction;
    }


    private void OnDrawGizmosSelected()
    {
        if (_distinctWaypoints.Count > 0)
        {
            Gizmos.color = Color.yellow;
            foreach (var pos in _distinctWaypoints)
            {
                Gizmos.DrawWireSphere(_startPos + pos, 0.2f);
            }
        }
    }

}
