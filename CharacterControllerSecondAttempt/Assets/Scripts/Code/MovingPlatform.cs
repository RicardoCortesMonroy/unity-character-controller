using System.Collections;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(KinematicMover))]
public class MovingPlatform : MonoBehaviour, IKinematicMoverController
{
    [SerializeField] List<Vector3> _waypoints = new() { Vector3.zero };
    [SerializeField] float _maxSpeed = 2.5f;
    [SerializeField] float _timeToAccelerate = 1f;
    [SerializeField] Vector3 _rotationalAxis = Vector3.up;
    [SerializeField] bool _localAxes = true;
    [SerializeField] float _angularSpeed; //angular speed in deg/s
    [SerializeField] float _angularAcceleration; // deg/s^2


    private List<Vector3> _distinctWaypoints = new();
    private KinematicMover _mover;

    private Vector3 _startPos;
    private Vector3 _direction;
    private Vector3 _directionPrev;

    private float _minimumDistanceToPoint = 0.05f;
    private float _speed;
    private float _acceleration;
    private float _distanceToAccelerateRaw;
    private int _originWaypointIndex = 0;
    private int _targetWaypointIndex = 1;
    private int _targetWaypointIndexPrev;

    Vector3 _rotationalAxisWorld;

    private void OnValidate()
    {

        _distinctWaypoints = _waypoints.Distinct().ToList();

        if (!Application.isPlaying && _distinctWaypoints.Count > 0) _startPos = transform.position + _distinctWaypoints[0];

        if (_timeToAccelerate == 0f)
        {
            _acceleration = Mathf.Infinity;
            _distanceToAccelerateRaw = 0f;
        }
        else
        {
            _acceleration = _maxSpeed / _timeToAccelerate;
            _distanceToAccelerateRaw = 0.5f * _acceleration * _timeToAccelerate * _timeToAccelerate;
        }

        _mover = GetComponent<KinematicMover>();

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

        if (_distinctWaypoints.Count < 2) return;

        if (_maxSpeed > 0f)
        {
            Vector3 _originWayPoint = _distinctWaypoints[_originWaypointIndex];
            Vector3 _targetWayPoint = _distinctWaypoints[_targetWaypointIndex];

            Vector3 deltaToTarget = _startPos + _targetWayPoint - _mover.TransientPosition;
            float distanceToTarget = deltaToTarget.magnitude;
            if (distanceToTarget > 0f)
            {
                _direction = deltaToTarget / distanceToTarget;
            }

            float distanceBetweenAdjacentPoints = Vector3.Distance(_originWayPoint, _targetWayPoint);
            float distanceToAccelerate = _distanceToAccelerateRaw;

            if (_distanceToAccelerateRaw > distanceBetweenAdjacentPoints / 2)
            {
                Debug.LogWarning($"Distance to accelerate ({_distanceToAccelerateRaw}) is greater than half the available distance ({distanceBetweenAdjacentPoints / 2}). Cannot reach max speed. Please distance the points farther or reduce the max speed.");
                distanceToAccelerate = distanceBetweenAdjacentPoints / 2;
            }

            if (distanceToTarget < distanceToAccelerate)
            {
                _speed -= _acceleration * Time.fixedDeltaTime;
            }
            else
            {
                _speed += _acceleration * Time.fixedDeltaTime;
            }

            //Debug.Log($"Index: {_targetWaypointIndex}, Index prev: {_targetWaypointIndexPrev}, Direction: {_direction}, Distance to target: {distanceToTarget}, Velocity: {velocity}");

            // If platform is within tolerance of target OR velocity is near zero OR its target direction has flipped (it's passed the target), then increment the WP index
            if (distanceToTarget < _minimumDistanceToPoint ||
                (_targetWaypointIndexPrev == _targetWaypointIndex && Vector3.Dot(_direction, _directionPrev) < 0f))
            {
                //Debug.Log($"Incrementing target index. Distance check: {distanceToTarget < _minimumDistanceToPoint}, Direction check: {_targetWaypointIndexPrev == _targetWaypointIndex && Vector3.Dot(_direction, _directionPrev) < 0f} ");
                _originWaypointIndex = _targetWaypointIndex;
                _targetWaypointIndexPrev = _targetWaypointIndex;
                _targetWaypointIndex = (_targetWaypointIndex + 1) % _distinctWaypoints.Count;
            }
            else
            {
                _targetWaypointIndexPrev = _targetWaypointIndex;
            }

            _directionPrev = _direction;
        }

        _speed = Mathf.Clamp(_speed, 0, _maxSpeed);
        velocity = _speed * _direction;
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
