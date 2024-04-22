using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public struct InputState
{
    public Vector3 MovementVelocity;
    public Vector3 ImpulseThisFrame;
    public Vector3 LookToVector;
    public Vector3 Gravity;
    public bool IsMoving;
    public bool MovementCanceledThisFrame;
}

[RequireComponent(typeof(CapsuleCollider))]
public class KinematicBody : MonoBehaviour
{
    public Vector3 LocalUpwards { get { return _localUpwards; } }

    private Transform _transform;
    private CapsuleCollider _collider;
    private ICharacterController _controller;

    // Mover variables
    private KinematicMover _currentMover;
    private Vector3 _moverDisplacement;

    [SerializeField] private TextMeshProUGUI _debug;
    private string _debugText;

    InputState _inputState;

    // Velocity components
    public VerletVelocity ForceVelocity = new();
    private Vector3 _movementVelocity;
    private Vector3 _groundVelocity;
    private Vector3 _residualGroundVelocity;

    // Current and transient (simulated) transform info
    private Vector3 _currentPosition;
    [SerializeField] private Vector3 _transientPosition;
    private Quaternion _currentRotation;
    private Quaternion _transientRotation;
    private Vector3 _lookToVector = Vector3.forward;
    private Vector3 _localUpwards = Vector3.up;

    [Range(0f, 1f)]
    [SerializeField] private float _moverVelocityDecayFactor = 0f; // factor that determines how residual ground velocity after leaving a mover will decay (0 means no decay, 1 is immediate)

    // Grounded-related variables
    public bool IsOnStableGround { get; private set; } // was the body on stable ground at the beginning of this frame?
    public Vector3 GroundNormal { get; private set; }
    public float StableGroundThreshold; // deg
    private bool _wasOnStableGroundInPreviousFrame;

    // Sweep variables
    private RaycastHit[] _sweepHits = new RaycastHit[8];
    private int _maxSweeps = 8;
    private float _colliderMargin = 0.01f; // distance repelled from a collision point within the next frame. Ensures there's always a small gap between colliders

    // Surface detection variables
    private Collider[] _collidersTouching = new Collider[8];
    private Vector3[] _collidersTouchingNormals = new Vector3[8];
    private int _nbCollidersTouching;
    private float _surfaceDetectionRange = 0.02f; // maximum distance away from collider in which the body is considered "touching" a surface
    readonly private int _maxPenetrationChecks = 10;

    // Collision interactions
    private float _ceilingCollisionDampening = 0.2f; //fraction of tangential velocity to ceiling applied on collision

    // Debug variables
    [Space(20)]
    [Header("Debug")]
    [SerializeField] private bool _showDebugText = true;
    [SerializeField] private bool _showSkinCollider;
    [SerializeField] private bool _showGroundNormal;
    [SerializeField] private bool _showVelocity;
    [SerializeField] private bool _showMovingPlatformContactPoints;
    [SerializeField] private bool _showSweep;
    [SerializeField] private bool _showDeltaMovement = true;
    [SerializeField] private bool _showContactSphere;

    // Velocity tracking
    private Vector3 _preSweepVelocity;
    private Vector3 _postSweepVelocity;
    private Vector3 _preSweepPosition;
    private Vector3 _postSweepPosition;


    private LayerMask _sceneLayer;


    private void OnValidate()
    {
        _transform = transform;
        _transform.position = _transientPosition;
        _transform.rotation = _transientRotation;
        _collider = GetComponent<CapsuleCollider>();
        _controller = GetComponent<ICharacterController>();
        if (_controller == null)
        {
            Debug.LogWarning($"Please assign an ICharacterController to {gameObject.name}");
        }

        ConfigureCapsule();
    }

    private void OnEnable()
    {
        KinematicSystem.EnsureCreation();
        KinematicSystem.AddBody(this);
    }

    private void OnDisable()
    {
        KinematicSystem.RemoveBody(this);
    }

    void Awake()
    {
        _currentPosition = _transform.position;
        _transientPosition = _transform.position;
        _currentRotation = _transform.rotation;
        _transientRotation = _transform.rotation;

        _sceneLayer = LayerMask.GetMask("Scene");

        GroundNormal = _localUpwards;
    }

    private void ConfigureCapsule()
    {
#if UNITY_EDITOR
        _collider.hideFlags = HideFlags.NotEditable;
        if (!Mathf.Approximately(transform.lossyScale.x, 1f) || !Mathf.Approximately(transform.lossyScale.y, 1f) || !Mathf.Approximately(transform.lossyScale.z, 1f))
        {
            Debug.LogError("Character's lossy scale is not (1,1,1). This is not allowed. Make sure the character's transform and all of its parents have a (1,1,1) scale.", this.gameObject);
        }
#endif
    }

    public void RegisterMoverDisplacement(Collider mover, Vector3 displacement)
    {
        if (mover.gameObject != _currentMover?.gameObject)
        {
            _moverDisplacement += displacement;
        }
    }

    public void UpdateCurrentPositionAndRotation()
    {
        _currentPosition = _transientPosition;
        _currentRotation = _transientRotation;
    }

    public void ApplyMoverDisplacement()
    {
        _transientPosition += _moverDisplacement;
        _moverDisplacement = Vector3.zero;
    }

    public void CalculateVelocity()
    {
        _controller?.UpdateInputState(ref _inputState);
        _movementVelocity = _inputState.MovementVelocity;
        ForceVelocity.Acceleration = _inputState.Gravity;
        
        if (_inputState.Gravity != Vector3.zero)
        {
            _localUpwards = -_inputState.Gravity.normalized;
        }
        


        // Impulse calculations
        bool IsImpulseThisFrame = _inputState.ImpulseThisFrame.sqrMagnitude > 0f;
        ForceVelocity.AddImpulse(_inputState.ImpulseThisFrame);
        _inputState.ImpulseThisFrame = Vector3.zero;
        //_gravity = _inputState.Gravity;


        #region Ground velocity calculations
        if (_currentMover != null)
        {
            // We choose the centre of the bottom sphere as the point on which to calculate tangential velocity
            // because its position is stationary relative to both the body and the platform
            // (unlike the base or the point of contact)
            Vector3 bottomSphereCentre = _transientPosition + _collider.radius * _localUpwards;
            _groundVelocity = _currentMover.GetTangentialVelocity(bottomSphereCentre);
            _residualGroundVelocity = _groundVelocity;
        }
        else
        {
            _groundVelocity = Vector3.zero;

            if (!IsOnStableGround)
            {
                _residualGroundVelocity *= Mathf.Pow(1 - _moverVelocityDecayFactor, Time.fixedDeltaTime);
            }
            else
            {
                _residualGroundVelocity = Vector3.zero;
            }
        }
        #endregion

        //Debug.DrawRay(_transientPosition, Time.fixedDeltaTime * _groundVelocity, Color.blue);

        if (!IsOnStableGround || IsImpulseThisFrame) ForceVelocity.ApplyAcceleration();

    }

    public void Simulate()
    {
        #region Handle rotation via the lookToVector

        if (_inputState.IsMoving)
        {
            _lookToVector = _inputState.LookToVector.With(_localUpwards, 0f).normalized;
        }
        else
        {
            if (_currentMover != null)
            {
                Vector3 appliedAngularVelocity = _currentMover.AngularVelocity.GetComponent(_localUpwards);
                _lookToVector = Quaternion.Euler(Mathf.Rad2Deg * Time.fixedDeltaTime * appliedAngularVelocity) * _lookToVector;
            }
            _lookToVector = Vector3.Cross(_localUpwards, Vector3.Cross(_lookToVector, _localUpwards));
        }
        _transientRotation = Quaternion.LookRotation(_lookToVector, _localUpwards);
        #endregion

        #region Handle position and perform sweeps
        Vector3 appliedVelocity = ForceVelocity.AppliedVelocity + _movementVelocity;

        _preSweepPosition = _transientPosition;
        _preSweepVelocity = appliedVelocity;
        // If on a mover, sweep across the ground velocity first while ignoring the mover itself
        // Prevent movers tunnelling through the body at fast speed or on thin geometry (e.g. the very edge of a corner)
        if (_currentMover != null)
        {
            SweepBodyForCollisions(_groundVelocity, ignoreCurrentMover: true);

        }
        // If there is no mover, we may still have residual velocity from the last mover
        else
        {
            appliedVelocity += _residualGroundVelocity;
            _preSweepVelocity = appliedVelocity;
        }

        SweepBodyForCollisions(appliedVelocity);

        #endregion
    }

    // Sweeps body in direction of velocity and detects & corrects collisions
    private void SweepBodyForCollisions(Vector3 appliedVelocity, bool ignoreCurrentMover = false)
    {
        if (appliedVelocity.sqrMagnitude == 0f) return;

        Vector3 capsuleTopHemi = (_collider.height - _collider.radius) * _localUpwards;
        Vector3 capsuleBottomHemi = (_collider.radius * _localUpwards);

        Vector3 sweepVectorRemaining = Time.fixedDeltaTime * appliedVelocity;
        bool terminateSweep = false;

        for (int i = 0; i < _maxSweeps; i++)
        {
            float sweepDistance = sweepVectorRemaining.magnitude;
            Vector3 sweepDirection = sweepVectorRemaining / sweepDistance;
            int nbHits = Physics.CapsuleCastNonAlloc(
                _transientPosition + capsuleTopHemi,
                _transientPosition + capsuleBottomHemi,
                _collider.radius,
                sweepDirection,
                _sweepHits,
                sweepDistance,
                _sceneLayer
                );
            int nbValidHits = nbHits;

            RaycastHit closestHit = new();
            float closestDistance = Mathf.Infinity;

            Vector3 previousTransientPosition = _transientPosition;

            // Check if hits are valid and cache the closest valid hit
            for (int h = 0; h < nbHits; h++)
            {
                if (_sweepHits[h].distance < closestDistance)
                {
                    // Ignore hit if it's the currentMover
                    if (ignoreCurrentMover &&
                        _currentMover != null &&
                        _sweepHits[h].colliderInstanceID == _currentMover.ColliderInstanceID)
                    {
                        nbValidHits -= 1;
                        continue;
                    }

                    // Ignore if hit distance is zero
                    if (Mathf.Approximately(_sweepHits[h].distance, 0f))
                    {
                        nbValidHits -= 1;
                        continue;
                    }

                    closestDistance = _sweepHits[h].distance;
                    closestHit = _sweepHits[h];
                }
            }


            // If hit was valid, execute logic to handle it
            if (nbValidHits > 0)
            {
                bool isHitStable = RegisterStability(closestHit);
                bool isCeiling = Vector3.Dot(closestHit.normal, _localUpwards) < -0.05f;

                Vector3 effectiveNormal = closestHit.normal;

                // If body is currently on stable ground and will collide with unstable ground, treat the unstable ground as a perpendicular obstacle
                if (!isHitStable && _wasOnStableGroundInPreviousFrame)
                {
                    effectiveNormal = Vector3.Cross(GroundNormal, Vector3.Cross(closestHit.normal, _localUpwards));

                    //Debug.DrawRay(_transientPosition, closestHit.normal, Color.red);
                    //Debug.DrawRay(_transientPosition, Vector3.Cross(closestHit.normal, _localUpwards), Color.cyan);
                    //Debug.DrawRay(_transientPosition, effectiveNormal, Color.magenta);
                    //Debug.DrawRay(_transientPosition, GroundNormal, Color.yellow);
                }

                // If body is going to hit stable ground, and was not on stable ground in the previous frame, terminate the sweep
                if (isHitStable && !_wasOnStableGroundInPreviousFrame)
                {
                    terminateSweep = true;
                }

                // If body is going to hit stable ground, reset the force velocity
                if (isHitStable || (!_wasOnStableGroundInPreviousFrame && IsOnStableGround))
                {
                    ForceVelocity.SetVelocity(Vector3.zero);
                }
                // else constrain it against the hit normal
                else
                {
                    ForceVelocity.ConstrainVelocity(effectiveNormal, min: 0f);
                }


                // If we've hit a ceiling and force velocity is moving up, then apply dampening
                if (isCeiling && Vector3.Dot(ForceVelocity.AppliedVelocity, _localUpwards) > 0f)
                {
                    ForceVelocity.SetVelocity(_ceilingCollisionDampening * ForceVelocity.AppliedVelocity);
                }


                Vector3 deltaPosition = (closestHit.distance - _colliderMargin) * sweepDirection;
                previousTransientPosition = _transientPosition;
                _transientPosition += deltaPosition;

                sweepVectorRemaining -= deltaPosition;
                sweepVectorRemaining = sweepVectorRemaining.Constrain(effectiveNormal, min: 0f);
            }
            else
            {
                _transientPosition += sweepVectorRemaining;
            }

            // Show path of sweep
            if (_showDeltaMovement)
            {
                Debug.DrawLine(previousTransientPosition, _transientPosition, Color.red);
            }

            // Break if no more collisions or if termination is called
            if (nbValidHits == 0 || terminateSweep) break;
        }
        _postSweepPosition = _transientPosition;
    }

    // Checks for all colliders within SurfaceDetectionRange and caches their normals
    // and updates collision check variables such as IsGrounded
    // All colliders within SurfaceDetectionRange are considered "touching" the character
    // Will also depenetrate from any colliders it's overlapping with up to maxPenetrationChecks times
    // Setting depentrateOnly to true will prevent resetting collision variables and
    // will not perform sweep collision tests with touching colliders
    public void CollisionCheck(bool depenetrateOnly = false)
    {
        if (!depenetrateOnly) ResetCollisionVariables();

        bool terminatePenetrationChecks = false;
        for (int i = 0; i < _maxPenetrationChecks; i++)
        {
            if (terminatePenetrationChecks) break;

            Vector3 capsuleTopHemi = (_collider.height - _collider.radius) * _localUpwards;
            Vector3 capsuleBottomHemi = _collider.radius * _localUpwards;
            //Debug.DrawLine(_transientPosition + capsuleTopHemi, _transientPosition + capsuleBottomHemi, Color.red);

            _nbCollidersTouching = Physics.OverlapCapsuleNonAlloc(
                    _transientPosition + capsuleTopHemi,
                    _transientPosition + capsuleBottomHemi,
                    _collider.radius + _surfaceDetectionRange,
                    _collidersTouching,
                    _sceneLayer
                    );

            // Assume no overlap at first, then set terminatePenetrationChecks to false if we detect an overlap
            terminatePenetrationChecks = true;

            // Compute penetration and register contact for each collider
            for (int j = 0; j < _nbCollidersTouching; j++)
            {
                Collider surfaceCollider = _collidersTouching[j];

                // If we're doing a collision check (!depenetrateOnly) and we're at the last penetration check
                // then reconcile the overlap by ignoring any collider that belongs to a mover
                // This will effectively give priority to non-movers if overlap is unsolvable
                if (!depenetrateOnly && i == _maxPenetrationChecks - 1 && surfaceCollider.CompareTag(KinematicMover.Tag))
                {
                    continue;
                }

                // If we're doing depenetrateOnly and we're overlapping with the current mover, ignore it.
                // The overlap will be fixed once we apply ground velocity, and we shouldn't depentrate because
                // the direction of depenetration may be differ from the ground velocity
                if (depenetrateOnly && _currentMover != null && _currentMover.ColliderInstanceID == surfaceCollider.GetInstanceID())
                {
                    continue;
                }

                // Inflate collider by _surfaceDetectionRange
                _collider.radius += _surfaceDetectionRange;
                _collider.height += 2 * _surfaceDetectionRange;
                Physics.ComputePenetration(
                    _collider,
                    _transientPosition,
                    _transientRotation,
                    surfaceCollider,
                    surfaceCollider.transform.position,
                    surfaceCollider.transform.rotation,
                    out Vector3 nearestDirection,
                    out float distance
                    );
                // Deflate back to original size
                _collider.radius -= _surfaceDetectionRange;
                _collider.height -= 2 * _surfaceDetectionRange;

                // If surface collider is overlapping with the collider (+margin), then perform depenetration
                // set terminatePenetrationChecks to false so that we can do another check to ensure the overlap is solved
                // We subtract a small epsilon to correct for floating point error
                if (distance > _surfaceDetectionRange)
                {
                    Vector3 overlapCorrection = (distance - _surfaceDetectionRange + _colliderMargin) * nearestDirection;

                    // Consider current stability if we're doing depenetration only, or consider the previous stability if we're doing the collision check (not depenetrateOnly)
                    bool onStableGroundAtBeginningOfFrame = (depenetrateOnly && IsOnStableGround) || (!depenetrateOnly && _wasOnStableGroundInPreviousFrame);
                    bool onMoverAndCollidingWithStatic = (_currentMover != null) && (!surfaceCollider.CompareTag(KinematicMover.Tag));

                    // If we were on stable ground at the beginning of the frame, constrain overlap correction to the plane of the ground
                    // EXCEPT in the case where we're on a mover and colliding with a static collider
                    // This is because if we're being squeezed between a mover and a static collider,
                    // we want to be squeezed through the mover, thereby prioritizing the collision correction from the static collider
                    if (onStableGroundAtBeginningOfFrame && !onMoverAndCollidingWithStatic)
                    {
                        overlapCorrection = overlapCorrection.Constrain(GroundNormal, min: 0f);
                    }

                    //Debug.Log($"Overlapping with {surfaceCollider.name}");

                    _transientPosition += overlapCorrection;
                    terminatePenetrationChecks = false;
                }

                // If only depentrating, we don't need to do the capsule cast
                if (depenetrateOnly) continue;

                int nbHits = Physics.CapsuleCastNonAlloc(
                    _transientPosition + capsuleTopHemi,
                    _transientPosition + capsuleBottomHemi,
                    _collider.radius,
                    -nearestDirection.normalized,
                    _sweepHits,
                    _surfaceDetectionRange,
                    _sceneLayer
                    );

                // Find and register the hit of the collider in question
                // Note: if we apply an overlap such that we're no longer contacting a collider, we will just continue with the function
                for (int h = 0; h < nbHits; h++)
                {
                    RaycastHit hit = _sweepHits[h];
                    if (hit.collider == surfaceCollider)
                    {
                        // Register contact
                        _collidersTouchingNormals[j] = hit.normal;
                        bool isHitStable = RegisterStability(hit);
                        if (isHitStable)
                        {
                            IsOnStableGround = true;
                            GroundNormal = hit.normal;

                            // Check if ground is a mover and register it if yes
                            int hitObjectId = hit.collider.gameObject.GetInstanceID();
                            _currentMover = KinematicSystem.CheckMover(hitObjectId);
                        }

                        break;
                    }
                }


            }
        }
    }

    // Resets variables involved in collision checks
    private void ResetCollisionVariables()
    {
        GroundNormal = _localUpwards;
        _wasOnStableGroundInPreviousFrame = IsOnStableGround;
        IsOnStableGround = false;
        _currentMover = null;
    }

    // Given a raycast hit, updates collision check variables according to the hit's normal
    private bool RegisterStability(RaycastHit hit)
    {
        float groundAngle = Vector3.Angle(hit.normal, _localUpwards);

        // If collided ground is stable
        return groundAngle < StableGroundThreshold;
    }

    public void ApplyTransientTransform()
    {
        _transform.position = _transientPosition;
        _transform.rotation = _transientRotation;
    }

    private void Update()
    {
        if (_showDebugText)
        {
            _debugText += $"\nOnStableGround: {IsOnStableGround}";
            _debugText += $"\nGround normal: {GroundNormal}";
            _debugText += $"\nForce velocity: {ForceVelocity.AppliedVelocity}\nMovement velocity: {_movementVelocity}\nGround velocity: {_groundVelocity}\nResidual ground velocity: {_residualGroundVelocity}\nPre-sweep velocity: {_preSweepVelocity}";
            _debugText += $"\nCurrent mover: {_currentMover?.name ?? "None"}";

            //_debugText += $"\nHitting normals:";
            //for (int i = 0; i < _nbCollidersTouching; i++)
            //{
            //    _debugText += $"\n  {_collidersTouchingNormals[i]}";
            //}

            _debug.text = _debugText;
            _debugText = "";
        }
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

    private void OnDrawGizmos()
    {
        if (_showSkinCollider)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(_postSweepPosition + _collider.radius * _localUpwards, _collider.radius + _colliderMargin);
            Gizmos.DrawWireSphere(_postSweepPosition + (_collider.height - _collider.radius) * _localUpwards, _collider.radius + _colliderMargin);
        }

        if (_showGroundNormal)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawRay(_transientPosition, GroundNormal);
        }

        if (_showVelocity)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawRay(_transientPosition, _preSweepVelocity);
            Gizmos.color = new Color(1f, 0.5f, 0f, 1f);
            Gizmos.DrawRay(_transientPosition, _postSweepVelocity);
        }

        if (_showSweep)
        {
            Gizmos.color = new Color(1f, 0.7f, 0f, 0.5f);
            Gizmos.DrawWireSphere(_preSweepPosition + _collider.radius * _localUpwards, _collider.radius);
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(_postSweepPosition + _collider.radius * _localUpwards, _collider.radius);

        }

        if (_showContactSphere)
        {
            Gizmos.color = new Color(1f, 0f, 0f, 0.5f);
            Gizmos.DrawWireSphere(_postSweepPosition + _collider.radius * _localUpwards, _collider.radius + _surfaceDetectionRange);
        }
    }

}
