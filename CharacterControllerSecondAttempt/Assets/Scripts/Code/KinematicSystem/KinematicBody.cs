using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using UnityEngine.InputSystem.HID;
using UnityEngine.UIElements;
using System.Linq;

public struct InputState
{
    public Vector3 MovementVelocity;
    public Vector3 ImpulseThisFrame;
    public Vector3 LookToVector;
    public Vector3 Gravity;
    public bool IsMoving;
    public bool MovementCanceledThisFrame;
}

public struct KinematicBodyDebugInfo
{
    public Vector3 StartPosition;
    public Vector3 PostMoverDisplacement;
    public Vector3 PostGroundSweepPosition;
    public Vector3 PostMovementSweepPosition;
    public Vector3 PostDepenetrationPosition;
    public Vector3 PostEdgeSnapPosition;

    public float MaxEdgeSnappingDistance;
}


[RequireComponent(typeof(CapsuleCollider))]
public class KinematicBody : MonoBehaviour
{
    public float Mass { get { return _mass; } }
    public Vector3 LocalUpwards { get { return _localUpwards; } }
    public Vector3 CentreOfMass { get { return transform.position + (_collider.radius + 0.5f * _collider.height) * transform.up; } }
    public bool IsOnStableGround { get; private set; } // was the body on stable ground at the beginning of this frame?
    public Vector3 GroundNormal { get; private set; }

    public VerletVelocity ForceVelocity = new();
    public float StableGroundThreshold { get; set; } // deg
    public float MaxEdgeSnappingAngle { get; set; }


    // PRIVATE MEMBERS:

    [SerializeField] private float _mass;

    private Transform _transform;
    private CapsuleCollider _collider;
    private ICharacterController _controller;

    // Mover variables
    private KinematicMover _currentMover;
    private Vector3 _moverDisplacement;

    [SerializeField] private TextMeshProUGUI _debug;
    private string _debugText;

    InputState _inputState;
    KinematicBodyDebugInfo _debugInfo;

    // Velocity components
    private Vector3 _movementVelocity;
    private Vector3 _groundVelocity;
    private Vector3 _residualGroundVelocity;
    private float _fractionOfFrameGroundVelocityApplied;
    private bool _isImpulseThisFrame;

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
    private Vector3 _groundNormalPrev;

    private bool _wasOnStableGroundInPreviousFrame;

    // Edge snapping

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
    [SerializeField] private bool _showGroundNormal;
    [SerializeField] private bool _showVelocity;
    [SerializeField] private bool _showMovingPlatformContactPoints;
    [SerializeField] private bool _showSweep;
    [SerializeField] private bool _showSweepSteps;

    [SerializeField] private bool _showStartPosition;
    [SerializeField] private bool _showPostMoverDisplacement;
    [SerializeField] private bool _showPostGroundSweepPosition;
    [SerializeField] private bool _showPostMovementSweepPosition;
    [SerializeField] private bool _showPostDepenetrationPosition;
    [SerializeField] private bool _showPostEdgeSnapPosition;

    [Space(10)]
    [SerializeField] private bool _showCollider;
    [SerializeField] private bool _showColliderMargin;
    [SerializeField] private bool _showSurfaceDetectionRange;
    [SerializeField] private bool _showTopSphere;

    // Velocity tracking
    private Vector3 _appliedSweepVelocity;
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

    public void RegisterMoverPush(Collider mover, Vector3 groundNormal, Vector3 moverVelocity, float fractionOfFrameApplied = 1.0f)
    {


        // If the body's velocity relative to the mover is pointing away
        // from the normal of the mover, then it will outpace the mover
        // within the same physics frame. Hence we shouldn't consider the collision.
        Vector3 relativeBodyVelocity = _groundVelocity + ForceVelocity.AppliedVelocity - moverVelocity;
        float bodyVelocityDotMoverNormal = Vector3.Dot(relativeBodyVelocity, groundNormal);
        if (bodyVelocityDotMoverNormal > 0) return;


        RegisterCollision(mover, groundNormal);

        if (mover.gameObject != _currentMover?.gameObject)
        {
            _moverDisplacement += fractionOfFrameApplied * Time.fixedDeltaTime * moverVelocity;
        }
        else
        {
            // If the body is only technically on the mover for 50% of the frame,
            // then we should only simulate 50% of the ground velocity
            _fractionOfFrameGroundVelocityApplied = fractionOfFrameApplied;
        }
    }

    // Given a raycast hit, updates collision check variables according to the hit's normal
    private void RegisterCollision(Collider collider, Vector3 normal)
    {
        if (IsStableGround(normal))
        {
            IsOnStableGround = true;
            GroundNormal = normal;
            ForceVelocity.SetVelocity(Vector3.zero);

            // Check if ground is a mover and register it if yes
            int hitObjectId = collider.gameObject.GetInstanceID();
            _currentMover = KinematicSystem.CheckMover(hitObjectId);
        }
    }

    public void UpdateCurrentPositionAndRotation()
    {
        _currentPosition = _transientPosition;
        _currentRotation = _transientRotation;

        _debugInfo.StartPosition = _currentPosition;
    }

    public void ApplyMoverDisplacement()
    {
        _transientPosition += _moverDisplacement;
        _moverDisplacement = Vector3.zero;

        _debugInfo.PostMoverDisplacement = _transientPosition;
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
        _isImpulseThisFrame = _inputState.ImpulseThisFrame.sqrMagnitude > 0f;
        ForceVelocity.AddImpulse(_inputState.ImpulseThisFrame);
        _inputState.ImpulseThisFrame = Vector3.zero;


        // Ground and residual velocity
        if (_currentMover != null)
        {
            // We choose the centre of the bottom sphere as the point on which to calculate tangential velocity
            // because its position is stationary relative to both the body and the platform
            // (unlike the base or the point of contact)
            Vector3 bottomSphereCentre = _transientPosition + _collider.radius * _localUpwards;
            _groundVelocity = _currentMover.GetTangentialVelocity(bottomSphereCentre);
            _residualGroundVelocity = _groundVelocity;
            _groundVelocity *= _fractionOfFrameGroundVelocityApplied;
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

        if (!IsOnStableGround || _isImpulseThisFrame) ForceVelocity.ApplyAcceleration();
    }

    public void Simulate()
    {
        // Rotation
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


        // Velocity
        _preSweepPosition = _transientPosition;
        _appliedSweepVelocity = ForceVelocity.AppliedVelocity + _movementVelocity;

        // If on a mover, sweep across the ground velocity first while ignoring the mover itself
        // Prevent movers tunnelling through the body at fast speed or on thin geometry (e.g. the very edge of a corner)
        if (_currentMover != null)
        {
            SweepBodyForCollisions(_groundVelocity, ignoreCurrentMover: true);

        }
        // If there is no mover, we may still have residual velocity from the last mover
        else
        {
            _appliedSweepVelocity += _residualGroundVelocity;
        }

        _debugInfo.PostGroundSweepPosition = _transientPosition;

        // Sweep across applied velocity
        SweepBodyForCollisions(_appliedSweepVelocity);

        _debugInfo.PostMovementSweepPosition = _transientPosition;


    }

    public void CheckForEdgeSnapping()
    {
        bool leftStableGroundThisFrame = _wasOnStableGroundInPreviousFrame && !IsOnStableGround;
        float velocitySweepDistance = _appliedSweepVelocity.magnitude;

        // Only check for edge snapping if the capsule left stable ground, if there's no impulse this frame, and if we're moving
        if (leftStableGroundThisFrame && !_isImpulseThisFrame && velocitySweepDistance > 0)
        {
            float groundAnglePrev = Mathf.Deg2Rad * Vector3.Angle(_groundNormalPrev, _localUpwards);

            // Prevent the maxEdgeSnappingAngle from going "steeper" than vertical
            float effectiveMaxEdgeSnappingAngle = Mathf.Min(MaxEdgeSnappingAngle, 89.0f + groundAnglePrev);


            // This is the maximum distance in which a valid edge snap could occur.
            // However, invalid edge snaps are not accounted for.
            // As long as the capsule speed is reasonably low, this shouldn't be noticable.
            float beta = 90 - effectiveMaxEdgeSnappingAngle + groundAnglePrev;
            float maxSnappingDistance = Mathf.Sin(Mathf.Deg2Rad * effectiveMaxEdgeSnappingAngle) * velocitySweepDistance / Mathf.Sin(Mathf.Deg2Rad * beta);

            _debugInfo.MaxEdgeSnappingDistance = maxSnappingDistance;

            Vector3 capsuleTopHemi = (_collider.height - _collider.radius) * _localUpwards;
            Vector3 capsuleBottomHemi = (_collider.radius * _localUpwards);

            int nbHits = Physics.CapsuleCastNonAlloc(
                point1: _transientPosition + capsuleTopHemi,
                point2: _transientPosition + capsuleBottomHemi,
                radius: _collider.radius + _colliderMargin,
                direction: -_localUpwards,
                maxDistance: maxSnappingDistance,
                results: _sweepHits,
                layerMask: _sceneLayer
            );

            RaycastHit closestHit = _sweepHits[0];


            for (int i = 1; i < nbHits; i++)
            {
                if (_sweepHits[i].distance < closestHit.distance)
                {
                    closestHit = _sweepHits[i];
                }
            }

            // Collider must hit AND the change in angle must be below the maximum edge snapping angle
            float angleChange = Vector3.Angle(_groundNormalPrev, closestHit.normal);
            if (nbHits > 0 && closestHit.distance > 0f && angleChange <= MaxEdgeSnappingAngle)
            {
                _transientPosition += closestHit.distance * -_localUpwards;
                RegisterCollision(closestHit.collider, closestHit.normal);

                // "Depenetrate" using the just-registered ground normal
                //_transientPosition += (_surfaceDetectionRange - _colliderMargin) * GroundNormal;
            }
        }

        _debugInfo.PostEdgeSnapPosition = _transientPosition;
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
                bool isHitStable = IsStableGround(closestHit.normal);
                bool isCeiling = Vector3.Dot(closestHit.normal, _localUpwards) < -0.05f;

                Vector3 effectiveNormal = closestHit.normal;

                // Check if the hit is a rigidbody, and apply force as necessary
                Rigidbody rb = closestHit.collider.GetComponent<Rigidbody>();
                if (rb != null)
                {
                    Vector3 v_ai = appliedVelocity;
                    Vector3 v_bi = rb.velocity;
                    float m_a = _mass;
                    float m_b = rb.mass;

                    Vector3 v_b = (2 * m_a / (m_a + m_b)) * v_ai + ((m_b - m_a) / (m_a + m_b)) * v_bi;

                    Vector3 delta_p_b = m_b * (v_b - v_bi);
                    Vector3 F_b = delta_p_b / 0.1f;

                    Debug.Log($"Colliding with RB, applying force {F_b} at {closestHit.point}");
                    rb.AddForceAtPosition(F_b, closestHit.point, ForceMode.Force);
                }

                // If body is currently on stable ground and will collide with unstable ground, treat the unstable ground as a perpendicular obstacle
                if (!isHitStable && IsOnStableGround)
                {
                    effectiveNormal = Vector3.Cross(GroundNormal, Vector3.Cross(closestHit.normal, _localUpwards));

                    //Debug.DrawRay(_transientPosition, closestHit.normal, Color.red);
                    //Debug.DrawRay(_transientPosition, Vector3.Cross(closestHit.normal, _localUpwards), Color.cyan);
                    //Debug.DrawRay(_transientPosition, effectiveNormal, Color.magenta);
                    //Debug.DrawRay(_transientPosition, GroundNormal, Color.yellow);
                }

                // If body is going to hit stable ground, and was not on stable ground in the previous frame, terminate the sweep
                if (isHitStable && !IsOnStableGround)
                {
                    terminateSweep = true;
                }

                // If body is going to hit stable ground, reset the force velocity
                if (isHitStable)
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
            if (_showSweepSteps)
            {
                Debug.DrawLine(previousTransientPosition, _transientPosition, Color.blue);
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
        if (!depenetrateOnly) ResetSimulationVariables();

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
                if (!depenetrateOnly && i == _maxPenetrationChecks - 1 && surfaceCollider.IsKinematicMover())
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

                nearestDirection = nearestDirection.normalized;

                //Debug.DrawRay(_transientPosition + 0.01f * Vector3.down, distance * nearestDirection, Color.blue);

                // If surface collider is overlapping with the collider (+margin), then perform depenetration
                // set terminatePenetrationChecks to false so that we can do another check to ensure the overlap is solved
                float overlapDistance = Mathf.Max(distance - _surfaceDetectionRange + _colliderMargin, 0f);
                Vector3 overlapCorrection = overlapDistance * nearestDirection;

                if (overlapDistance > 0.0001f)
                {
                    // Consider current stability if we're doing depenetration only, or consider the previous stability if we're doing the collision check (not depenetrateOnly)
                    bool onStableGroundAtBeginningOfFrame = (depenetrateOnly && IsOnStableGround) || (!depenetrateOnly && _wasOnStableGroundInPreviousFrame);
                    bool onMoverAndCollidingWithStatic = (_currentMover != null) && (!surfaceCollider.CompareTag(KinematicMover.Tag));

                    // Only constrain overlap to ground normal if it's from a steep wall (not a ceiling, or stable ground)
                    float surfaceAngle = Vector3.Angle(_localUpwards, nearestDirection);
                    bool surfaceIsSteep = surfaceAngle > StableGroundThreshold && surfaceAngle < 90f;


                    // If we were on stable ground at the beginning of the frame, constrain overlap correction to the plane of the ground
                    // EXCEPT in the case where we're on a mover and colliding with a static collider
                    // This is because if we're being squeezed between a mover and a static collider,
                    // we want to be squeezed through the mover, thereby prioritizing the collision correction from the static collider
                    if (onStableGroundAtBeginningOfFrame && !onMoverAndCollidingWithStatic)
                    {
                        // Steep inclines
                        if (surfaceIsSteep)
                        {
                            // o = |o_0| (n1 x n2) x n1 / sin(theta)^2
                            // o: new overlapCorrection
                            // o_0: original overlapCorrection
                            // n1: stable ground normal (GroundNormal)
                            // n2: current depenetration normal (nearestDirection)
                            // theta: angle between normals
                            // Reminder that |(n1 x n2) x n1| = sin(theta)

                            // flattenedOverlapVector is the direction of overlap correction constrained to the current ground normal
                            Vector3 flattenedOverlapVector = Vector3.Cross(Vector3.Cross(_groundNormalPrev, nearestDirection), _groundNormalPrev);
                            float correctionFactor = overlapDistance / flattenedOverlapVector.sqrMagnitude;

                            overlapCorrection = correctionFactor * flattenedOverlapVector;

                        }
                        // Stable ground or ceilings
                        else
                        {
                            overlapCorrection = overlapCorrection.Constrain(_groundNormalPrev, 0f);
                        }
                    }

                    _transientPosition += overlapCorrection;
                    terminatePenetrationChecks = false;
                }

                // If only depentrating, we don't need to do the capsule cast
                if (depenetrateOnly) continue;

                // Sweep the capsule over the surfaceDetectionRange.
                int nbHits = Physics.CapsuleCastNonAlloc(
                    _transientPosition + capsuleTopHemi,
                    _transientPosition + capsuleBottomHemi,
                    _collider.radius,
                    -nearestDirection,
                    _sweepHits,
                    _surfaceDetectionRange,
                    _sceneLayer
                    );

                // Find and register the hit of the collider in question
                for (int h = 0; h < nbHits; h++)
                {
                    RaycastHit hit = _sweepHits[h];
                    if (hit.collider == surfaceCollider)
                    {

                        // If the collider is a rigidbody, then stop it from pushing body
                        Rigidbody rb = hit.collider.GetComponent<Rigidbody>();
                        if (rb != null)
                        {
                            Vector3 relativeVelocity = rb.velocity - _appliedSweepVelocity;
                            rb.velocity -= relativeVelocity.GetComponent(hit.normal);
                        }

                        // Register contact
                        _collidersTouchingNormals[j] = hit.normal;
                        RegisterCollision(hit.collider, hit.normal);

                        //Debug.Log($"Registering collision with {hit.collider.name}, penetration check {i}, collider {j}, overlapDistance {overlapDistance}, overlapCorrection {overlapCorrection}");

                        break;
                    }
                }


            }
        }

        _debugInfo.PostDepenetrationPosition = _transientPosition;
    }

    // Resets variables involved in collision checks
    private void ResetSimulationVariables()
    {
        _groundNormalPrev = GroundNormal;
        GroundNormal = _localUpwards;
        _wasOnStableGroundInPreviousFrame = IsOnStableGround;
        IsOnStableGround = false;
        _currentMover = null;
        _fractionOfFrameGroundVelocityApplied = 1.0f;
    }

    private bool IsStableGround(Vector3 groundNormal)
    {
        float groundAngle = Vector3.Angle(groundNormal, _localUpwards);
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
            _debugText += $"\nWasOnStableGround: {_wasOnStableGroundInPreviousFrame}";
            _debugText += $"\nGround normal: {GroundNormal}";
            _debugText += $"\nForce velocity: {ForceVelocity.AppliedVelocity}\nMovement velocity: {_movementVelocity}\nGround velocity: {_groundVelocity}\nResidual ground velocity: {_residualGroundVelocity}\nPre-sweep velocity: {_appliedSweepVelocity}";
            _debugText += $"\nCurrent mover: {_currentMover?.name ?? "None"}";
            _debugText += $"\nLocalUpwards: {_localUpwards}";

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
        if (!Application.isPlaying) return;

        if (_showGroundNormal)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawRay(_transientPosition, GroundNormal);
        }

        if (_showVelocity)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawRay(_transientPosition, _appliedSweepVelocity);
            Gizmos.color = new Color(1f, 0.5f, 0f, 1f);
            Gizmos.DrawRay(_transientPosition, _postSweepVelocity);
        }

        if (_showSweep)
        {
            Gizmos.color = new Color(1f, 0.7f, 0f, 1f);
            GizmoExtensions.DrawSphere(_currentPosition + _collider.radius * _localUpwards, _collider.radius);
            Gizmos.color = Color.red;
            GizmoExtensions.DrawSphere(_transientPosition + _collider.radius * _localUpwards, _collider.radius);
        }


        List<bool> showPosition = new List<bool>
        {
            _showStartPosition,
            _showPostMoverDisplacement,
            _showPostGroundSweepPosition,
            _showPostMovementSweepPosition,
            _showPostDepenetrationPosition,
            _showPostEdgeSnapPosition
        };

        Vector3 topOffset = (_collider.height - 2 * _collider.radius) * _localUpwards;
        List<Vector3> debugPositions = new List<Vector3>
        {
            _debugInfo.StartPosition,
            _debugInfo.PostMoverDisplacement,
            _debugInfo.PostGroundSweepPosition,
            _debugInfo.PostMovementSweepPosition,
            _debugInfo.PostDepenetrationPosition,
            _debugInfo.PostEdgeSnapPosition
        };

        for (int i = 0; i < debugPositions.Count; i++)
        {
            if (!showPosition[i]) continue;

            Vector3 position = debugPositions[i];
            Gizmos.color = Color.red;
            if (i > 0) Gizmos.DrawRay(debugPositions[i - 1], position - debugPositions[i - 1]);

            if (_showCollider)
            {
                GizmoExtensions.DrawSphere(position + _collider.radius * _localUpwards, _collider.radius);
                if (_showTopSphere) GizmoExtensions.DrawSphere(position + topOffset + _collider.radius * _localUpwards, _collider.radius);
            }

            if (_showColliderMargin)
            {
                Gizmos.color = Color.yellow;
                GizmoExtensions.DrawSphere(position + _collider.radius * _localUpwards, _collider.radius + _colliderMargin);
                if (_showTopSphere) GizmoExtensions.DrawSphere(position + topOffset + _collider.radius * _localUpwards, _collider.radius + _colliderMargin);
            }

            if (_showSurfaceDetectionRange)
            {
                Gizmos.color = Color.green;
                GizmoExtensions.DrawSphere(position + _collider.radius * _localUpwards, _collider.radius + _surfaceDetectionRange);
                if (_showTopSphere) GizmoExtensions.DrawSphere(position + topOffset + _collider.radius * _localUpwards, _collider.radius + _surfaceDetectionRange);
            }
        }

        if (_showPostEdgeSnapPosition)
        {
            Gizmos.color = Color.blue;
            GizmoExtensions.DrawSphere(_debugInfo.PostDepenetrationPosition + _debugInfo.MaxEdgeSnappingDistance * -_localUpwards + _collider.radius * _localUpwards, _collider.radius + _surfaceDetectionRange);
        }

    }
}
