using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.LowLevel;


public class PlayerInputHandler : MonoBehaviour, ICharacterController
{
    enum MovementMode
    {
        GROUNDED,
        HANGING,
        AIRBORNE
    }


    public KinematicBody KinematicBody { get { return _body; } }
    public Vector3 WorldInput { get { return _worldInputVector; } } // input vector in world coordinates
    public float WalkingSpeed { get { return _walkingSpeed; } }
    public float RunningSpeed { get { return _runningSpeed; } }

    private Camera _camera;
    private KinematicBody _body;
    private PlayerInput _playerInput;

    [SerializeField] private Transform _gravityFocus;

    [SerializeField] private bool _forceInput;
    [SerializeField] private Vector2 _forcedMovementInput;

    [Space(10)]
    [Header("Movement variables")]
    [SerializeField] private float _walkingSpeed = 4;
    [SerializeField] private float _runningSpeed = 8;
    [SerializeField] private float _shimmySpeed = 1f;
    [Tooltip("Parameter used in lerping between different speeds. 1 is instant, 0 is infinite")]
    [Range(0.01f, 1f)]
    [SerializeField] private float _speedSmoothing = 0.1f;

    [Space(10)]
    [Header("Jump variables")]
    [SerializeField] private float _timeToApex;
    [SerializeField] private float _maxJumpHeight;
    [SerializeField] private float _maxAerialMovementSpeed;
    [SerializeField] private float _aerialAcceleration;
    [SerializeField] Vector3 _gravityDirection;
    [SerializeField] private float _gravityReleaseMultiplier; // multiplier of gravity when jump is released

    [Space(10)]
    [Header("Miscellaneous")]
    [SerializeField] private float _stableGroundThreshold;
    [SerializeField] private float _maxEdgeSnappingAngle;
    [SerializeField] private float _shimmyAngleTolerance;
    [SerializeField] private float _ledgeHangLevel;


    private float _gravityMagnitude;
    private float _initialJumpVelocity;

    // Movement mode
    MovementMode _movementMode;

    // Movement bases
    private Vector3 _forwardBasis;
    private Vector3 _rightBasis;

    // Rotation variables
    private Vector3 _targetLookToVector;
    private Vector3 _appliedLookToVector;
    private float _slerpFactor = 0.999f; // Factor by which the the rotation will slerp to the target rotation each second

    // Vector input
    private Vector2 _movementInput;

    // Movement tracking
    private Vector3 _worldInputVector = new();
    private Vector3 _worldInputVectorPersist = new();
    private float _appliedSpeed = 0f;

    // Boolean flags
    private bool _releaseFromLedge = false;

    // Boolean input
    private bool _isJumpPressed;
    private bool _isJumpQueued;
    private bool _isRunningPressed;
    private bool _isAccelerating;
    private bool _isMovementPressed;
    private bool _isMovementCanceled;

    private void OnValidate()
    {
        _camera = Camera.main;
        _body = GetComponent<KinematicBody>();

        if (Application.isPlaying)
        {
            _forcedMovementInput = _forcedMovementInput.CapMagnitude(1f);
            _gravityDirection = _gravityDirection.normalized;
        }

        _initialJumpVelocity = (2 * _maxJumpHeight) / _timeToApex;
        _gravityMagnitude = (2 * _maxJumpHeight) / Mathf.Pow(_timeToApex, 2);

        _body.StableGroundThreshold = _stableGroundThreshold;
        _body.MaxEdgeSnappingAngle = _maxEdgeSnappingAngle;
    }

    private void Awake()
    {
        _playerInput = new PlayerInput();

        _playerInput.CharacterControls.MoveController.started += OnMovementInputController;
        _playerInput.CharacterControls.MoveController.performed += OnMovementInputController;
        _playerInput.CharacterControls.MoveController.canceled += OnMovementInputController;

        _playerInput.CharacterControls.MoveKeyboard.started += OnMovementInputKeyboard;
        _playerInput.CharacterControls.MoveKeyboard.performed += OnMovementInputKeyboard;
        _playerInput.CharacterControls.MoveKeyboard.canceled += OnMovementInputKeyboard;

        _playerInput.CharacterControls.Run.started += OnRun;
        _playerInput.CharacterControls.Run.canceled += OnRun;

        _playerInput.CharacterControls.Jump.started += OnJump;
        _playerInput.CharacterControls.Jump.canceled += OnJump;

        _playerInput.CharacterControls.Pause.started += OnPause;
        _playerInput.CharacterControls.Pause.canceled += OnPause;
    }

    public void UpdateInputState(ref InputState inputState)
    {
        // Determine the applied input base
        Vector3 appliedInput = _forceInput ? _forcedMovementInput : _movementInput;
        inputState.IsMoving = appliedInput.sqrMagnitude > Mathf.Epsilon;


        // Determine input vector in world coordinates based on camera orientation
        _worldInputVector = GetWorldFromInput(appliedInput, _camera);

        // Keeps track of the previous input vector if player is stationary
        if (inputState.IsMoving) _worldInputVectorPersist = _worldInputVector;


        // Determine movement mode
        if (_body.IsOnStableGround) _movementMode = MovementMode.GROUNDED;
        else if (_body.IsHangingOnLedge) _movementMode = MovementMode.HANGING;
        else _movementMode = MovementMode.AIRBORNE;

        // Handle Movement velocity
        inputState.MovementVelocity = GetMovementFromMode(_movementMode, _worldInputVector, inputState);
        inputState.ReleaseFromLedge = _releaseFromLedge;

        // Handle Rotation
        inputState.LookToVector = GetLookToVector(inputState);


        // Handle Gravity
        inputState.Gravity = GetGravityVector();


        // Handle Jump
        inputState.ImpulseVelocity = GetImpulseVector();
    }


    private Vector3 GetWorldFromInput(Vector3 input, Camera camera)
    {
        Vector3 SetBasisFromCamera(Vector3 cameraBasis)
        {
            // 1. Flatten to XZ plane
            Vector3 basis = cameraBasis.With(_body.LocalUpwards, 0f);
            // 2. Project to the ground plane and normalize
            basis = basis.With(_body.GroundNormal, 0f).normalized;
            return basis;
        }

        _forwardBasis = SetBasisFromCamera(camera.transform.forward);
        _rightBasis = SetBasisFromCamera(camera.transform.right);

        Vector3 worldInputVector = new()
        {
            x = input.x * _rightBasis.x + input.y * _forwardBasis.x,
            y = input.x * _rightBasis.y + input.y * _forwardBasis.y,
            z = input.x * _rightBasis.z + input.y * _forwardBasis.z
        };

        return worldInputVector;
    }

    private Vector3 GetMovementFromMode(MovementMode mode, Vector3 inputVector, InputState inputState)
    {
        Vector3 movementVelocity;

        switch (mode)
        {
            case MovementMode.GROUNDED:
            {
                float targetSpeed = _isMovementPressed ? _isRunningPressed ? _runningSpeed : _walkingSpeed : 0f;
                if (_forceInput) targetSpeed = _walkingSpeed;
                _appliedSpeed = Mathf.Lerp(_appliedSpeed, targetSpeed, 1 - Mathf.Pow(1 - _speedSmoothing, Time.deltaTime));
                movementVelocity = _worldInputVectorPersist * _appliedSpeed;
                break;
            }

            case MovementMode.HANGING:
            {
                // Keep capsule against ledge if moving towards it
                // Otherwise release it from the ledge
                float shimmyAngle = Vector3.Angle(inputVector, _body.LedgeNormal);
                if (inputState.IsMoving && shimmyAngle < 90 - _shimmyAngleTolerance)
                {
                    _releaseFromLedge = true;
                }
                else
                {
                    _releaseFromLedge = false;
                    inputVector = inputVector.With(_body.LedgeNormal, 0f);
                }

                movementVelocity = inputVector * _shimmySpeed;
                break;
            }

            case MovementMode.AIRBORNE:
            {
                // neutralVelocity is the movement velocity required to align the total applied velocity (including force) with the local upwards vector
                // it acts as an 'origin' point for the movement velocity
                Vector3 neutralVelocity = -_body.ForceVelocity.AppliedVelocity.With(_body.LocalUpwards, 0f);
                Vector3 targetAerialVelocity = (neutralVelocity + _maxAerialMovementSpeed * inputVector).CapMagnitude(_maxAerialMovementSpeed);
                Vector3 targetDelta = targetAerialVelocity - inputState.MovementVelocity;
                float t = Mathf.Clamp01(Time.fixedDeltaTime * _aerialAcceleration / targetDelta.magnitude);

                movementVelocity = Vector3.Lerp(inputState.MovementVelocity, targetAerialVelocity, t);
                break;
            }

            default:
                movementVelocity = Vector3.zero;
                break;
        }

        return movementVelocity;
    }

    private Vector3 GetLookToVector(InputState inputState)
    {
        // Player should be facing the wall when hanging
        if (_body.IsHangingOnLedge && !inputState.ReleaseFromLedge)
        {
            _targetLookToVector = -_body.LedgeNormal;
            _appliedLookToVector = _targetLookToVector;

            Debug.DrawRay(transform.position, 2f * _body.LedgeNormal, Color.magenta);
        }
        // Only update active movement direction when movement is pressed.
        // Keeps track of how the player was last moving in previous input
        else if (inputState.IsMoving)
        {
            _targetLookToVector = inputState.MovementVelocity.normalized;
        }

        _appliedLookToVector = Vector3.Slerp(_appliedLookToVector, _targetLookToVector, 1 - Mathf.Pow(1 - _slerpFactor, Time.fixedDeltaTime));
        return _appliedLookToVector;
    }

    private Vector3 GetGravityVector()
    {
        if (_gravityFocus != null)
        {
            _gravityDirection = (_gravityFocus.position - _body.CentreOfMass).normalized;
        }
        Vector3 appliedGravity = _gravityMagnitude * _gravityDirection;

        if (!_isJumpPressed || Vector3.Dot(_body.ForceVelocity.AppliedVelocity, _body.LocalUpwards) < 0f)
        {
            appliedGravity *= _gravityReleaseMultiplier;
        }

        return appliedGravity;
    }

    private Vector3 GetImpulseVector()
    {
        Vector3 impulseVector;

        if (_isJumpQueued)
        {
            _isJumpQueued = false;
            impulseVector = _initialJumpVelocity * -_gravityDirection;
        }
        else
        {
            impulseVector = Vector3.zero;
        }

        return impulseVector;
    }


    // -------------------------------------------------------------------
    // INPUT SYSTEM
    // -------------------------------------------------------------------

    private void OnEnable()
    {
        _playerInput.Enable();
    }

    private void OnDisable()
    {
        _playerInput.Disable();
    }

    private void OnJump(InputAction.CallbackContext context)
    {
        _isJumpPressed = context.ReadValueAsButton();

        if (context.started && (_body.IsOnStableGround || _body.IsHangingOnLedge)) _isJumpQueued = true;
        if (context.canceled) _isJumpQueued = false;
    }

    private void OnRun(InputAction.CallbackContext context)
    {
        _isRunningPressed = context.ReadValueAsButton();
    }

    private void OnMovementInputController(InputAction.CallbackContext context)
    {
        _movementInput = context.ReadValue<Vector2>();
        _isMovementPressed = _movementInput != Vector2.zero;
    }

    private void OnMovementInputKeyboard(InputAction.CallbackContext context)
    {
        _movementInput = context.ReadValue<Vector2>().normalized;
        _isMovementPressed = _movementInput != Vector2.zero;
        _isMovementCanceled = context.canceled;
    }

    private void OnPause(InputAction.CallbackContext context)
    {
        Debug.Log($"Pause");
    }
}
