using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class PlayerInputHandler : MonoBehaviour, ICharacterController
{
    public KinematicBody KinematicBody { get { return _body; } }
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
    [SerializeField] private float _shimmyAngleTolerance = 5f;


    private float _gravityMagnitude;
    private float _initialJumpVelocity;

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
    private Vector3 _appliedMovementVector = new();
    private float _appliedSpeed = 0f;

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
        Vector2 appliedInput = _forceInput ? _forcedMovementInput : _movementInput;
        float targetSpeed = _isMovementPressed ? _isRunningPressed ? _runningSpeed : _walkingSpeed : 0f;
        if (_forceInput) targetSpeed = _walkingSpeed;

        _appliedSpeed = Mathf.Lerp(_appliedSpeed, targetSpeed, 1- Mathf.Pow(1 - _speedSmoothing, Time.deltaTime));
        inputState.IsMoving = appliedInput.sqrMagnitude > Mathf.Epsilon;

        if (_gravityFocus != null)
        {
            _gravityDirection = (_gravityFocus.position - _body.CentreOfMass).normalized;
        }

        Vector3 appliedGravity = _gravityMagnitude * _gravityDirection;

        // Takes in a transform direction from the camera and converts into a basis vector for horizontal movement
        Vector3 SetBasisFromCamera(Vector3 cameraBasis)
        {
            // 1. Flatten to XZ plane
            Vector3 basis = cameraBasis.With(_body.LocalUpwards, 0f);
            // 2. Project to the ground plane and normalize
            basis = basis.With(_body.GroundNormal, 0f).normalized;
            return basis;
        }

        //_forwardBasis = SetBasisFromCamera(_forceInput ? Vector3.forward : _camera.transform.forward);
        //_rightBasis = SetBasisFromCamera(_forceInput ? Vector3.right : _camera.transform.right);
        _forwardBasis = SetBasisFromCamera(_camera.transform.forward);
        _rightBasis = SetBasisFromCamera(_camera.transform.right);

        // Only change movement vector when moving
        // Ensures that we have decceleration when stopping
        if (targetSpeed > 0f)
        {
            // Applying input. Note that appliedMovementVector is not necessarily normalized (can be <1 for joysticks)
            _appliedMovementVector.x = appliedInput.x * _rightBasis.x + appliedInput.y * _forwardBasis.x;
            _appliedMovementVector.y = appliedInput.x * _rightBasis.y + appliedInput.y * _forwardBasis.y;
            _appliedMovementVector.z = appliedInput.x * _rightBasis.z + appliedInput.y * _forwardBasis.z;
        }
                
        // Different movement modes
        if (_body.IsOnStableGround)
        {
            inputState.MovementVelocity = _appliedMovementVector * _appliedSpeed;
        }
        else if (_body.IsHangingOnLedge)
        {
            // Keep capsule against ledge if moving towards it
            // Otherwise release it from the ledge
            float shimmyAngle = Vector3.Angle(_appliedMovementVector, _body.LedgeNormal);
            if (inputState.IsMoving && shimmyAngle < 90 - _shimmyAngleTolerance)
            {
                inputState.ReleaseFromLedge = true;
            }
            else
            {
                _appliedMovementVector = _appliedMovementVector.With(_body.LedgeNormal, 0f);
            }
            
            inputState.MovementVelocity = _appliedMovementVector * _appliedSpeed;
        }
        else
        {
            // neutralVelocity is the movement velocity required to align the total applied velocity (including force) with the ground normal
            // it acts as an 'origin' point for the movement velocity
            Vector3 neutralVelocity = -_body.ForceVelocity.AppliedVelocity.With(_body.GroundNormal, 0f);
            Vector3 targetAerialVelocity = (neutralVelocity + _maxAerialMovementSpeed * _appliedMovementVector).CapMagnitude(_maxAerialMovementSpeed);
            Vector3 targetDelta = targetAerialVelocity - inputState.MovementVelocity;
            float interpolationFactor = Mathf.Clamp01(Time.fixedDeltaTime * _aerialAcceleration / targetDelta.magnitude);

            inputState.MovementVelocity = Vector3.Lerp(inputState.MovementVelocity, targetAerialVelocity, interpolationFactor);
        }

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
        inputState.LookToVector = _appliedLookToVector;


        if (_isJumpQueued)
        {
            inputState.ImpulseThisFrame = _initialJumpVelocity * -_gravityDirection;
            _isJumpQueued = false;
        }

        if (!_isJumpPressed || Vector3.Dot(_body.ForceVelocity.AppliedVelocity, _body.LocalUpwards) < 0f)
        {
            appliedGravity *= _gravityReleaseMultiplier;
        }

        inputState.Gravity = appliedGravity;
    }

    #region Input system
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
    #endregion
}
