using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class PlayerInputHandler : MonoBehaviour, ICharacterController
{
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

    // Boolean input
    private bool _isJumpPressed;
    private bool _isJumpQueued;
    private bool _isRunningPressed;
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

    private void Start()
    {
        _body.StableGroundThreshold = _stableGroundThreshold;
    }

    public void UpdateInputState(ref InputState inputState)
    {
        Vector2 appliedInput = _forceInput ? _forcedMovementInput : _movementInput;
        float appliedSpeed = _isRunningPressed ? _runningSpeed : _walkingSpeed;
        inputState.IsMoving = appliedInput.sqrMagnitude > Mathf.Epsilon;

        if (_gravityFocus != null)
        {
            _gravityDirection = (_gravityFocus.position - transform.position).normalized;
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

        _forwardBasis = SetBasisFromCamera(_forceInput ? Vector3.forward : _camera.transform.forward);
        _rightBasis = SetBasisFromCamera(_forceInput ? Vector3.right : _camera.transform.right);

        Vector3 appliedMovementVector = new();

        // Applying input. Note that appliedMovementDirection is not necessarily normalized (can be <1 for joysticks)
        appliedMovementVector.x = appliedInput.x * _rightBasis.x + appliedInput.y * _forwardBasis.x;
        appliedMovementVector.y = appliedInput.x * _rightBasis.y + appliedInput.y * _forwardBasis.y;
        appliedMovementVector.z = appliedInput.x * _rightBasis.z + appliedInput.y * _forwardBasis.z;

        // Immediate movement if grounded, accelerated movement if in air
        if (_body.IsOnStableGround)
        {
            inputState.MovementVelocity = appliedMovementVector * appliedSpeed;
        }
        else
        {
            // neutralVelocity is the movement velocity required to align the total applied velocity (including force) with the ground normal
            // it acts as an 'origin' point for the movement velocity
            Vector3 neutralVelocity = -_body.ForceVelocity.AppliedVelocity.With(_body.GroundNormal, 0f);
            Vector3 targetAerialVelocity = (neutralVelocity + _maxAerialMovementSpeed * appliedMovementVector).CapMagnitude(_maxAerialMovementSpeed);
            Vector3 targetDelta = targetAerialVelocity - inputState.MovementVelocity;
            float interpolationFactor = Mathf.Clamp01(Time.fixedDeltaTime * _aerialAcceleration / targetDelta.magnitude);

            inputState.MovementVelocity = Vector3.Lerp(inputState.MovementVelocity, targetAerialVelocity, interpolationFactor);
        }

        // Only update active movement direction when movement is pressed.
        // Keeps track of how the player was last moving in previous input
        if (inputState.IsMoving)
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
        // Only trigger jump on the frame that you press jump if you're grounded (prevents holding jump to jump repeatedly)
        if (context.started && _body.IsOnStableGround) _isJumpQueued = true;
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
