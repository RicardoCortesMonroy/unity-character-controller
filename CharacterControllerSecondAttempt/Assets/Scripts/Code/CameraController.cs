using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class CameraController : MonoBehaviour
{
    [SerializeField] Transform _focus;
    [SerializeField] bool _smoothMotion = true;

    //[SerializeField] float _movementGain = 15;
    [SerializeField] Vector3 _targetRotationEuler = new(30f, 0f, 0f);
    [Range(0f, 1f)]
    [SerializeField] float _startZoom = 0.5f;
    [Range(0.5f, 179.5f)]
    [SerializeField] float _FOV = 70f;
    [Space(10)]
    [SerializeField] float _controllerSensitivity = 12f;
    [SerializeField] float _mouseSensitivityX = 0.05f;
    [SerializeField] float _mouseSensitivityY = 0.05f;

    [SerializeField] float _yawMaxSpeed = 90f; //deg/s
    [SerializeField] float _yawGain = 5f;

    [SerializeField] float _minPitch = 15f; // deg (+ve means looking down)
    [SerializeField] float _maxPitch = 50f; // deg
    [SerializeField] float _pitchMaxSpeed = 45f; // deg/s
    [SerializeField] float _pitchGain = 5f;

    [SerializeField] float _minDistance = 4f; // Distances are normalized for a 70 deg FOV
    [SerializeField] float _maxDistance = 45;
    [SerializeField] float _zoomMaxSpeed = 2f; // normalized (1 means a full zoom in one second)
    [SerializeField] float _zoomGain = 3f; // normalized (1 means that zoom speed will be at maximum only at maximum separation between target and current zoom)

    [SerializeField] float _zoomScrollIncrement = -0.1f; // Negative ensures that scroll up zooms in and scroll down zooms out
    private Camera _camera;
    private PlayerInput _playerInput;
    private Vector2 _rotationInput;
    private Vector3 _smoothedRotationEuler;

    private Vector3 _localUp;
    private Vector3 _localRight;
    private Vector3 _previousLocalUp;

    private Quaternion _focusOrientationSmoothed;
    private float _slerpFactor = 0.99f;

    private bool _enableMouseMovement = false;

    private float _distanceFromPlayer;
    private float _targetZoom;
    private float _dollyCorrection; // factor to correct distance to compensate for FOV
    private float _zoomSmoothed;
    private float _sizeRange;

    private void Awake()
    {
        _playerInput = new();

        _playerInput.CharacterControls.CameraController.started += OnRotateByController;
        _playerInput.CharacterControls.CameraController.performed += OnRotateByController;
        _playerInput.CharacterControls.CameraController.canceled += OnRotateByController;

        _playerInput.CharacterControls.CameraMouse.started += OnRotateByMouse;
        _playerInput.CharacterControls.CameraMouse.performed += OnRotateByMouse;
        _playerInput.CharacterControls.CameraMouse.canceled += OnRotateByMouse;

        _playerInput.CharacterControls.Zoom.started += OnZoom;
        _playerInput.CharacterControls.Zoom.performed += OnZoom;
        _playerInput.CharacterControls.Zoom.canceled += OnZoom;

        _playerInput.CharacterControls.ZoomScroll.started += OnZoomScroll;

        _playerInput.CharacterControls.Pause.started += OnPause;

        _playerInput.CharacterControls.LeftClick.started += OnToggleMouseControl;


    }

    private void OnToggleMouseControl(InputAction.CallbackContext obj)
    {
        _enableMouseMovement = !_enableMouseMovement;
        Cursor.lockState = _enableMouseMovement ? CursorLockMode.Locked : CursorLockMode.None;
    }

    private void Reset()
    {
        _camera = GetComponent<Camera>();
        _camera.orthographic = true;
    }


    private void OnValidate()
    {
        _camera = GetComponent<Camera>();
        _sizeRange = _maxDistance - _minDistance;
        _smoothedRotationEuler = _targetRotationEuler;
        _camera.orthographic = false;
        _camera.fieldOfView = _FOV;

        _targetZoom = _startZoom;
        _zoomSmoothed = _startZoom;

        _dollyCorrection = Mathf.Tan(70f * Mathf.PI / 360f) / Mathf.Tan(_camera.fieldOfView * Mathf.PI / 360f);
        _localUp = Vector3.up;
        _localRight = Vector3.right;

        if (_focus != null) Update();
    }



    void Update()
    {
        _previousLocalUp = _localUp;
        _localUp = _focus.transform.up;
        Quaternion deltaRotation = Quaternion.FromToRotation(_previousLocalUp, _localUp);
        _localRight = deltaRotation * _localRight;

        // Yaw
        _targetRotationEuler += Time.deltaTime * _yawMaxSpeed * _rotationInput.x * Vector3.up;
        _smoothedRotationEuler.y += Time.deltaTime * Mathf.Clamp((_targetRotationEuler.y - _smoothedRotationEuler.y) * _yawGain, -_yawMaxSpeed, _yawMaxSpeed);

        // Pitch
        _targetRotationEuler += Time.deltaTime * _pitchMaxSpeed * -_rotationInput.y * Vector3.right;
        _targetRotationEuler.x = Mathf.Clamp(_targetRotationEuler.x, _minPitch, _maxPitch);
        _smoothedRotationEuler.x += Time.deltaTime * Mathf.Clamp((_targetRotationEuler.x - _smoothedRotationEuler.x) * _pitchGain, -_pitchMaxSpeed, _pitchMaxSpeed);

        // Distance
        _zoomSmoothed += Time.deltaTime * _zoomMaxSpeed * _sizeRange * Mathf.Clamp(_zoomGain * (_targetZoom - _zoomSmoothed) / _sizeRange, -1f, 1f);
        _distanceFromPlayer = _dollyCorrection * Mathf.Lerp(_minDistance, _maxDistance, _zoomSmoothed);

        RefreshPositionAndRotation();
    }

    private void RefreshPositionAndRotation()
    {
        // Rotation of the camera in default player orientation
        Quaternion localRotation = Quaternion.Euler(_smoothMotion ? _smoothedRotationEuler : _targetRotationEuler);

        // Orientation of the player
        Vector3 localForward = Vector3.Cross(_localUp, _localRight);
        Quaternion targetFocusOrientation = Quaternion.LookRotation(localForward, _localUp);
        _focusOrientationSmoothed = Quaternion.Slerp(_focusOrientationSmoothed, targetFocusOrientation, 1 - Mathf.Pow(1 - _slerpFactor, Time.fixedDeltaTime));
        
        Quaternion appliedRotation = _focusOrientationSmoothed * localRotation;
        Vector3 appliedPosition = _focus.position + appliedRotation * (-_distanceFromPlayer * Vector3.forward);
        transform.SetPositionAndRotation(appliedPosition, appliedRotation);
    }

    private void OnEnable()
    {
        _playerInput.Enable();
    }

    private void OnDisable()
    {
        _playerInput.Disable();
    }

    private void OnRotateByController(InputAction.CallbackContext context)
    {
        _rotationInput = _controllerSensitivity * context.ReadValue<Vector2>();
    }

    private void OnRotateByMouse(InputAction.CallbackContext context)
    {
        if (!_enableMouseMovement) return;

        Vector2 value = context.ReadValue<Vector2>();

        _rotationInput = new Vector2 (
            _mouseSensitivityX * value.x,
            _mouseSensitivityY * value.y
        );
    }

    private void OnZoom(InputAction.CallbackContext context)
    {
        _targetZoom = context.ReadValue<float>();
    }

    private void OnZoomScroll(InputAction.CallbackContext context)
    {
        float scrollDelta = Mathf.Clamp(context.ReadValue<float>(), -1f, 1f);
        _targetZoom = Mathf.Clamp01(_targetZoom + scrollDelta * _zoomScrollIncrement);
    }

    private void OnPause(InputAction.CallbackContext obj)
    {
        Debug.Break();
    }
}
