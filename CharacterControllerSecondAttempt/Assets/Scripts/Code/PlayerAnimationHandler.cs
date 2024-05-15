using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.U2D;
using UnityEngine;
using UnityEngine.InputSystem;

public class PlayerAnimationHandler : MonoBehaviour
{
    [SerializeField] float _speed = 0.1f;

    private Animator _animator;
    private PlayerInputHandler _input;

    private bool _isIdle;
    private bool _isMovingOnGround;

    void OnValidate()
    {
        _animator = GetComponent<Animator>();
        _input = transform.parent.GetComponent<PlayerInputHandler>();

        if ( _input == null )
        {
            Debug.LogError("Player animation handler needs a player input handler. Please add one to the gameObject parent");
            return;
        }
    }
    void Start()
    {
        StartCoroutine(LookAround());    
    }

    void Update()
    {
        float speed = _input.KinematicBody.MovementVelocity.magnitude;

        _isIdle = _input.KinematicBody.IsOnStableGround && Mathf.Approximately(speed, 0f);
        _isMovingOnGround = _input.KinematicBody.IsOnStableGround && !_isIdle;

        float blend;

        if (speed > _input.WalkingSpeed)
        {
            blend = 0.5f + 0.5f * Mathf.Max(0, (speed - _input.WalkingSpeed) / (_input.RunningSpeed - _input.WalkingSpeed));
        }
        else
        {
            blend = 0.5f * speed / _input.WalkingSpeed;
        }


        _animator.SetFloat("Movement", blend);
        _animator.SetBool("IsWalkingOnGround", _isMovingOnGround);
        _animator.SetBool("IsOnStableGround", _input.KinematicBody.IsOnStableGround);
        
        Vector3 localUpwards = _input.KinematicBody.LocalUpwards;
        bool isFalling = Vector3.Dot(_input.KinematicBody.ForceVelocity.AppliedVelocity, localUpwards) < 0f;
        _animator.SetBool("IsFalling", isFalling);
    }

    Vector2 intervalRange = new Vector2(5f, 10f);
    WaitForSeconds checkInterval = new WaitForSeconds(1);
    IEnumerator LookAround()
    {
        while (true)
        {
            if (!_isIdle)
            {
                yield return checkInterval;
            }
            else
            {
                float interval = Random.Range(intervalRange.x, intervalRange.y);

                yield return new WaitForSeconds(interval);

                _animator.Play("IdleMovements.LookAround", -1, 0f);
                Debug.Log($"PLaying LookAround");
            }
        }
    }
}
