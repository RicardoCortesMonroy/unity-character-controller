using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.U2D;
using UnityEngine;
using UnityEngine.InputSystem;


// The gameobject this script is attached to should be:
//  1. The child of an object containing the kinematic body and input handler
//  2. The parent of the root of the armature
//  3. The parent of the gameObject containing the mesh

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


        bool movingLeft = Vector3.Dot(_input.WorldInput, transform.right) < Mathf.Epsilon;
        bool flip = movingLeft && _input.KinematicBody.IsHangingOnLedge;
        FlipCharacterHorizontal(flip);


        _animator.SetFloat("Movement", blend);
        _animator.SetBool("IsWalkingOnGround", _isMovingOnGround);
        _animator.SetBool("IsOnStableGround", _input.KinematicBody.IsOnStableGround);
        _animator.SetBool("IsHangingOnLedge", _input.KinematicBody.IsHangingOnLedge);
        _animator.SetBool("IsMoving", speed > Mathf.Epsilon);

        Vector3 localUpwards = _input.KinematicBody.LocalUpwards;
        bool isFalling = Vector3.Dot(_input.KinematicBody.ForceVelocity.AppliedVelocity, localUpwards) < 0f;
        _animator.SetBool("IsFalling", isFalling);
    }

    private void FlipCharacterHorizontal(bool flip)
    {
        float scaleX = flip ? -1f : 1f;
        transform.localScale = new Vector3(scaleX, 1f, 1f);
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
