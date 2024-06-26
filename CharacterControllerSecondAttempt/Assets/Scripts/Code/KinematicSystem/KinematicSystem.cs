using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[DefaultExecutionOrder(-100)]
public class KinematicSystem : MonoBehaviour
{
    private static KinematicSystem _instance;
    public static KinematicSystem Instance
    {
        get
        {
            EnsureCreation();
            return _instance;
        }
    }

    private static Dictionary<int, KinematicBody> _bodies = new();
    private static Dictionary<int, KinematicMover> _movers = new();

    private bool _interpolate = false;
    private bool _alternateSteps = false;

    private int _fixedFrame;
    private float _simulationStartTime;

    public static void EnsureCreation()
    {
        if (_instance == null)
        {
            GameObject systemGameObject = new GameObject("Kinematic System");
            _instance = systemGameObject.AddComponent<KinematicSystem>();

            //systemGameObject.hideFlags = HideFlags.NotEditable;
            //_instance.hideFlags = HideFlags.NotEditable;

            GameObject.DontDestroyOnLoad(systemGameObject);
        }
    }

    private void Awake()
    {
        // IMPORTANT: Ensures that colliders update as soon as their respective transforms are updated,
        // as opposed to waiting for the end of the physics update for the transforms to apply.
        // If this is false, then bodies will detect movers a frame late so collisions will be incorrect
        Physics.autoSyncTransforms = true;
    }


    private void Start()
    {
        foreach (var body in _bodies.Values)
        {
            body.CollisionCheck();
            body.ApplyTransientTransform();
        }
    }

    private void FixedUpdate()
    {
        _simulationStartTime = Time.time;

        //Debug.Log($"----------------------Physics frame: {_fixedFrame}----------------------");

        if (!_alternateSteps || _fixedFrame % 2 == 0 )
        {
            //Debug.Log($"----------------------Mover frame: {_fixedFrame/2}----------------------");
            foreach (var body in _bodies.Values)
            {
                body.ApplyTransientTransform();
            }

            foreach (var mover in _movers.Values)
            {
                mover.CalculateVelocity();

                if (mover.UseCCD) mover.SweepForBodies();

                mover.Simulate();
                mover.ApplyTransientTransform();
            }
        }

        if (!_alternateSteps || _fixedFrame % 2 == 1)
        {
            //Debug.Log($"----------------------Body frame: {_fixedFrame / 2}----------------------");
            foreach (var body in _bodies.Values)
            {
                body.UpdateCurrentPositionAndRotation();
                body.ApplyMoverDisplacement();

                body.CalculateVelocity();
                body.Simulate();
                body.CollisionCheck();
                body.CheckForEdgeSnapping();
                body.CheckForLedgeGrabbing();

                body.ApplyTransientTransform();
            }
        }
        _fixedFrame++;
    }

    private void Update()
    {
        foreach (var mover in _movers.Values)
        {
            mover.HandleInterpolation(_simulationStartTime, Time.fixedDeltaTime, _interpolate);
        }

        foreach (var body in _bodies.Values)
        {
            body.HandleInterpolation(_simulationStartTime, Time.fixedDeltaTime, _interpolate);
        }
    }

    public static void AddBody(KinematicBody kinematicBody)
    {
        _bodies.Add(kinematicBody.gameObject.GetInstanceID(), kinematicBody);
    }

    public static void RemoveBody(KinematicBody kinematicBody)
    {
        _bodies.Remove(kinematicBody.gameObject.GetInstanceID());
    }

    public static void AddMover(KinematicMover kinematicMover)
    {
        _movers.Add(kinematicMover.gameObject.GetInstanceID(), kinematicMover);
    }

    public static void RemoveMover(KinematicMover kinematicMover)
    {
        _movers.Remove(kinematicMover.gameObject.GetInstanceID());
    }

    public static KinematicMover CheckMover(int gameObjectId)
    {
        if (_movers.ContainsKey(gameObjectId))
        {
            return _movers[gameObjectId];
        }

        return null;
    }

    private void OnApplicationQuit()
    {
        Destroy(this.gameObject);
    }

}
