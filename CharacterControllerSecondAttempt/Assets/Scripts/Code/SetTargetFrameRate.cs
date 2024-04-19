using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SetTargetFrameRate : MonoBehaviour
{
    [SerializeField] private int targetFrameRate = 60;

    private void OnValidate()
    {
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = targetFrameRate;
    }
}
