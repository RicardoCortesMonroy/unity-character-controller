using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public static class GizmoExtensions
{
    public static void DrawWireCube(Vector3 center, Vector3 extents, Quaternion orientation)
    {
        List<Vector3> corners = new List<Vector3>();

        for (int sz = -1; sz <= 1; sz+=2)
        {
            for (int sy = -1; sy <= 1; sy += 2)
            {
                for (int sx = -1; sx <= 1; sx += 2)
                {
                    Vector3 corner = new Vector3
                    (
                        sx * extents.x,
                        sy * extents.y,
                        sz * extents.z
                    );

                    corner = orientation * corner;
                    corner += center;
                    corners.Add( corner );
                }
            }
        }

        // Draw all 12 lines of the cube
        Gizmos.DrawLine(corners[0], corners[1]);
        Gizmos.DrawLine(corners[0], corners[2]);
        Gizmos.DrawLine(corners[0], corners[4]);
        Gizmos.DrawLine(corners[3], corners[1]);
        Gizmos.DrawLine(corners[3], corners[2]);
        Gizmos.DrawLine(corners[3], corners[7]);
        Gizmos.DrawLine(corners[5], corners[1]);
        Gizmos.DrawLine(corners[5], corners[4]);
        Gizmos.DrawLine(corners[5], corners[7]);
        Gizmos.DrawLine(corners[6], corners[2]);
        Gizmos.DrawLine(corners[6], corners[4]);
        Gizmos.DrawLine(corners[6], corners[7]);
    }

    public static void DrawSphere(Vector3 center, float radius)
    {
        Quaternion cameraRotation = SceneView.lastActiveSceneView.camera.transform.rotation;
        Vector3 normal = cameraRotation * -Vector3.forward;

        Handles.color = Gizmos.color;
        Handles.DrawWireDisc(center, normal, radius);
    }
}
