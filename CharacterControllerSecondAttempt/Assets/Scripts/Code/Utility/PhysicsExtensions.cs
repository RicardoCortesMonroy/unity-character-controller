using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using UnityEngine;

public static class PhysicsExtensions
{
    public static void CapsuleOverlap(Vector3 capsuleTop, Vector3 capsuleBottom, float radius, Mesh mesh, out float closestDistance, out Vector3 depenetrationDirection)
    {
        int closestVertex = ClosestVertexToCapsule(capsuleTop, capsuleBottom, radius, mesh);
        int[] connectedVertices = FindConnectedVertices(mesh, closestVertex);

        closestDistance = float.MaxValue;
        depenetrationDirection = Vector3.up;

        for (int i = 0; i < connectedVertices.Length; i++)
        {
            Vector3 closestPathFromCapsuleToEdge = ClosestPathBetweenTwoEdges(
                P11: capsuleBottom,
                P12: capsuleTop,
                P21: mesh.vertices[closestVertex],
                P22: mesh.vertices[connectedVertices[i]]
                );

            float distance = closestPathFromCapsuleToEdge.magnitude;
            
            if (distance < closestDistance)
            {
                closestDistance = distance;
                depenetrationDirection = -closestPathFromCapsuleToEdge.normalized;
            }
        }
    }

    // Returns closest vector from Edge 1 (P11->P12) to Edge 2 (P21->P22)
    private static Vector3 ClosestPathBetweenTwoEdges(Vector3 P11, Vector3 P12, Vector3 P21, Vector3 P22)
    {
        Vector3 edge1 = P12 - P11;
        Vector3 edge2 = P22 - P21;

        Vector3 deltaBase = P21 - P11;

        float A11 = Vector3.Dot(edge2, edge1);
        float A12 = -Vector3.Dot(edge1, edge1);
        float A13 = Vector3.Dot(deltaBase, edge1);
        float A21 = Vector3.Dot(edge2, edge2);
        float A22 = -A11;
        float A23 = Vector3.Dot(deltaBase, edge2);

        Matrix<float> A = DenseMatrix.OfArray(new float[,]
        {
            { A11, A12 },
            { A21, A22 }
        });

        Vector<float> B = Vector<float>.Build.Dense(new float[] { A13, A23 });

        Vector<float> solution = A.Solve(B);

        float t1 = Mathf.Clamp(solution[0], 0f, 1f);
        float t2 = Mathf.Clamp(solution[1], 0f, 1f);

        Vector3 Q1 = P11 + t1 * edge1;
        Vector3 Q2 = P21 + t2 * edge2;

        return Q2 - Q1;
    }

    private static int ClosestVertexToCapsule(Vector3 capsuleTop, Vector3 capsuleBottom, float radius, Mesh mesh)
    {
        int closestVertex = -1;

        float shortestDistance = float.MaxValue;
        Vector3 capsuleAxis = capsuleTop - capsuleBottom;
        Vector3 capsuleAxisNorm = capsuleAxis.normalized;
        float capsulAxisLength = capsuleAxis.magnitude;

        for (int i = 0; i < mesh.vertices.Length; i++)
        {
            Vector3 vertex = mesh.vertices[i];
            Vector3 capsuleBottomToVertex = vertex - capsuleBottom;
            float vertexProjectedOnAxis = Vector3.Dot(capsuleBottomToVertex, capsuleAxisNorm);
            vertexProjectedOnAxis = Mathf.Clamp(vertexProjectedOnAxis, 0.0f, capsulAxisLength);

            Vector3 vertexToClosestPoint = -capsuleBottomToVertex + vertexProjectedOnAxis * capsuleAxis;
            float distanceToCapsule = vertexToClosestPoint.magnitude;


            if (distanceToCapsule < shortestDistance)
            {
                shortestDistance = distanceToCapsule;
                closestVertex = i;
            }
        }

        return closestVertex;
    }

    private static int[] FindConnectedVertices(Mesh mesh, int targetVertexIndex)
    {
        List<int> connectedVertices = new List<int>();
        int[] triangles = mesh.triangles;

        for (int i = 0; i < triangles.Length; i += 3)
        {
            // Check if the target vertex is part of the current triangle
            if (triangles[i] == targetVertexIndex ||
                triangles[i + 1] == targetVertexIndex ||
                triangles[i + 2] == targetVertexIndex)
            {
                connectedVertices.Add(triangles[i]);
                connectedVertices.Add(triangles[i + 1]);
                connectedVertices.Add(triangles[i + 2]);
            }
        }

        // Remove duplicates and the target vertex itself
        connectedVertices = connectedVertices.Distinct().Where(v => v != targetVertexIndex).ToList();

        return connectedVertices.ToArray();
    }

}
