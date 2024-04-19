using UnityEditor;
using UnityEngine;

public static class ExtensionMethods
{
    public static Vector3 With(this Vector3 vector, float? x = null, float? y = null, float? z = null)
    {
        return new Vector3(x ?? vector.x, y ?? vector.y, z ?? vector.z);
    }

    // sets the magnitude of a vector to a given value in a normalized direction
    public static Vector3 With(this Vector3 vector, Vector3 component, float magnitude)
    {
        component = component.normalized;
        float currentMagnitude = Vector3.Dot(vector, component);
        Vector3 output = vector + (magnitude - currentMagnitude) * component;
        return output;
    }

    public static Vector3 ProjectOnPlane(this Vector3 vector, Vector3 normal)
    {
        return vector.With(normal, 0f);
    }

    public static Vector3 Constrain(this Vector3 vector, Vector3 component, float min = -Mathf.Infinity, float max = Mathf.Infinity)
    {
        component = component.normalized;
        float currentMagnitude = Vector3.Dot(vector, component);
        float magnitude = Mathf.Clamp(min, currentMagnitude, max);
        Vector3 output = vector + (magnitude - currentMagnitude) * component;
        return output;
    }

    public static float GetComponentMagnitude(this Vector3 vector, Vector3 component)
    {
        return Vector3.Dot(vector, component);
    }

    public static Vector3 GetComponent(this Vector3 vector, Vector3 component)
    {
        component = component.normalized;
        float magnitude = Vector3.Dot(vector, component);
        return magnitude * component;
    }

    public static Vector2 GetComponent(this Vector2 vector, Vector2 component)
    {
        component = component.normalized;
        float magnitude = Vector2.Dot(vector, component);
        return magnitude * component;
    }

    public static Vector3 CapMagnitude(this Vector3 vector, float magnitude)
    {
        magnitude = Mathf.Abs(magnitude);
        float currentMagnitude = vector.magnitude;
        if (currentMagnitude == 0f) return vector;
        float multiplier = Mathf.Min(currentMagnitude, magnitude) / currentMagnitude;
        return multiplier * vector ;
    }

    public static Vector2 CapMagnitude(this Vector2 vector, float magnitude)
    {
        magnitude = Mathf.Abs(magnitude);
        float currentMagnitude = vector.magnitude;
        if (currentMagnitude == 0f) return vector;
        float multiplier = Mathf.Min(currentMagnitude, magnitude) / currentMagnitude;
        return multiplier * vector;
    }

    public static string ToFullString(this Vector3 vector)
    {
        return $"({vector.x},{vector.y},{vector.z})";
    }
}