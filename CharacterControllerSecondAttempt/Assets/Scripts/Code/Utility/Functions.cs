using System;
using UnityEditor;

public static class Functions
{
    public static int Mod(int x, int m)
    {
        return x - (x / m) * m;
    }

    // Returns a blending parameter to blend between states 1 and 2 that accounts for the different lengths of the respective animation clips
    // dim refers to the dimension that you're basing the blending on (e.g. movement speed)
    // when dim = dim_1, then x = 0,
    // when dim = dim_2, then x = 1,
    public static float AccurateBlend(float dim, float dim_1, float dim_2, float length_1, float length_2)
    {
        // trust me the maths checks out
        float x = length_1 * (dim_1 - dim) / (length_1 * (dim_1 - dim) - length_2 * (dim_2 - dim));
        return x;
    }
}
