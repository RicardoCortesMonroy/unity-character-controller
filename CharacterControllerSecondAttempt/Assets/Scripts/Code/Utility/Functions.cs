using System;
using UnityEditor;

public static class Functions
{
    public static int Mod(int x, int m)
    {
        return x - (x / m) * m;
    }

}
