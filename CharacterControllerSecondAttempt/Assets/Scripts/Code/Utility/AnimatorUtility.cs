using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.Animations;
using UnityEngine;


public class AnimatorUtility
{

    public Animator Anim;
    public AnimatorController AnimatorController;
    public Dictionary<string, BlendTree> BlendTrees;


    public AnimatorUtility(GameObject gameObject)
    {
        Anim = gameObject.GetComponent<Animator>();
        AnimatorController = (AnimatorController) Anim.runtimeAnimatorController;

        if (AnimatorController != null)
        {
            FindBlendTrees(AnimatorController);
        }
    }

    private void FindBlendTrees(AnimatorController controller)
    {
        // Iterate through:
        // - layers
        // - statemachines
        // - child statemachines
        // - motions (if blendTrees)
        // and add all blendTrees to the list

        Debug.Log($"Locating blendTrees in {controller.name}");

        foreach (AnimatorControllerLayer layer in controller.layers)
        {
            Debug.Log($"Layer: {layer.name}");

            foreach (ChildAnimatorState childState in layer.stateMachine.states)
            {
                if (childState.state.motion is BlendTree blendTree)
                {
                    BlendTrees.Add(blendTree.name, blendTree);
                }
            }
        }
    }


    // Returns a blending parameter to blend between states 1 and 2 that accounts for the different lengths of the respective animation clips
    // dim refers to the dimension that you're basing the blending on (e.g. movement speed)
    // when dim = dim_1, then x = 0,
    // when dim = dim_2, then x = 1,
    private float AccurateBlend(float dim, float dim_1, float dim_2, float length_1, float length_2)
    {
        // trust me the maths checks out
        float x = length_1 * (dim_1 - dim) / (length_1 * (dim_1 - dim) - length_2 * (dim_2 - dim));
        return x;
    }
}



