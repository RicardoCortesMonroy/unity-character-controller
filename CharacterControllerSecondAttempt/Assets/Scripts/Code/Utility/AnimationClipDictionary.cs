using System.Collections.Generic;
using UnityEditor.Animations;
using UnityEngine;

public class AnimationClipDictionary
{
    public Dictionary<string, float> Dict { get; private set; }

    private Animator _anim;

    public AnimationClipDictionary(Animator anim)
    {
        _anim = anim;
        Dict = new();
        PopulateDictionary();
    }

    private void PopulateDictionary()
    {
        AnimationClip[] animationClips = _anim.runtimeAnimatorController.animationClips;

        // Get raw clip length
        foreach (var clip in animationClips)
        {
            if (!Dict.ContainsKey(clip.name))
            {
                Dict.Add(clip.name, clip.length);
            }
        }

        var ac = (AnimatorController)_anim.runtimeAnimatorController;
        var bt = (BlendTree)ac.layers[0].stateMachine.states[0].state.motion;
        ChildMotion[] childMotions = bt.children;

        for (int i = 0; i < childMotions.Length; i++)
        {
            var child = childMotions[i];
            string motionName = child.motion.name;
            float timeScale = child.timeScale;
            Dict[motionName] /= timeScale;
        }
    }
}