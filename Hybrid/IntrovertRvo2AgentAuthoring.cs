#if UNITY_EDITOR
using Unity.Entities;
using UnityEngine;

namespace Introvert.RVO2.Authoring
{
    [DisallowMultipleComponent]
    public class IntrovertRvo2AgentAuthoring : MonoBehaviour
    {

        private class IntrovertRvo2AgentBaker : Baker<IntrovertRvo2AgentAuthoring>
        {
            public override void Bake(IntrovertRvo2AgentAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.None);
                AddComponent<IntrovertRvoAgent>(entity);
            }
        }

    }

}
#endif