#if UNITY_EDITOR
using Introvert.RVO2;
using Unity.Entities;
using UnityEngine;

namespace Xacce.Introvert.Hybrid
{
    [DisallowMultipleComponent]
    public class IntrovertAgentAuthoring : MonoBehaviour
    {
        private class IntrovertRvo2AgentBaker : Baker<IntrovertAgentAuthoring>
        {
            public override void Bake(IntrovertAgentAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.None);
                AddComponent<IntrovertAgent>(entity);
            }
        }

    }

}
#endif