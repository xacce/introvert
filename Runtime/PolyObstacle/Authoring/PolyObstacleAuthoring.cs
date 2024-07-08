#if UNITY_EDITOR
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace Introvert.PolyObstacle.Authoring
{
    public class PolyObstacleAuthoring : MonoBehaviour
    {

        private class PolyObstacleAuthoringB : Baker<PolyObstacleAuthoring>
        {
            public override void Bake(PolyObstacleAuthoring authoring)
            {
                var e = GetEntity(TransformUsageFlags.WorldSpace);
                foreach (var provider in authoring.GetComponents<IPolyObstacle>())
                {
                    provider.GetWrapped(authoring.transform.position, this);
                }
            }
        }
    }
}
#endif