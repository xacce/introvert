#if UNITY_EDITOR
using System.Collections.Generic;
using Core.Runtime;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace Introvert.PolyObstacle.Authoring
{
    public class PolyObstacleLineAuthoring : MonoBehaviour, IPolyObstacle
    {
        [SerializeField] private Vector3 offset;
        [SerializeField] private float segments;
        [SerializeField] private float segmentLength = 3f;
        [SerializeField] private float overlap = 0.1f;
        [SerializeField] private XaObjectType type = XaObjectType.Static;
        private float length => segments * (segmentLength+overlap);
        private void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(transform.position + offset - transform.forward * (length / 2), transform.forward * (length));
            Gizmos.color = Color.white;
        }

        public void GetWrapped(Vector3 position, IBaker baker)
        {
            var pos = position + offset + transform.forward * (length / 2);
            for (int i = 0; i < segments; i++)
            {
                var ch = baker.CreateAdditionalEntity(TransformUsageFlags.WorldSpace, true);
                baker.AddComponent(
                    ch,
                    new LocalToWorld()
                    {
                        Value = float4x4.TRS(pos, quaternion.identity, new float3(1f)),

                    });
                var b = baker.AddBuffer<Unwrap>(ch);

                b.Add(
                    new Unwrap()
                    {
                        entity = baker.CreateAdditionalEntity(TransformUsageFlags.WorldSpace),
                        position = pos,
                    });
                b.Add(
                    new()
                    {
                        entity = baker.CreateAdditionalEntity(TransformUsageFlags.WorldSpace),
                        position = pos - transform.forward * (segmentLength + overlap),
                    });
                pos += transform.forward * segmentLength;
            }

        }
    }
}
#endif