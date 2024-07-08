#if UNITY_EDITOR
using Core.Runtime;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace Introvert.PolyObstacle.Authoring
{
    [RequireComponent(typeof(PolyObstacleAuthoring))]
    public class PolyObstaclePlaneAuthoring : MonoBehaviour, IPolyObstacle
    {
        [SerializeField] private Vector3 offset;
        [SerializeField] private float2 extents;
        [SerializeField] private XaObjectType type = XaObjectType.Static;

        private void OnDrawGizmos()
        {

            Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one);
            Gizmos.color = Color.blue;
            Gizmos.DrawWireCube(offset, new Vector3(extents.x, 0f, extents.y));
            Gizmos.color = Color.white;
            Gizmos.matrix = Matrix4x4.identity;
        }

        public void GetWrapped(Vector3 position, IBaker baker)
        {
            var ch = baker.CreateAdditionalEntity(TransformUsageFlags.WorldSpace, true);
            baker.AddComponent(
                ch,
                new LocalToWorld()
                {
                    Value = float4x4.TRS(position, quaternion.identity, new float3(1f)),

                });
            var b = baker.AddBuffer<Unwrap>(ch);
            var t = extents / 2;
            var p = transform.position + offset;
            b.Add(
                new Unwrap()
                {
                    entity = baker.CreateAdditionalEntity(TransformUsageFlags.WorldSpace),
                    position = p + (transform.rotation * new float3(-t.x, 0f, -t.y)),
                });
            b.Add(
                new Unwrap()
                {
                    entity = baker.CreateAdditionalEntity(TransformUsageFlags.WorldSpace),
                    position = p + (transform.rotation * new float3(t.x, 0f, -t.y)),
                });
            b.Add(
                new Unwrap()
                {
                    entity = baker.CreateAdditionalEntity(TransformUsageFlags.WorldSpace),
                    position = p + (transform.rotation * new float3(t.x, 0f, t.y)),
                });
            b.Add(
                new Unwrap()
                {
                    entity = baker.CreateAdditionalEntity(TransformUsageFlags.WorldSpace),
                    position = p + (transform.rotation * new float3(-t.x, 0f, t.y)),
                });
        }


    }
}
#endif