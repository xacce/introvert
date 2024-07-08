using Core.Runtime;
using RVO;
using SpatialHashing.Uniform;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Introvert.PolyObstacle.Authoring
{
    [TemporaryBakingType]
    public partial struct Unwrap : IBufferElementData
    {
        public Entity entity;
        public float3 position;

    }

    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    public partial struct PolyObstacleBakingSystem : ISystem
    {
        private float2 As2d(float3 d3)
        {
            return new float2(d3.x, d3.z);
        }

        public void OnUpdate(ref SystemState state)
        {
            var ecb = new EntityCommandBuffer(Allocator.TempJob);
            foreach (var (ltw, points, entity) in SystemAPI.Query<LocalToWorld, DynamicBuffer<Unwrap>>().WithEntityAccess())
            {
                for (int i = 0; i < points.Length; i++)
                {
                    var prev = points[i == 0 ? points.Length - 1 : i - 1];
                    var curr = points[i];
                    var nextI = i == points.Length - 1 ? 0 : i + 1;
                    var next = points[nextI];
                    var nextNext = points[nextI == points.Length - 1 ? 0 : nextI + 1];
                    var prevPos = As2d(prev.position);
                    var curPos = As2d(curr.position);
                    var nextPos = As2d(next.position);
                    var nextNextPos = As2d(nextNext.position);
                    var obstacle = new PolyObstacle()
                    {
                        p1 = new PolyObstacle.Point()
                        {
                            p = curPos,
                            convex = RVOMath.leftOf(prevPos, curPos, nextPos) >= 0.0f,
                            direction = math.normalize(nextPos - curPos),
                            prevInvDirection = math.normalize(curPos - prevPos),
                            index = 0,
                        },
                        p2 = new PolyObstacle.Point()
                        {
                            p = nextPos,
                            convex = RVOMath.leftOf(curPos, nextPos, nextNextPos) >= 0.0f,
                            direction = math.normalize(nextNextPos - nextPos),
                            prevInvDirection = math.normalize(nextPos - curPos),
                            index = 1,
                        },
                    };

                    ecb.RemoveComponent<Parent>(points[i].entity);
                    ecb.RemoveComponent<LocalTransform>(points[i].entity);
                    ecb.AddComponent(
                        points[i].entity,
                        new UniformSpatialNode()
                        {
                            type = XaObjectType.Static,
                        });
                    ecb.AddComponent(points[i].entity, obstacle);
                    ecb.AddComponent(
                        points[i].entity,
                        new LocalToWorld()
                        {
                            Value = float4x4.TRS(
                                new float3(curPos.x, ltw.Position.y, curPos.y) + new float3((nextPos - curPos).x, ltw.Position.y, (nextPos - curPos).y),
                                quaternion.identity,
                                new float3(1f)),
                        });
                }
                ecb.RemoveComponent<LocalToWorld>(entity);
                ecb.RemoveComponent<LocalTransform>(entity);
                ecb.RemoveComponent<Child>(entity);
                ecb.RemoveComponent<LinkedEntityGroup>(entity);
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();

        }
    }
}