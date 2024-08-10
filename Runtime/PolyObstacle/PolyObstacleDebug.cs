using System;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace Introvert.PolyObstacle
{
    public partial struct PolyObstacleDebugSystem : ISystem
    {
        public void OnUpdate(ref SystemState state)
        {
            foreach (var (ltw, obstacle) in SystemAPI.Query<LocalToWorld, Xacce.Introvert.Runtime.PolyObstacle.PolyObstacle>())
            {
                // Debug.DrawLine(new float3(obstacle.p1.p.x, ltw.Position.y, obstacle.p1.p.y), new float3(obstacle.p2.p.x, ltw.Position.y, obstacle.p2.p.y), Color.blue, 0.1f);
                // Debug.DrawRay(new float3(obstacle.p1.p.x, ltw.Position.y, obstacle.p1.p.y), new float3(obstacle.p1.direction.x, 0f, obstacle.p1.direction.y), Color.green, 0.1f);
                // Debug.DrawRay(new float3(obstacle.p1.p.x, ltw.Position.y, obstacle.p1.p.y), new float3(obstacle.p1.prevInvDirection.x, 0f, obstacle.p1.prevInvDirection.y), Color.red, 0.1f);
                //
                // Debug.DrawRay(new float3(obstacle.p2.p.x, ltw.Position.y, obstacle.p2.p.y), new float3(obstacle.p2.direction.x, 0f, obstacle.p2.direction.y), Color.green, 0.1f);
                // Debug.DrawRay(new float3(obstacle.p2.p.x, ltw.Position.y, obstacle.p2.p.y), new float3(obstacle.p2.prevInvDirection.x, 0f, obstacle.p2.prevInvDirection.y), Color.red, 0.1f);
            }
        }
    }
}