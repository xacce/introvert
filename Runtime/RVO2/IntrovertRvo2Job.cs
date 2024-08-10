using Core.Runtime;
using GameReady.Runtime;
using Introvert.RVO2;
using Pathfinding;
using RVO;
using SpatialHashing.Uniform;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Xacce.BlobActor.Runtime;

namespace Xacce.Introvert.Runtime.RVO2
{
    public struct Visitor : IUniformSpatialQueryCollector
    {
        public NativeList<UniformSpatialElement> neighbors;
        public NativeList<UniformSpatialElement> obstacles;

        public void OnVisit(in UniformSpatialCell cell, in UnsafeList<UniformSpatialElement> elements, out bool shouldEarlyExit)
        {
            for (int i = 0; i < cell.length; i++)
            {
                var element = elements[cell.start + i];
                if ((element.type & XaObjectType.Character) != 0)
                {
                    neighbors.Add(element);
                }
                else if ((element.type & XaObjectType.Static) != 0)
                {
                    obstacles.Add(element);
                }
            }
            shouldEarlyExit = false;
        }
    }

    public partial struct Line
    {
        public float2 direction;
        public float2 point;
    }

    [BurstCompile]
    [WithAll(typeof(IntrovertAgent))]
    [WithNone(typeof(Dead))]
    public partial struct IntrovertRvo2Job : IJobEntity, IJobEntityChunkBeginEnd
    {
        public float timeStep;
        [ReadOnly] public ComponentLookup<LocalToWorld> localToWorldRo;
        [NativeDisableParallelForRestriction] public ComponentLookup<IntrovertAgent> rvoAgentRw;
        [ReadOnly] public ComponentLookup<BlobActorFlags> actorFlagsRo;
        [ReadOnly] public ComponentLookup<BlobActorLimits> actorLimitsRo;
        [ReadOnly] public ComponentLookup<DynamicObjectVelocity> dynamicRo;
        [ReadOnly] public ComponentLookup<BlobActor.Runtime.BlobActor> actorLookupRo;

        [NativeDisableContainerSafetyRestriction]
        private NativeList<UniformSpatialElement> neighbours;

        [NativeDisableContainerSafetyRestriction]
        private NativeList<UniformSpatialElement> obstacles;

        [NativeDisableContainerSafetyRestriction]
        private NativeList<Line> lines;

        public UniformSpatialDatabaseReadonlyBridge bridge;
        [ReadOnly] public ComponentLookup<global::Xacce.Introvert.Runtime.PolyObstacle.PolyObstacle> obstacleLineRo;

        [BurstCompile]
        public void Execute(Entity entity)
        {
            var flags = actorFlagsRo[entity];
            if ((flags.flags & BlobActorFlags.Flag.AvoidanceEnabled) == 0) return;
            var dynamicObject = dynamicRo[entity];
            var limits = actorLimitsRo[entity];
            neighbours.Clear();
            obstacles.Clear();
            lines.Clear();
            var visitor = new Visitor()
            {
                neighbors = neighbours,
                obstacles = obstacles,
            };

            var ltw = localToWorldRo[entity];
            var position2d = new float2(ltw.Position.x, ltw.Position.z);
            var rvo2Agent = rvoAgentRw[entity];

            ref var actorBlob = ref actorLookupRo[entity].blob.Value;
            var runtimeAgentVelocity = new float2(dynamicObject.velocity.x, dynamicObject.velocity.z);
            UniformSpatialDatabase.QueryAABB(bridge.database, bridge.cellsUnsafe, bridge.elementsUnsafe, ltw.Position, actorBlob.neighborsExtents, ref visitor);

            #region Obstacles

            float invTimeHorizonObst = 1.0f / actorBlob.timeHorizonObst;
            for (int i = 0; i < obstacles.Length; ++i)
            {
                var obstacleUniformElement = obstacles[i];
                var obstacle = obstacleLineRo[obstacleUniformElement.entity];

                var p1 = obstacle.p1;
                var p2 = obstacle.p2;
                var radiusObs = actorBlob.radius + obstacle.thickness;

                var relativePosition1 = p1.p - position2d;
                var relativePosition2 = p2.p - position2d;


                bool alreadyCovered = false;

                for (int j = 0; j < lines.Length; ++j)
                {
                    if (RVOMath.det(invTimeHorizonObst * relativePosition1 - lines[j].point, lines[j].direction) - invTimeHorizonObst * radiusObs >= -RVOMath.RVOEPSILON &&
                        RVOMath.det(invTimeHorizonObst * relativePosition2 - lines[j].point, lines[j].direction) - invTimeHorizonObst * radiusObs >= -RVOMath.RVOEPSILON)
                    {
                        alreadyCovered = true;

                        break;
                    }
                }

                if (alreadyCovered)
                {
                    continue;
                }

                /* Not yet covered. Check for collisions. */
                float distSq1 = RVOMath.absSq(relativePosition1);
                float distSq2 = RVOMath.absSq(relativePosition2);

                float radiusSq = RVOMath.sqr(radiusObs);
                float2 obstacleVector = p2.p - p1.p;
                float s = math.dot(-relativePosition1, obstacleVector) / RVOMath.absSq(obstacleVector);
                float distSqLine = RVOMath.absSq(-relativePosition1 - s * obstacleVector);

                Line line;

                if (s < 0.0f && distSq1 <= radiusSq)
                {
                    /* Collision with left vertex. Ignore if non-convex. */
                    if (p1.convex)
                    {
                        line.point = new float2(0.0f, 0.0f);
                        line.direction = RVOMath.normalize(new float2(-relativePosition1.y, relativePosition1.x));
                        lines.Add(line);
                    }

                    continue;
                }
                else if (s > 1.0f && distSq2 <= radiusSq)
                {
                    /*
                     * Collision with right vertex. Ignore if non-convex or if
                     * it will be taken care of by neighboring obstacle.
                     */
                    if (p2.convex && RVOMath.det(relativePosition2, p2.direction) >= 0.0f)
                    {
                        line.point = new float2(0.0f, 0.0f);
                        line.direction = RVOMath.normalize(new float2(-relativePosition2.y, relativePosition2.x));
                        lines.Add(line);
                    }

                    continue;
                }
                else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq)
                {
                    /* Collision with obstacle segment. */
                    line.point = new float2(0.0f, 0.0f);
                    line.direction = -p1.direction;
                    lines.Add(line);

                    continue;
                }

                /*
                 * No collision. Compute legs. When obliquely viewed, both legs
                 * can come from a single vertex. Legs extend cut-off line when
                 * non-convex vertex.
                 */

                float2 leftLegDirection, rightLegDirection;

                if (s < 0.0f && distSqLine <= radiusSq)
                {
                    /*
                     * Obstacle viewed obliquely so that left vertex
                     * defines velocity obstacle.
                     */
                    if (!p1.convex)
                    {
                        /* Ignore obstacle. */
                        continue;
                    }

                    p2 = p1;

                    float leg1 = RVOMath.sqrt(distSq1 - radiusSq);
                    leftLegDirection = new float2(relativePosition1.x * leg1 - relativePosition1.y * radiusObs, relativePosition1.x * radiusObs + relativePosition1.y * leg1) /
                                       distSq1;
                    rightLegDirection =
                        new float2(relativePosition1.x * leg1 + relativePosition1.y * radiusObs, -relativePosition1.x * radiusObs + relativePosition1.y * leg1) / distSq1;
                }
                else if (s > 1.0f && distSqLine <= radiusSq)
                {
                    /*
                     * Obstacle viewed obliquely so that
                     * right vertex defines velocity obstacle.
                     */
                    if (!p2.convex)
                    {
                        /* Ignore obstacle. */
                        continue;
                    }

                    p1 = p2;

                    float leg2 = RVOMath.sqrt(distSq2 - radiusSq);
                    leftLegDirection = new float2(relativePosition2.x * leg2 - relativePosition2.y * radiusObs, relativePosition2.x * radiusObs + relativePosition2.y * leg2) /
                                       distSq2;
                    rightLegDirection =
                        new float2(relativePosition2.x * leg2 + relativePosition2.y * radiusObs, -relativePosition2.x * radiusObs + relativePosition2.y * leg2) / distSq2;
                }
                else
                {
                    /* Usual situation. */
                    if (p1.convex)
                    {
                        float leg1 = RVOMath.sqrt(distSq1 - radiusSq);
                        leftLegDirection = new float2(
                            relativePosition1.x * leg1 - relativePosition1.y * radiusObs,
                            relativePosition1.x * radiusObs + relativePosition1.y * leg1) / distSq1;
                    }
                    else
                    {
                        /* Left vertex non-convex; left leg extends cut-off line. */
                        leftLegDirection = -p1.direction;
                    }

                    if (p2.convex)
                    {
                        float leg2 = RVOMath.sqrt(distSq2 - radiusSq);
                        rightLegDirection = new float2(
                            relativePosition2.x * leg2 + relativePosition2.y * radiusObs,
                            -relativePosition2.x * radiusObs + relativePosition2.y * leg2) / distSq2;
                    }
                    else
                    {
                        /* Right vertex non-convex; right leg extends cut-off line. */
                        rightLegDirection = p1.direction;
                    }
                }

                /*
                 * Legs can never point into neighboring edge when convex
                 * vertex, take cutoff-line of neighboring edge instead. If
                 * velocity projected on "foreign" leg, no constraint is added.
                 */

                // Obstacle leftNeighbor = p1.previous;

                bool isLeftLegForeign = false;
                bool isRightLegForeign = false;

                if (p1.convex && RVOMath.det(leftLegDirection, -p1.prevInvDirection) >= 0.0f)
                {
                    /* Left leg points into obstacle. */
                    leftLegDirection = -p1.prevInvDirection;
                    isLeftLegForeign = true;
                }

                if (p2.convex && RVOMath.det(rightLegDirection, p2.direction) <= 0.0f)
                {
                    /* Right leg points into obstacle. */
                    rightLegDirection = p2.direction;
                    isRightLegForeign = true;
                }

                /* Compute cut-off centers. */
                float2 leftCutOff = invTimeHorizonObst * (p1.p - position2d);
                float2 rightCutOff = invTimeHorizonObst * (p2.p - position2d);
                var cutOffVector = rightCutOff - leftCutOff;

                /* Project current velocity on velocity obstacle. */

                /* Check if current velocity is projected on cutoff circles. */
                float t = p1.index == p2.index ? 0.5f : (math.dot((runtimeAgentVelocity - leftCutOff), cutOffVector)) / RVOMath.absSq(cutOffVector);
                float tLeft = math.dot((runtimeAgentVelocity - leftCutOff), leftLegDirection);
                float tRight = math.dot((runtimeAgentVelocity - rightCutOff), rightLegDirection);

                if ((t < 0.0f && tLeft < 0.0f) || (p1.index == p2.index && tLeft < 0.0f && tRight < 0.0f))
                {
                    /* Project on left cut-off circle. */
                    float2 unitW = RVOMath.normalize(runtimeAgentVelocity - leftCutOff);

                    line.direction = new float2(unitW.y, -unitW.x);
                    line.point = leftCutOff + radiusObs * invTimeHorizonObst * unitW;
                    lines.Add(line);

                    continue;
                }
                else if (t > 1.0f && tRight < 0.0f)
                {
                    /* Project on right cut-off circle. */
                    float2 unitW = RVOMath.normalize(runtimeAgentVelocity - rightCutOff);

                    line.direction = new float2(unitW.y, -unitW.x);
                    line.point = rightCutOff + radiusObs * invTimeHorizonObst * unitW;
                    lines.Add(line);

                    continue;
                }

                /*
                 * Project on left leg, right leg, or cut-off line, whichever is
                 * closest to velocity.
                 */
                float distSqCutoff = (t < 0.0f || t > 1.0f || p2.index == p1.index) ? float.PositiveInfinity : RVOMath.absSq(runtimeAgentVelocity - (leftCutOff + t * cutOffVector));
                float distSqLeft = tLeft < 0.0f ? float.PositiveInfinity : RVOMath.absSq(runtimeAgentVelocity - (leftCutOff + tLeft * leftLegDirection));
                float distSqRight = tRight < 0.0f ? float.PositiveInfinity : RVOMath.absSq(runtimeAgentVelocity - (rightCutOff + tRight * rightLegDirection));

                if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
                {
                    /* Project on cut-off line. */
                    line.direction = -p1.direction;
                    line.point = leftCutOff + radiusObs * invTimeHorizonObst * new float2(-line.direction.y, line.direction.x);
                    lines.Add(line);

                    continue;
                }

                if (distSqLeft <= distSqRight)
                {
                    /* Project on left leg. */
                    if (isLeftLegForeign)
                    {
                        continue;
                    }

                    line.direction = leftLegDirection;
                    line.point = leftCutOff + radiusObs * invTimeHorizonObst * new float2(-line.direction.y, line.direction.x);
                    lines.Add(line);

                    continue;
                }

                /* Project on right leg. */
                if (isRightLegForeign)
                {
                    continue;
                }

                line.direction = -rightLegDirection;
                line.point = rightCutOff + radiusObs * invTimeHorizonObst * new float2(-line.direction.y, line.direction.x);
                lines.Add(line);
            }

            #endregion

            // if (lines.Length > 0) Debug.Log(lines.Length);
            int numObstLines = lines.Length;

            #region Other agents

            float invTimeHorizon = 1.0f / actorBlob.timeHorizon;
            for (int i = 0; i < neighbours.Length; ++i)
            {
                var neightbor = neighbours[i];
                if (neightbor.entity.Equals(entity)) continue;
                var otherFlags = actorFlagsRo[neightbor.entity];
                var otherDynamicObject = dynamicRo[neightbor.entity];
                var otherVelocity = new float2(otherDynamicObject.velocity.x, otherDynamicObject.velocity.z);
                ref var otherActorBlob = ref actorLookupRo[neightbor.entity].blob.Value;
                var otherLtw = localToWorldRo[neightbor.entity];

                float2 relativePosition = new float2(otherLtw.Position.x, otherLtw.Position.z) - position2d;
                float2 relativeVelocity = runtimeAgentVelocity - otherVelocity;
                float distSq = RVOMath.absSq(relativePosition);
                float combinedRadius = actorBlob.radius + otherActorBlob.radius;
                float combinedRadiusSq = RVOMath.sqr(combinedRadius);

                Line line;
                float2 u;

                if (distSq > combinedRadiusSq)
                {
                    /* No collision. */
                    float2 w = relativeVelocity - invTimeHorizon * relativePosition;

                    /* Vector from cutoff center to relative velocity. */
                    float wLengthSq = RVOMath.absSq(w);
                    float dotProduct1 = math.dot(w, relativePosition);

                    if (dotProduct1 < 0.0f && RVOMath.sqr(dotProduct1) > combinedRadiusSq * wLengthSq)
                    {
                        /* Project on cut-off circle. */
                        float wLength = RVOMath.sqrt(wLengthSq);
                        float2 unitW = w / wLength;

                        line.direction = new float2(unitW.y, -unitW.x);
                        u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                    }
                    else
                    {
                        /* Project on legs. */
                        float leg = RVOMath.sqrt(distSq - combinedRadiusSq);

                        if (RVOMath.det(relativePosition, w) > 0.0f)
                        {
                            /* Project on left leg. */
                            line.direction = new float2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg) /
                                             distSq;
                        }
                        else
                        {
                            /* Project on right leg. */
                            line.direction = -new float2(
                                relativePosition.x * leg + relativePosition.y * combinedRadius,
                                -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }

                        float dotProduct2 = math.dot(relativeVelocity, line.direction);
                        u = dotProduct2 * line.direction - relativeVelocity;
                    }
                }
                else
                {
                    /* Collision. Project on cut-off circle of time timeStep. */
                    float invTimeStep = 1.0f / actorBlob.timeStep;

                    /* Vector from cutoff center to relative velocity. */
                    float2 w = relativeVelocity - invTimeStep * relativePosition;

                    float wLength = RVOMath.abs(w);
                    float2 unitW = w / wLength;

                    line.direction = new float2(unitW.y, -unitW.x);
                    u = (combinedRadius * invTimeStep - wLength) * unitW;
                }

                line.point = runtimeAgentVelocity + 0.5f * u;
                lines.Add(line);
            }

            #endregion

            float2 newVelocity = float2.zero;

            int lineFail = RVO2Math.linearProgram2(lines, limits.maxSpeed, new float2(dynamicObject.velocity.x, dynamicObject.velocity.z), false, ref newVelocity);
            if (lineFail < lines.Length)
            {
                RVO2Math.linearProgram3(lines, numObstLines, lineFail, limits.maxSpeed, ref newVelocity);
            }

            // rvo2Agent.velocity = newVelocity; //todo research why sometimes we gen null velocity
            rvo2Agent.velocity = math.clamp(newVelocity, new float2(-100f), new float2(100f)); //todo research why sometimes we gen null velocity
            rvoAgentRw.GetRefRW(entity).ValueRW = rvo2Agent;

        }

        public bool OnChunkBegin(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
        {
            if (!neighbours.IsCreated) neighbours = new NativeList<UniformSpatialElement>(Allocator.Temp);
            if (!lines.IsCreated) lines = new NativeList<Line>(Allocator.Temp);
            if (!obstacles.IsCreated) obstacles = new NativeList<UniformSpatialElement>(Allocator.Temp);
            bridge.CreateBridge();
            return true;
        }

        public void OnChunkEnd(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask, bool chunkWasExecuted)
        {
        }
    }

}