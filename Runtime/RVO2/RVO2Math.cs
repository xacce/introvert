using System;
using System.Runtime.CompilerServices;
using Introvert.RVO2;
using RVO;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using Xacce.Introvert.Runtime.RVO2;

namespace Pathfinding
{
    [BurstCompile]
    public static class RVO2Math
    {
        internal const float RVOEPSILON = 0.00001f;

        [BurstCompile]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float det(in float2 vector1, in float2 vector2)
        {
            return vector1.x * vector2.y - vector1.y * vector2.x;
        }

        [BurstCompile]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float abs(in float2 vector)
        {
            return math.sqrt(absSq(vector));
        }

        [BurstCompile]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float absSq(in float2 vector)
        {
            return math.dot(vector, vector);
        }
        [BurstCompile]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float sqr(float scalar)
        {
            return scalar * scalar;
        }
        [BurstCompile]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void normalize(in float2 vector, out float2 normalized)
        {
            normalized = vector / abs(vector);
        }

      
        /**
         * <summary>Solves a one-dimensional linear program on a specified line
         * subject to linear constraints defined by lines and a circular
         * constraint.</summary>
         *
         * <returns>True if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="lineNo">The specified line constraint.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        [BurstCompile]
        public static void linearProgram1(in NativeList<Line> lines, int lineNo, float radius, in float2 optVelocity, bool directionOpt, ref float2 result, out bool success)
        {
            success = false;
            float dotProduct = math.dot(lines[lineNo].point, lines[lineNo].direction);
            float discriminant = RVOMath.sqr(dotProduct) + RVOMath.sqr(radius) - RVOMath.absSq(lines[lineNo].point);

            if (discriminant < 0.0f)
            {
                /* Max speed circle fully invalidates line lineNo. */
                return;
            }

            float sqrtDiscriminant = RVOMath.sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;
            float tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < lineNo; ++i)
            {
                float denominator = RVOMath.det(lines[lineNo].direction, lines[i].direction);
                float numerator = RVOMath.det(lines[i].direction, lines[lineNo].point - lines[i].point);

                if (RVOMath.fabs(denominator) <= RVOMath.RVOEPSILON)
                {
                    /* Lines lineNo and i are (almost) parallel. */
                    if (numerator < 0.0f)
                    {

                        return;
                    }

                    continue;
                }

                float t = numerator / denominator;

                if (denominator >= 0.0f)
                {
                    /* Line i bounds line lineNo on the right. */
                    tRight = Math.Min(tRight, t);
                }
                else
                {
                    /* Line i bounds line lineNo on the left. */
                    tLeft = Math.Max(tLeft, t);
                }

                if (tLeft > tRight)
                {
                    return;
                }
            }

            if (directionOpt)
            {
                /* Optimize direction. */
                if (math.dot(optVelocity, lines[lineNo].direction) > 0.0f)
                {
                    /* Take right extreme. */
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    /* Take left extreme. */
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
            }
            else
            {
                /* Optimize closest point. */
                float t = math.dot(lines[lineNo].direction, (optVelocity - lines[lineNo].point));

                if (t < tLeft)
                {
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
                else if (t > tRight)
                {
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    result = lines[lineNo].point + t * lines[lineNo].direction;
                }
            }

            success = true;
        }

        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <returns>The number of the line it fails on, and the number of lines
         * if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        [BurstCompile]
        public static int linearProgram2(in NativeList<Line> lines, float radius, in  float2 optVelocity, bool directionOpt, ref float2 result)
        {
            if (directionOpt)
            {
                /*
                 * Optimize direction. Note that the optimization velocity is of
                 * unit length in this case.
                 */
                result = optVelocity * radius;
            }
            else if (RVOMath.absSq(optVelocity) > RVOMath.sqr(radius))
            {
                /* Optimize closest point and outside circle. */
                result = RVOMath.normalize(optVelocity) * radius;
            }
            else
            {
                /* Optimize closest point and inside circle. */
                result = optVelocity;
            }

            for (int i = 0; i < lines.Length; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > 0.0f)
                {
                    /* Result does not satisfy constraint i. Compute new optimal result. */
                    float2 tempResult = result;
                    linearProgram1(lines, i, radius, optVelocity, directionOpt, ref result, out var success);
                    if (!success)
                    {
                        result = tempResult;

                        return i;
                    }
                }
            }

            return lines.Length;
        }

        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="numObstLines">Count of obstacle lines.</param>
         * <param name="beginLine">The line on which the 2-d linear program
         * failed.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        [BurstCompile]
        public static void linearProgram3(in NativeList<Line> lines, int numObstLines, int beginLine, float radius, ref float2 result)
        {
            float distance = 0.0f;
            NativeList<Line> projLines = new NativeList<Line>(Allocator.Temp);
            for (int i = beginLine; i < lines.Length; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > distance)
                {
                    /* Result does not satisfy constraint of line i. */
                    projLines.Clear();
                    for (int ii = 0; ii < numObstLines; ++ii)
                    {
                        projLines.Add(lines[ii]);
                    }

                    for (int j = numObstLines; j < i; ++j)
                    {
                        Line line;

                        float determinant = RVOMath.det(lines[i].direction, lines[j].direction);

                        if (RVOMath.fabs(determinant) <= RVOMath.RVOEPSILON)
                        {
                            /* Line i and line j are parallel. */
                            if (math.dot(lines[i].direction, lines[j].direction) > 0.0f)
                            {
                                /* Line i and line j point in the same direction. */
                                continue;
                            }
                            else
                            {
                                /* Line i and line j point in opposite direction. */
                                line.point = 0.5f * (lines[i].point + lines[j].point);
                            }
                        }
                        else
                        {
                            line.point = lines[i].point + (RVOMath.det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
                        }

                        line.direction = RVOMath.normalize(lines[j].direction - lines[i].direction);
                        projLines.Add(line);
                    }

                    float2 tempResult = result;
                    if (linearProgram2(projLines, radius, new float2(-lines[i].direction.y, lines[i].direction.x), true, ref result) < projLines.Length)
                    {
                        /*
                         * This should in principle not happen. The result is by
                         * definition already in the feasible region of this
                         * linear program. If it fails, it is due to small
                         * floating point error, and the current result is kept.
                         */
                        result = tempResult;
                    }

                    distance = RVOMath.det(lines[i].direction, lines[i].point - result);
                }
            }
        }
    }
}