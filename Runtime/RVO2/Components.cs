using System;
using Unity.Entities;
using Unity.Mathematics;

namespace Introvert.RVO2
{
    public partial struct IntrovertRvoAgent : IComponentData
    {
        public float2 velocity;
    }
}