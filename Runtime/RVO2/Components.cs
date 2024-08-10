using System;
using Unity.Entities;
using Unity.Mathematics;

namespace Introvert.RVO2
{
    public partial struct IntrovertAgent : IComponentData
    {
        public float2 velocity;
    }
}