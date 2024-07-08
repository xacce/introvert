using Unity.Entities;
using Unity.Mathematics;

namespace Introvert.PolyObstacle
{
    public partial struct PolyObstacle : IComponentData
    {
        public struct Point
        {
            public bool convex;
            public float2 p;
            public float2 direction;
            public float2 prevInvDirection;
            public byte index;
        }


        public Point p1;
        public Point p2;
        public float thickness;
        public float2 direction;
    }

}