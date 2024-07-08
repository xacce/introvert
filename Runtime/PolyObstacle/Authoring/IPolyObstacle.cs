using System.Collections.Generic;
using Unity.Entities;
using UnityEngine;

namespace Introvert.PolyObstacle.Authoring
{
    public interface IPolyObstacle
    {
        public void GetWrapped(Vector3 position,IBaker baker);
    }
}