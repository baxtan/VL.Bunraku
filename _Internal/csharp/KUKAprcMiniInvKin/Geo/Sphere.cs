using System;
using System.Collections.Generic;
using System.Text;

namespace KUKAprcCoreMini.Geometry
{
    public class Sphere
    {
        public Sphere(Point3d center, double radius)
        {
            this.center = center;
            this.radius = radius;
        }

        public Sphere()
        { }

        public Point3d center;
        public double radius;
    }
}
