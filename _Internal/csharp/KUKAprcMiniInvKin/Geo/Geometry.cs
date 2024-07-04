using System;
using System.Collections.Generic;
using System.Text;

namespace KUKAprcCoreMini.Geometry
{

    public static class Intersect
    {
        public static class Intersection
        {
            public static void LineSphere(Line line, Sphere sphere, out Point3d? pt1, out Point3d? pt2)
            {
                Vector3d point2origin = line.To - line.From;
                Vector3d direction = sphere.center - line.From;

                double magnitude = point2origin.SquareLength;
                double product = Vector3d.Dot(point2origin, direction);
                double tca = product / magnitude * point2origin.Length;

                double d2 = Vector3d.Dot(direction, direction) - tca * tca;

                if (Math.Sqrt(d2) > sphere.radius)
                {
                    pt1 = null;
                    pt2 = null;
                }
                else
                {
                    double thc = Math.Sqrt(sphere.radius * sphere.radius - d2);

                    double pt1val = tca - thc;
                    double pt2val = tca + thc;

                    point2origin = point2origin.Unitize();

                    pt1 = line.From + point2origin * pt1val;
                    pt2 = line.From + point2origin * pt2val;
                }
            }
        }
    }
    
}
