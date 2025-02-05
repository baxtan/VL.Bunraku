// For examples, see:
// https://thegraybook.vvvv.org/reference/extending/writing-nodes.html#examples

using System;

namespace Bunraku.Geometry
{

    using Bunraku.Geometry;

    public static class TransformExtensions
    {
        /// <summary>
        /// Computes a transformation that maps the 'from' plane onto the 'to' plane.
        /// </summary>
        /// <param name="from">The source plane.</param>
        /// <param name="to">The target plane.</param>
        /// <returns>A Transform that maps the coordinate system of 'from' to 'to'.</returns>
        public static Transform PlaneToPlane(Plane from, Plane to)
        {
            // Construct a 4x4 matrix for the 'from' plane.
            // It is assumed that the Plane class provides properties:
            // Origin, XAxis, YAxis, and ZAxis.
            Matrix4 fromMatrix = new Matrix4(
                from.XAxis.X, from.YAxis.X, from.ZAxis.X, from.Origin.X,
                from.XAxis.Y, from.YAxis.Y, from.ZAxis.Y, from.Origin.Y,
                from.XAxis.Z, from.YAxis.Z, from.ZAxis.Z, from.Origin.Z,
                0, 0, 0, 1
            );

            // Construct a 4x4 matrix for the 'to' plane.
            Matrix4 toMatrix = new Matrix4(
                to.XAxis.X, to.YAxis.X, to.ZAxis.X, to.Origin.X,
                to.XAxis.Y, to.YAxis.Y, to.ZAxis.Y, to.Origin.Y,
                to.XAxis.Z, to.YAxis.Z, to.ZAxis.Z, to.Origin.Z,
                0, 0, 0, 1
            );

            // Compute the inverse of the 'from' matrix.
            Matrix4 invFromMatrix = Matrix4.Invert(fromMatrix);

            // Compute the transformation matrix that maps from 'from' to 'to'.
            Matrix4 resultMatrix = toMatrix * invFromMatrix;

            return new Transform(resultMatrix);
        }
    }

    public class Point3d
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }

        public Point3d(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public static Point3d operator +(Point3d p, Vector3d v)
        {
            return new Point3d(p.X + v.X, p.Y + v.Y, p.Z + v.Z);
        }

        public static Point3d operator -(Point3d p, Vector3d v)
        {
            return new Point3d(p.X - v.X, p.Y - v.Y, p.Z - v.Z);
        }
    }

    public class Vector3d
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }

        public Vector3d(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public double Dot(Vector3d v)
        {
            return X * v.X + Y * v.Y + Z * v.Z;
        }

        public Vector3d Cross(Vector3d v)
        {
            return new Vector3d(
                Y * v.Z - Z * v.Y,
                Z * v.X - X * v.Z,
                X * v.Y - Y * v.X
            );
        }

        public double Magnitude => Math.Sqrt(X * X + Y * Y + Z * Z);

        public Vector3d Normalized()
        {
            double magnitude = Magnitude;
            if (magnitude < 1e-6)
                throw new InvalidOperationException("Cannot normalize a zero-length vector.");
            return new Vector3d(X / magnitude, Y / magnitude, Z / magnitude);
        }

        public static Vector3d operator +(Vector3d v1, Vector3d v2)
        {
            return new Vector3d(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z);
        }

        public static Vector3d operator -(Vector3d v1, Vector3d v2)
        {
            return new Vector3d(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z);
        }

        public static Vector3d operator *(Vector3d v, double scalar)
        {
            return new Vector3d(v.X * scalar, v.Y * scalar, v.Z * scalar);
        }
    }

    public class Plane
    {
        public Point3d Origin { get; set; }
        public Vector3d XAxis { get; set; }
        public Vector3d YAxis { get; set; }
        public Vector3d ZAxis => XAxis.Cross(YAxis).Normalized();

        public Plane(Point3d origin, Vector3d xAxis, Vector3d yAxis)
        {
            Origin = origin;
            XAxis = xAxis.Normalized();
            YAxis = yAxis.Normalized();
            if (XAxis.Cross(YAxis).Magnitude < 1e-6)
                throw new InvalidOperationException("Invalid plane: axes are parallel or degenerate.");
        }

        public bool IsValid(double tolerance = 1e-6)
        {
            return XAxis.Cross(YAxis).Magnitude > tolerance;
        }

        public void Transform(Transform t)
        {
            Origin = t.Apply(Origin);
            XAxis = t.Apply(XAxis).Normalized();
            YAxis = t.Apply(YAxis).Normalized();
        }
    }

    public class Transform
    {
        public static Transform Rotation(double angle, Vector3d axis, Point3d origin)
        {
            // Placeholder logic for rotation
            return new Transform();
        }

        public static Transform Translation(Vector3d translation)
        {
            // Placeholder logic for translation
            return new Transform();
        }

        public static Transform operator *(Transform t1, Transform t2)
        {
            // Placeholder for combining two transformations
            return new Transform();
        }

        public Point3d Apply(Point3d point)
        {
            // Placeholder logic for applying transformation to a point
            return point;
        }

        public Vector3d Apply(Vector3d vector)
        {
            // Placeholder logic for applying transformation to a vector
            return vector;
        }
    }
}
