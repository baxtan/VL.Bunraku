using System;
using Stride.Core.Mathematics;

namespace SolverCoreMini.Geometry
{
    /// <summary>
    /// Represents a coordinate frame defined by an origin and three orthonormal axes.
    /// (This is analogous to your previous “Plane” class but implemented using Stride’s math library.)
    /// </summary>
    public class Frame
    {
        public Vector3 Origin;
        public Vector3 XAxis;
        public Vector3 YAxis;
        public Vector3 ZAxis;

        public Frame(Vector3 origin, Vector3 xAxis, Vector3 yAxis)
        {
            Origin = origin;
            XAxis = xAxis;
            YAxis = yAxis;
            // Compute the third axis as the cross product.
            ZAxis = Vector3.Normalize(Vector3.Cross(xAxis, yAxis));
        }

        /// <summary>
        /// Applies a transformation matrix to this frame.
        /// </summary>
        public void Transform(ref Matrix transform)
        {
            Origin = Vector3.TransformCoordinate(Origin, transform);
            XAxis = Vector3.Normalize(Vector3.TransformNormal(XAxis, transform));
            YAxis = Vector3.Normalize(Vector3.TransformNormal(YAxis, transform));
            ZAxis = Vector3.Normalize(Vector3.TransformNormal(ZAxis, transform));
        }
    }

    /// <summary>
    /// Provides transformation utility methods based on Stride’s Matrix functions.
    /// </summary>
    public static class TransformUtil
    {
        /// <summary>
        /// Returns a transformation matrix representing a rotation about a given axis through a specified origin.
        /// </summary>
        public static Matrix Rotation(float angle, Vector3 axis, Vector3 origin)
        {
            // Create a rotation matrix around the given axis.
            Matrix rotation = Matrix.RotationAxis(axis, angle);
            // Create translation matrices to move to and from the origin.
            Matrix translationToOrigin = Matrix.Translation(-origin);
            Matrix translationBack = Matrix.Translation(origin);
            return translationToOrigin * rotation * translationBack;
        }

        /// <summary>
        /// Returns a translation transformation matrix.
        /// </summary>
        public static Matrix Translation(Vector3 translation)
        {
            return Matrix.Translation(translation);
        }

        /// <summary>
        /// Computes a transformation matrix that changes basis from one frame to another.
        /// (For simplicity, this sample computes a basic basis change.)
        /// </summary>
        public static Matrix ChangeBasis(Frame from, Frame to)
        {
            // Construct matrices representing the coordinate systems.
            Matrix fromMatrix = new Matrix(
                from.XAxis.X, from.XAxis.Y, from.XAxis.Z, 0,
                from.YAxis.X, from.YAxis.Y, from.YAxis.Z, 0,
                from.ZAxis.X, from.ZAxis.Y, from.ZAxis.Z, 0,
                from.Origin.X, from.Origin.Y, from.Origin.Z, 1);

            Matrix toMatrix = new Matrix(
                to.XAxis.X, to.XAxis.Y, to.XAxis.Z, 0,
                to.YAxis.X, to.YAxis.Y, to.YAxis.Z, 0,
                to.ZAxis.X, to.ZAxis.Y, to.ZAxis.Z, 0,
                to.Origin.X, to.Origin.Y, to.Origin.Z, 1);

            Matrix inverseFrom;
            Matrix.Invert(ref fromMatrix, out inverseFrom);
            return inverseFrom * toMatrix;
        }

        /// <summary>
        /// Computes a transformation matrix that maps one frame to another.
        /// For our purposes, this is equivalent to ChangeBasis.
        /// </summary>
        public static Matrix PlaneToPlane(Frame from, Frame to)
        {
            return ChangeBasis(from, to);
        }
    }

    /// <summary>
    /// Represents a line defined by two points.
    /// </summary>
    public class Line
    {
        public Vector3 From;
        public Vector3 To;

        public Line(Vector3 from, Vector3 to)
        {
            From = from;
            To = to;
        }

        public Vector3 Direction => Vector3.Normalize(To - From);

        /// <summary>
        /// Applies a transformation matrix to the line.
        /// </summary>
        public void Transform(ref Matrix transform)
        {
            From = Vector3.TransformCoordinate(From, transform);
            To = Vector3.TransformCoordinate(To, transform);
        }
    }
}
