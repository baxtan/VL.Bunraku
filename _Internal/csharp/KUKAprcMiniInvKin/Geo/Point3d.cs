using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Xml.Linq;
using g = KUKAprcCoreMini.PRC_MathSharpDX;

namespace KUKAprcCoreMini.Geometry
{
    public struct Point3d
    {
        /// <summary>
        /// A point with (0,0) coordinates.
        /// </summary>
        public static readonly Point3d Zero = new Point3d(0, 0, 0);

        /// <summary>
        /// A point with (0,0) coordinates.
        /// </summary>
        public static readonly Point3d Origin = new Point3d(0, 0, 0);

        //public Point3d()
        //{
        //    xval = 0;
        //    yval = 0;
        //    zval = 0;
        //    ptmatrix = new g.Matrix();
        //    ptmatrix.Row1 = new g.Vector4(1, 0, 0, 0);
        //    ptmatrix.Row2 = new g.Vector4(0, 1, 0, 0);
        //    ptmatrix.Row3 = new g.Vector4(0, 0, 1, 0);
        //    ptmatrix.Row4 = new g.Vector4(X, Y, Z, 1);
        //}

        /// <summary>
        /// Initializes a new instance of the <see cref="Point"/> struct.
        /// </summary>
        /// <param name="X">The x.</param>
        /// <param name="Y">The y.</param>
        
        public Point3d(float X, float Y, float Z)
        {
            ptint = new g.Vector3(X, Y, Z);
            
        }

        
        public Point3d(double X, double Y, double Z)
        {
            ptint = new g.Vector3((float)X,(float)Y, (float)Z);
        }

        public Point3d(Point3d pt)
        {
            ptint = new g.Vector3(pt.X, pt.Y, pt.Z);
        }

        public Point3d(Vector3d pt)
        {
            ptint = new g.Vector3(pt.X, pt.Y, pt.Z);
        }

        public Point3d(g.Vector3 pt)
        {
            ptint = pt;
        }

        public Point3d(g.Matrix matrix)
        {
            ptint = new g.Vector3(matrix.Row4.X, matrix.Row4.Y, matrix.Row4.Z);
        }

        //private float xval;
        //private float yval;
        //private float zval;

        /// <summary>
        /// Left coordinate.
        /// </summary>
        public float X
        {
            get
            {
                return this.ptint.X;
            }
            set
            {
                this.ptint.X = value;
            }
        }

        /// <summary>
        /// Top coordinate.
        /// </summary>
        public float Y
        {
            get
            {
                return this.ptint.Y;
            }
            set
            {
                this.ptint.Y = value;
            }
        }


        /// <summary>
        /// Top coordinate.
        /// </summary>
        public float Z
        {
            get
            {
                return this.ptint.Z;
            }
            set
            {
                this.ptint.Z = value;
            }
        }

        //[Newtonsoft.Json.JsonIgnoreAttribute]
        //private g.Matrix ptmatrix;


        private g.Vector3 ptint;

        /// <summary>
        /// Determines whether the specified <see cref="System.Object"/> is equal to this instance.
        /// </summary>
        /// <param name="other">The <see cref="System.Object"/> to compare with this instance.</param>
        /// <returns>
        ///   <c>true</c> if the specified <see cref="System.Object"/> is equal to this instance; otherwise, <c>false</c>.
        /// </returns>
        public bool Equals(Point3d other)
        {
            return other.X == X && other.Y == Y && other.Z == Z;
        }

        /// <inheritdoc/>
        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj)) return false;
            if (obj.GetType() != typeof(Point3d)) return false;
            return Equals((Point3d)obj);
        }

        /// <inheritdoc/>
        public override int GetHashCode()
        {
            unchecked
            {
                return Convert.ToInt32((X * 395) + Y + Z);
            }
        }

        public Point3d Transform(g.Matrix matrix)
        {
            g.Vector4 pt = new g.Vector4(
                           (ptint.X * matrix.M11) + (ptint.Y * matrix.M21) + (ptint.Z * matrix.M31) + matrix.M41,
                           (ptint.X * matrix.M12) + (ptint.Y * matrix.M22) + (ptint.Z * matrix.M32) + matrix.M42,
                           (ptint.X * matrix.M13) + (ptint.Y * matrix.M23) + (ptint.Z * matrix.M33) + matrix.M43,
                           (ptint.X * matrix.M14) + (ptint.Y * matrix.M24) + (ptint.Z * matrix.M34) + matrix.M44);
            return new Point3d(pt.X, pt.Y, pt.Z);
            
        }

        public override string ToString()
        {
            return string.Format("({0},{1},{2})", X, Y, Z);
        }

        public static Vector3d Subtract(Point3d left, Point3d right)
        {
            return new Vector3d(left.X - right.X, left.Y - right.Y, left.Z - right.Z);
        }

        public double DistanceTo(Point3d pt)
        {
            Vector3d distance = Vector3d.Subtract(pt, new Point3d(this.X, this.Y, this.Z));
            return Math.Sqrt(Math.Pow(distance.X, 2) + Math.Pow(distance.Y, 2) + Math.Pow(distance.Z, 2));
        }

        public double SqrtDistanceTo(Point3d pt)
        {
            Vector3d distance = Vector3d.Subtract(pt, new Point3d(this.X, this.Y, this.Z));
            return Math.Pow(distance.X, 2) + Math.Pow(distance.Y, 2) + Math.Pow(distance.Z, 2);
        }

        public int ClosestPoint(List<Point3d> pts)
        {
            double bestval = this.SqrtDistanceTo(pts[0]);
            int bestint = 0;
            for (int i = 1; i < pts.Count; i++)
            {
                double dist = this.SqrtDistanceTo(pts[i]);
                if (dist < bestval)
                {
                    bestint = i;
                    bestval = dist;
                }
            }
            return bestint;
        }

        public XElement ToXML()
        {
            XElement vector =
               new XElement("point",
                    new XAttribute("X", ptint.X),
                    new XAttribute("Y", ptint.Y),
                    new XAttribute("Z", ptint.Z)
           );
            return vector;
        }

        /// <summary>
        /// Performs an explicit conversion from <see cref="Vector2"/> to <see cref="Point"/>.
        /// </summary>
        /// <param name="value">The value.</param>
        /// <returns>The result of the conversion.</returns>
        public static implicit operator Point3d(g.Vector3 value)
        {
            return new Point3d(value); //(value.X, value.Y, value.Z);
        }

        public static implicit operator g.Vector3(Point3d value)
        {
            return new g.Vector3(value.X, value.Y, value.Z);
        }


        public static implicit operator Point3d(Vector3d value)
        {
            return new Point3d(value.X, value.Y, value.Z);
        }

        public static implicit operator Point3d(g.Matrix value)
        {
            return new Point3d(value.Row4.X, value.Row4.Y, value.Row4.Z);
        }

        public static Vector3d operator -(Point3d left, Point3d right)
        {
            return new Vector3d(left.X - right.X, left.Y - right.Y, left.Z - right.Z);
        }

        public static Vector3d operator +(Point3d left, Point3d right)
        {
            return new Vector3d(left.X + right.X, left.Y + right.Y, left.Z + right.Z);
        }

        public static Point3d operator *(Point3d left, double right)
        {
            return new Point3d(left.X * right, left.Y * right, left.Z * right);
        }

        public static Point3d operator *(Point3d left, float right)
        {
            return new Point3d(left.X * right, left.Y * right, left.Z * right);
        }

        public static Point3d operator *(float right, Point3d left)
        {
            return new Point3d(left.X * right, left.Y * right, left.Z * right);
        }

        /// <summary>
        /// Performs an implicit conversion from <see cref="Point"/> to <see cref="Vector2"/>.
        /// </summary>
        /// <param name="value">The value.</param>
        /// <returns>The result of the conversion.</returns>
        public static implicit operator g.Matrix(Point3d value)
        {
            g.Matrix ptmatrix = new g.Matrix();
            ptmatrix.Row1 = new g.Vector4(1, 0, 0, 0);
            ptmatrix.Row2 = new g.Vector4(0, 1, 0, 0);
            ptmatrix.Row3 = new g.Vector4(0, 0, 1, 0);
            ptmatrix.Row4 = new g.Vector4(value.X, value.Y, value.Z, 1);

            return ptmatrix;
        }


    }
}
