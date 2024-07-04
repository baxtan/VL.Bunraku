using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Xml.Linq;
using g = KUKAprcCoreMini.PRC_MathSharpDX;

namespace KUKAprcCoreMini.Geometry
{
    public struct Vector3d
    {
        //private Vector3d()
        //{
        //    X = 0;
        //    Y = 0;
        //    Z = 0;
        //    ptmatrix = new g.Matrix();
        //    ptmatrix.Row1 = new g.Vector4(0, 0, 0, 0);
        //    ptmatrix.Row2 = new g.Vector4(0, 0, 0, 0);
        //    ptmatrix.Row3 = new g.Vector4(0, 0, 0, 0);
        //    ptmatrix.Row4 = new g.Vector4(X, Y, Z, 1);
        //}

        /// <summary>
        /// A point with (0,0) coordinates.
        /// </summary>
        public static readonly Vector3d Zero = new Vector3d(0, 0, 0);

        /// <summary>
        /// A point with (0,0) coordinates.
        /// </summary>
        public static readonly Vector3d XAxis = new Vector3d(1, 0, 0);

        /// <summary>
        /// A point with (0,0) coordinates.
        /// </summary>
        public static readonly Vector3d YAxis = new Vector3d(0, 1, 0);

        /// <summary>
        /// A point with (0,0) coordinates.
        /// </summary>
        public static readonly Vector3d ZAxis = new Vector3d(0, 0, 1);

        /// <summary>
        /// Initializes a new instance of the <see cref="Point"/> struct.
        /// </summary>
        /// <param name="X">The x.</param>
        /// <param name="Y">The y.</param>
        
        public Vector3d(float X, float Y, float Z)
        {
            vecint = new g.Vector3(X, Y, Z);
            //xval = X;
            //yval = Y;
            //zval = Z;
            //ptmatrix = new g.Matrix();
            //ptmatrix.Row1 = new g.Vector4(1, 0, 0, 0);
            //ptmatrix.Row2 = new g.Vector4(0, 1, 0, 0);
            //ptmatrix.Row3 = new g.Vector4(0, 0, 1, 0);
            //ptmatrix.Row4 = new g.Vector4(X, Y, Z, 1);
        }

        
        public Vector3d(double X, double Y, double Z)
        {
            vecint = new g.Vector3((float)X, (float)Y, (float)Z);

            //xval = (float)X;
            //yval = (float)Y;
            //zval = (float)Z;
            //ptmatrix = new g.Matrix();
            //ptmatrix.Row1 = new g.Vector4(1, 0, 0, 0);
            //ptmatrix.Row2 = new g.Vector4(0, 1, 0, 0);
            //ptmatrix.Row3 = new g.Vector4(0, 0, 1, 0);
            //ptmatrix.Row4 = new g.Vector4((float)X, (float)Y, (float)Z, 1);
        }

        public Vector3d(g.Matrix matrix)
        {
            vecint = new g.Vector3(matrix.Row4.X, matrix.Row4.Y, matrix.Row4.Z);
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
                return this.vecint.X;
            }
            set
            {
                this.vecint.X = value;
            }
        }

        /// <summary>
        /// Top coordinate.
        /// </summary>
        public float Y
        {
            get
            {
                return this.vecint.Y;
            }
            set
            {
                this.vecint.Y = value;
            }
        }


        /// <summary>
        /// Top coordinate.
        /// </summary>
        public float Z
        {
            get
            {
                return this.vecint.Z;
            }
            set
            {
                this.vecint.Z = value;
            }
        }


        //
        //private g.Matrix ptmatrix;

        
        private g.Vector3 vecint;

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
                return Convert.ToInt32((X * 397) + Y + Z);
            }
        }

        public Vector3d Transform(g.Matrix matrix)
        {
            //g.Matrix mout = g.Matrix.Multiply(ptmatrix, matrix);
            //ptmatrix = mout;
            //X = mout.TranslationVector.X;
            //Y = mout.TranslationVector.Y;
            //Z = mout.TranslationVector.Z;

            vecint = (g.Vector3)new g.Vector4(
               (vecint.X * matrix.M11) + (vecint.Y * matrix.M21) + (vecint.Z * matrix.M31) + matrix.M41,
               (vecint.X * matrix.M12) + (vecint.Y * matrix.M22) + (vecint.Z * matrix.M32) + matrix.M42,
               (vecint.X * matrix.M13) + (vecint.Y * matrix.M23) + (vecint.Z * matrix.M33) + matrix.M43,
               (vecint.X * matrix.M14) + (vecint.Y * matrix.M24) + (vecint.Z * matrix.M34) + matrix.M44);
            return new Vector3d(vecint.X, vecint.Y, vecint.Z);
        }

        public override string ToString()
        {
            return string.Format("({0},{1},{2})", X, Y, Z);
        }

        public static double VectorAngle (Vector3d x, Vector3d y, Plane pln)
        {
            Vector3d Va = new Vector3d(x.X, x.Y, x.Z);
            Vector3d Vb = new Vector3d(y.X, y.Y, y.Z);
            Vector3d Vn = pln.ZAxis;

            Va = Va.Unitize();
            Vb = Vb.Unitize();
            Vn = Vn.Unitize();

            //double angle = Math.Acos(Vector3d.Dot(Va, Vb));
            //Vector3d cross = Vector3d.CrossProduct(Va, Vb);

            //double tmp = Vector3d.Dot(Vn, cross.Unitize());

            double angle = Math.Atan2(Vector3d.Dot(Vector3d.CrossProduct(Vb, Va), Vn), Vector3d.Dot(Va, Vb));

            //if (Vector3d.Dot(Vn, cross) < 0)
            //{ // Or > 0
            //    angle = -angle;
            //}

            if (double.IsNaN(angle))
            {
                angle = 0;
            }
            return -angle;










            //Plane refpln = new Plane(Point3d.Origin, pln.XAxis, pln.YAxis);

            //KUKAprcCore.Geometry.Transform torigin = KUKAprcCore.Geometry.Transform.PlaneToPlane(refpln, Plane.WorldXY);
            //Vector3d vec1 = new Vector3d(x.X, x.Y, x.Z);
            //Vector3d vec2 = new Vector3d(y.X, y.Y, y.Z);

            //vec1.Transform(torigin);
            //vec2.Transform(torigin);
            //vec1.Z = 0;
            //vec2.Z = 0;

            //vec1.Unitize();
            //vec2.Unitize();

            //double ang = Math.Acos(Vector3d.Dot(vec1, vec2) / (vec1.Length / vec2.Length));

            //if (double.IsNaN(ang))
            //{
            //    ang = 0;
            //}

            //return ang;
        }

        public static double VectorAngle(Vector3d x, Vector3d y)
        {
            if (x.IsParallelTo(y) == 1)
            {
                return 0.0;
            }
            else
            {
                Plane pln = new Plane(Point3d.Origin, x, y);
                KUKAprcCoreMini.Geometry.Transform torigin = KUKAprcCoreMini.Geometry.Transform.PlaneToPlane(pln, Plane.WorldXY);
                Vector3d vec1 = new Vector3d(x.X, x.Y, x.Z);
                Vector3d vec2 = new Vector3d(y.X, y.Y, y.Z);

                vec1 = vec1.Transform(torigin);
                vec2 = vec2.Transform(torigin);
                vec1.Z = 0;
                vec2.Z = 0;

                vec1 = vec1.Unitize();
                vec2 = vec2.Unitize();

                return Math.Acos(Vector3d.Dot(vec1, vec2));
            }
        }

        public int IsParallelTo(Vector3d vec)
        {
            Vector3d cross = CrossProduct(vecint, vec);
            if (Math.Abs(cross.X + cross.Y + cross.Z) < 0.00001)
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }

        public Vector3d Unitize ()
        {
            g.Vector3 vec = new g.Vector3(vecint.X, vecint.Y, vecint.Z);
            vec.Normalize();
            return vec;
        }

        public Vector3d Reverse()
        {
            g.Vector3 vec = new g.Vector3(vecint.X * (-1.0f), vecint.Y * (-1.0f), vecint.Z * (-1.0f));

            return vec;
        }

        public Vector3d RoundZero()
        {
            double treshold = 0.00001;
            if (vecint.X < treshold && vecint.X > treshold * -1.0)
            {
                vecint.X = 0;
            }
            if (vecint.Y < treshold && vecint.Y > treshold * -1.0)
            {
                vecint.Y = 0;
            }
            if (vecint.Z < treshold && vecint.Z > treshold * -1.0)
            {
                vecint.Z = 0;
            }

            return new g.Vector3(vecint.X, vecint.Y, vecint.Z); ;
        }

        public XElement ToXML()
        {
            XElement vector =
               new XElement("vector",
                    new XAttribute("X", vecint.X),
                    new XAttribute("Y", vecint.Y),
                    new XAttribute("Z", vecint.Z)
           );
            return vector;
        }

        
        public float Length
        {
            get
            {
                return (float)Math.Sqrt((X * X) + (Y * Y) + (Z * Z));
                //return ptmatrix.Row4.Length();
            }
        }

        
        public float SquareLength
        {
            get
            {
                return vecint.LengthSquared();
            }
        }

        public static float Dot (Vector3d vec1, Vector3d vec2)
        {
            return g.Vector3.Dot(vec1, vec2);
        }

        public static Vector3d CrossProduct(Vector3d vec1, Vector3d vec2)
        {
            return g.Vector3.Cross(vec1, vec2);
        }

        public static Vector3d Subtract(Vector3d left, Vector3d right)
        {
            return new Vector3d(left.X - right.X, left.Y - right.Y, left.Z - right.Z);
        }

        /// <summary>
        /// Performs an explicit conversion from <see cref="Vector2"/> to <see cref="Point"/>.
        /// </summary>
        /// <param name="value">The value.</param>
        /// <returns>The result of the conversion.</returns>
        public static implicit operator Vector3d(g.Vector3 value)
        {
            return new Vector3d(value.X, value.Y, value.Z);
        }

        public static implicit operator g.Vector3(Vector3d value)
        {
            return new g.Vector3(value.X, value.Y, value.Z);
        }

        public static implicit operator Vector3d(Point3d value)
        {
            return new Vector3d(value.X, value.Y, value.Z);
        }

        /// <summary>
        /// Subtracts two vectors.
        /// </summary>
        /// <param name="left">The first vector to subtract.</param>
        /// <param name="right">The second vector to subtract.</param>
        /// <returns>The difference of the two vectors.</returns>
        public static Vector3d operator -(Vector3d left, Vector3d right)
        {
            return new Vector3d(left.X - right.X, left.Y - right.Y, left.Z - right.Z);
        }

        /// <summary>
        /// Adds two vectors.
        /// </summary>
        /// <param name="left">The first vector to add.</param>
        /// <param name="right">The second vector to add.</param>
        /// <returns>The addition of the two vectors.</returns>
        public static Vector3d operator +(Vector3d left, Vector3d right)
        {
            return new Vector3d(left.X + right.X, left.Y + right.Y, left.Z + right.Z);
        }

        /// <summary>
        /// Adds two vectors.
        /// </summary>
        /// <param name="left">The first vector to add.</param>
        /// <param name="right">The second vector to add.</param>
        /// <returns>The addition of the two vectors.</returns>
        public static Vector3d operator +(Point3d left, Vector3d right)
        {
            return new Vector3d(left.X + right.X, left.Y + right.Y, left.Z + right.Z);
        }

        /// <summary>
        /// Scales a vector by the given value.
        /// </summary>
        /// <param name="value">The vector to scale.</param>
        /// <param name="scale">The amount by which to scale the vector.</param>
        /// <returns>The scaled vector.</returns>
        public static Vector3d operator *(double scale, Vector3d value)
        {
            return new Vector3d(value.X * scale, value.Y * scale, value.Z * scale);
        }

        /// <summary>
        /// Scales a vector by the given value.
        /// </summary>
        /// <param name="value">The vector to scale.</param>
        /// <param name="scale">The amount by which to scale the vector.</param>
        /// <returns>The scaled vector.</returns>
        public static Vector3d operator *(Vector3d value, double scale)
        {
            return new Vector3d(value.X * scale, value.Y * scale, value.Z * scale);
        }

        /// <summary>
        /// Dot Product.
        /// </summary>
        /// <param name="vec1">First vector.</param>
        /// <param name="vec2">Second vector.</param>
        /// <returns>The scaled vector.</returns>
        public static double operator *(Vector3d vec1, Vector3d vec2)
        {
            return Vector3d.Dot(vec1, vec2);
        }

        public static bool operator ==(Vector3d left, Vector3d right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Vector3d left, Vector3d right)
        {
            return !left.Equals(right);
        }

        /// <summary>
        /// Performs an implicit conversion from <see cref="Point"/> to <see cref="Vector2"/>.
        /// </summary>
        /// <param name="value">The value.</param>
        /// <returns>The result of the conversion.</returns>
        public static implicit operator g.Matrix(Vector3d value)
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
