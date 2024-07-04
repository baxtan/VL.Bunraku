using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using g = KUKAprcCoreMini.PRC_MathSharpDX;

namespace KUKAprcCoreMini.Geometry
{
    public class Plane
    {


        /// <summary>
        /// An unset plane.
        /// </summary>
        public static Plane Unset
        {
            get
            {
                Plane pln = new Plane(new Point3d(0, 0, 0), new Vector3d(0, 0, 0), new Vector3d(0, 0, 0));
                return pln;
            }

        }

        public static Plane WorldXY
        {
            get
            {
                Plane pln = new Plane(new Point3d(0, 0, 0), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));

                return pln;
            }

        }


        /// <summary>
        /// Initializes a new instance of the <see cref="Plane"/> struct.
        /// </summary>
        /// <param name="Origin">The Origin</param>
        /// <param name="XAxis">The x Vector</param>
        /// <param name="YAxis">The y Vector</param>

        
        public Plane(Point3d Origin, Vector3d XAxis, Vector3d YAxis)
        {
            g.Vector3 z = g.Vector3.Cross(XAxis, YAxis);
            z.Normalize();
            g.Vector3 y = g.Vector3.Cross(z, XAxis);
            y.Normalize();
            PLNmatrix = new g.Matrix();
            PLNmatrix.Row1 = new g.Vector4(XAxis.X, XAxis.Y, XAxis.Z, 0);
            PLNmatrix.Row2 = new g.Vector4(y.X, y.Y, y.Z, 0);
            PLNmatrix.Row3 = new g.Vector4(z.X, z.Y, z.Z, 0);
            PLNmatrix.Row4 = new g.Vector4(Origin.X, Origin.Y, Origin.Z, 1);
            PLNmatrix.M44 = 1;

            this.origin = Origin;
            xAxis = XAxis;
            yAxis = y;
            zAxis = z;
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Plane"/> struct.
        /// </summary>
        /// <param name="origin">The Origin</param>
        /// <param name="pt2">Another Point</param>
        /// <param name="pt2">Another Point</param>
        public Plane(Point3d origin, Point3d pt2, Point3d pt3)
        {
            Vector3d x = Point3d.Subtract(pt2, origin);
            Vector3d y = Point3d.Subtract(pt3, origin);
            g.Vector3 z = g.Vector3.Cross(x, y);
            PLNmatrix = new g.Matrix();
            PLNmatrix.Row1 = new g.Vector4(x.X, x.Y, x.Z, 0);
            PLNmatrix.Row2 = new g.Vector4(y.X, y.Y, y.Z, 0);
            PLNmatrix.Row3 = new g.Vector4(z.X, z.Y, z.Z, 0);
            PLNmatrix.Row4 = new g.Vector4(origin.X, origin.Y, origin.Z, 1);
            PLNmatrix.M44 = 1;

            this.origin = origin;
            xAxis = x;
            yAxis = y;
            zAxis = z;
        }



        public Plane()
        {
            g.Vector3 x = Vector3d.XAxis;
            g.Vector3 y = Vector3d.YAxis;
            g.Vector3 z = Vector3d.ZAxis;
            g.Vector3 origin = Point3d.Origin;

            PLNmatrix = new g.Matrix();
            PLNmatrix.Row1 = new g.Vector4(x.X, x.Y, x.Z, 0);
            PLNmatrix.Row2 = new g.Vector4(y.X, y.Y, y.Z, 0);
            PLNmatrix.Row3 = new g.Vector4(z.X, z.Y, z.Z, 0);
            PLNmatrix.Row4 = new g.Vector4(origin.X, origin.Y, origin.Z, 1);
            PLNmatrix.M44 = 1;

            xAxis = x;
            yAxis = y;
            zAxis = z;
            this.origin = origin;
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Plane"/> struct.
        /// </summary>
        /// <param name="origin">The Origin</param>
        /// <param name="vec">The Normal Vector</param>
        public Plane(Point3d origin, Vector3d vec)
        {
            KUKAprcCoreMini.PRC_MathSharpDX.Plane pln = new g.Plane(origin, vec);

            //PRC_MathSharpDX.Matrix tmpmatrix = pln.Reflection();

            //g.Vector3 x = new g.Vector3(tmpmatrix.Row1.X, tmpmatrix.Row1.Y, tmpmatrix.Row1.Z);
            //g.Vector3 y = new g.Vector3(tmpmatrix.Row2.X, tmpmatrix.Row2.Y, tmpmatrix.Row2.Z);
            //g.Vector3 x = new g.Vector3(tmpmatrix.Row1.X, tmpmatrix.Row1.Y, tmpmatrix.Row1.Z);
            //x = x * -1;
            g.Vector3 x = new g.Vector3(0,0,1);

            g.Vector3 vectmp = vec.Unitize();
            if (x.X == vectmp.X && x.Y == vectmp.Y && x.Z == Math.Abs(vectmp.Z))
            {
                x = new g.Vector3(1, 0, 0);
            }


            //if (x.Equals(vec))
            //{
            //    x = new g.Vector3(1, 0, 0);
            //}

            g.Vector3 z = vec;
            g.Vector3 y = g.Vector3.Cross(z, x);
            x = g.Vector3.Cross(y, z);

            x.Normalize();
            y.Normalize();
            z.Normalize();


            PLNmatrix = new g.Matrix();
            PLNmatrix.Row1 = new g.Vector4(x.X, x.Y, x.Z, 0);
            PLNmatrix.Row2 = new g.Vector4(y.X, y.Y, y.Z, 0);
            PLNmatrix.Row3 = new g.Vector4(z.X, z.Y, z.Z, 0);
            PLNmatrix.Row4 = new g.Vector4(origin.X, origin.Y, origin.Z, 1);
            PLNmatrix.M44 = 1;

            xAxis = x;
            yAxis = y;
            zAxis = z;
            this.origin = new Vector3d(origin);



        }

        public Plane(g.Matrix matrix)
        {
            xAxis = new g.Vector3(matrix.Row1.X, matrix.Row1.Y, matrix.Row1.Z);
            yAxis = new g.Vector3(matrix.Row2.X, matrix.Row2.Y, matrix.Row2.Z);
            zAxis = new g.Vector3(matrix.Row3.X, matrix.Row3.Y, matrix.Row3.Z);
            origin = new g.Vector3(matrix.Row4.X, matrix.Row4.Y, matrix.Row4.Z);
            PLNmatrix.M44 = 1;
            PLNmatrix = matrix;
        }

        /// <summary>
        /// Left coordinate.
        /// </summary>


        
        private g.Vector3 xAxis;

        
        private g.Vector3 yAxis;

        
        private g.Vector3 zAxis;

        
        private g.Vector3 origin;

        
        public g.Matrix PLNmatrix;

        
        public g.Matrix GetMatrix
        {
            get
            {
                return this.PLNmatrix;
            }
        }

        public Vector3d XAxis
        {
            get
            {
                return this.xAxis;
            }
            set
            {
                this.xAxis = value;
                this.PLNmatrix.Row1 = new g.Vector4(xAxis.X, xAxis.Y, xAxis.Z, 0);
            }
        }

        public Vector3d YAxis
        {
            get
            {
                return this.yAxis;
            }
            set
            {
                this.yAxis = value;
                this.PLNmatrix.Row2 = new g.Vector4(yAxis.X, yAxis.Y, yAxis.Z, 0);
            }
        }

        
        public Vector3d ZAxis
        {
            get
            {
                return this.zAxis;
            }
            set
            {
                this.zAxis = value;
                this.PLNmatrix.Row3 = new g.Vector4(zAxis.X, zAxis.Y, zAxis.Z, 0);
            }
        }

        public Point3d Origin
        {
            get
            {
                return this.origin;
            }
            set
            {
                this.origin = value;
                this.PLNmatrix.Row4 = new g.Vector4(origin.X, origin.Y, origin.Z, 1);
            }
        }


        
        public double OriginX
        {
            get
            {
                return this.origin.X;
            }
        }


        
        public double OriginY
        {
            get
            {
                return this.origin.Y;
            }
        }


        
        public double OriginZ
        {
            get
            {
                return this.origin.Z;
            }
        }

        public Point3d ClosestPoint(Point3d refpt)
        {
            g.Plane pln = new g.Plane(this.origin, this.zAxis);
            g.Ray ray = new g.Ray(refpt, this.zAxis);

            g.Vector3 outpt = new g.Vector3();
            pln.Intersects(ref ray, out outpt);

            return new Point3d(outpt.X, outpt.Y, outpt.Z);
        }

        public Plane Round(int digits)
        {
            g.Matrix mat = this.GetMatrix;
            mat.M11 = Convert.ToSingle(Math.Round(mat.M11, digits));
            mat.M12 = Convert.ToSingle(Math.Round(mat.M12, digits));
            mat.M13 = Convert.ToSingle(Math.Round(mat.M13, digits));
            mat.M14 = Convert.ToSingle(Math.Round(mat.M14, digits));
            mat.M21 = Convert.ToSingle(Math.Round(mat.M21, digits));
            mat.M22 = Convert.ToSingle(Math.Round(mat.M22, digits));
            mat.M23 = Convert.ToSingle(Math.Round(mat.M23, digits));
            mat.M24 = Convert.ToSingle(Math.Round(mat.M24, digits));
            mat.M31 = Convert.ToSingle(Math.Round(mat.M31, digits));
            mat.M32 = Convert.ToSingle(Math.Round(mat.M32, digits));
            mat.M33 = Convert.ToSingle(Math.Round(mat.M33, digits));
            mat.M34 = Convert.ToSingle(Math.Round(mat.M34, digits));
            mat.M41 = Convert.ToSingle(Math.Round(mat.M41, digits));
            mat.M42 = Convert.ToSingle(Math.Round(mat.M42, digits));
            mat.M43 = Convert.ToSingle(Math.Round(mat.M43, digits));
            mat.M44 = Convert.ToSingle(Math.Round(mat.M44, digits));

            Plane pln = new Plane(mat);
            pln.xAxis.Normalize();
            pln.yAxis.Normalize();

            return new Plane(pln);
        }

        /// <summary>
        /// Determines whether the specified <see cref="Systeg.Object"/> is equal to this instance.
        /// </summary>
        /// <param name="other">The <see cref="Systeg.Object"/> to compare with this instance.</param>
        /// <returns>
        ///   <c>true</c> if the specified <see cref="Systeg.Object"/> is equal to this instance; otherwise, <c>false</c>.
        /// </returns>
        public bool Equals(Plane other)
        {
            return other.XAxis.Equals(XAxis) && other.YAxis.Equals(YAxis) && other.Origin.Equals(Origin);
        }

        /// <inheritdoc/>
        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj)) return false;
            if (obj.GetType() != typeof(Plane)) return false;
            return Equals((Plane)obj);
        }

        /// <inheritdoc/>
        public override int GetHashCode()
        {
            unchecked
            {
                return Convert.ToInt32(0);
            }
        }

        public void Transform(g.Matrix matrix)
        {
            g.Matrix mout = g.Matrix.Multiply(PLNmatrix, matrix);
            xAxis = new g.Vector3(mout.Row1.X, mout.Row1.Y, mout.Row1.Z);
            yAxis = new g.Vector3(mout.Row2.X, mout.Row2.Y, mout.Row2.Z);
            zAxis = new g.Vector3(mout.Row3.X, mout.Row3.Y, mout.Row3.Z);
            origin = new g.Vector3(mout.Row4.X, mout.Row4.Y, mout.Row4.Z);
            
            PLNmatrix = mout;
            PLNmatrix.M44 = 1;
        }

        public void Rotate (double angle, Vector3d vector)
        {
            Transform baserot = KUKAprcCoreMini.Geometry.Transform.Rotation(angle, vector, this.Origin);
            this.Transform(baserot);
        }

        
        public Vector3d Normal
        {
            get
            {
                return new g.Vector3(zAxis.X, zAxis.Y, zAxis.Z);
            }
        }

        public override string ToString()
        {
            return "";
        }

        /// <summary>
        /// Performs an implicit conversion from <see cref="Point"/> to <see cref="Vector2"/>.
        /// </summary>
        /// <param name="value">The value.</param>
        /// <returns>The result of the conversion.</returns>
        public static implicit operator g.Matrix(Plane value)
        {
            return value.PLNmatrix;
        }


    }
}
