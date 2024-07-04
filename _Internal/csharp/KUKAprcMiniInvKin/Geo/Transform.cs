using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using g = KUKAprcCoreMini.PRC_MathSharpDX;

namespace KUKAprcCoreMini.Geometry
{
    public class Transform
    {


        /// <summary>
        /// A point with (0,0) coordinates.
        /// </summary>
        public static readonly Transform Zero = new Transform(g.Matrix.Zero);

        /// <summary>
        /// Initializes a new instance of the <see cref="Point"/> struct.
        /// </summary>
        /// <param name="x">The x.</param>
        /// <param name="y">The y.</param>
        public Transform(g.Matrix matrix)
        {
            tmatrix = matrix;
        }

        public Transform()
        {
            tmatrix = g.Matrix.Zero;
        }

        public g.Matrix tmatrix;

        public static Transform Translation(double dx, double dy, double dz)
        {
            g.Matrix matrix = g.Matrix.Translation(new g.Vector3((float)dx, (float)dy, (float)dz));
            Transform xf = new Transform(matrix);
            return xf;
        }

        public static Transform Translation(Vector3d vec)
        {
            g.Matrix matrix = g.Matrix.Translation(new g.Vector3(vec.X, vec.Y, vec.Z));
            Transform xf = new Transform(matrix);
            return xf;
        }

        public static Transform Rotation(float angle, g.Vector3 vector, g.Vector3 origin)
        {
            g.Matrix movematrix = g.Matrix.Translation(origin);
            g.Matrix invmovematrix = g.Matrix.Translation(origin);
            invmovematrix.Invert();

            g.Matrix rotmatrix = g.Matrix.RotationAxis(vector, angle);

            g.Matrix transformmatrix = g.Matrix.Multiply(rotmatrix, invmovematrix);
            g.Matrix outmatrix = g.Matrix.Multiply(movematrix, transformmatrix);

            Transform xf = new Transform(outmatrix);
            return xf;
        }

        public static Transform Rotation(double angle, g.Vector3 vector, g.Vector3 origin)
        {
            vector.Normalize();
            g.Matrix movematrix = g.Matrix.Translation(origin);
            g.Matrix invmovematrix = g.Matrix.Translation(origin);
            invmovematrix.Invert();

            g.Matrix rotmatrix = g.Matrix.RotationAxis(vector, Convert.ToSingle(angle));

            g.Matrix transformmatrix = g.Matrix.Multiply(invmovematrix, rotmatrix);
            g.Matrix outmatrix = g.Matrix.Multiply(transformmatrix, movematrix);

            Transform xf = new Transform(outmatrix);
            return xf;
        }

        public static Transform Rotation(double angle, g.Vector3 origin)
        {
            g.Vector3 vector = new g.Vector3(0, 0, 1);
            g.Matrix movematrix = g.Matrix.Translation(origin);
            g.Matrix invmovematrix = g.Matrix.Translation(origin);
            invmovematrix.Invert();

            g.Matrix rotmatrix = g.Matrix.RotationAxis(vector, (float)angle);

            g.Matrix transformmatrix = g.Matrix.Multiply(invmovematrix, rotmatrix);
            g.Matrix outmatrix = g.Matrix.Multiply(transformmatrix, movematrix);

            Transform xf = new Transform(outmatrix);
            return xf;
        }

        public static Transform PlaneToPlane(Plane plnfrom, Plane plnto)
        {
            g.Matrix from = plnfrom.GetMatrix;
            g.Matrix to = plnto.GetMatrix;

            from.Invert();

            Transform xf = new Transform(g.Matrix.Multiply(from, to));
            return xf;
        }

        public static Transform ChangeBasis(Plane plnfrom, Plane plnto)
        {
            Plane XY = new Plane(new Point3d(0, 0, 0), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));


            g.Matrix from = XY.GetMatrix;
            g.Matrix to = plnfrom.GetMatrix;
            from.Invert();
            g.Matrix tmp = g.Matrix.Multiply(from, to);

            g.Matrix from2 = plnto.GetMatrix;
            g.Matrix to2 = XY.GetMatrix;
            from2.Invert();
            g.Matrix tmp2 = g.Matrix.Multiply(from2, to2);

            Transform xf = new Transform(g.Matrix.Multiply(tmp, tmp2));
            return xf;
        }

        /// <summary>
        /// Determines whether the specified <see cref="Systeg.Object"/> is equal to this instance.
        /// </summary>
        /// <param name="other">The <see cref="Systeg.Object"/> to compare with this instance.</param>
        /// <returns>
        ///   <c>true</c> if the specified <see cref="Systeg.Object"/> is equal to this instance; otherwise, <c>false</c>.
        /// </returns>
        public bool Equals(g.Matrix other)
        {
            return (tmatrix.Equals(other));
        }

        /// <inheritdoc/>
        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj)) return false;
            if (obj.GetType() != typeof(Transform)) return false;
            return Equals((Transform)obj);
        }

        /// <inheritdoc/>
        public override int GetHashCode()
        {
            unchecked
            {
                return 0;
            }
        }

        public override string ToString()
        {
            return "transform";
        }

        public double M00
        {
            get
            {
                return this.tmatrix.M11;
            }
            set
            {
                this.tmatrix.M11 = Convert.ToSingle(value);
            }
        }

        public double M01
        {
            get
            {
                return this.tmatrix.M12;
            }
            set
            {
                this.tmatrix.M12 = Convert.ToSingle(value);
            }
        }

        public double M02
        {
            get
            {
                return this.tmatrix.M13;
            }
            set
            {
                this.tmatrix.M13 = Convert.ToSingle(value);
            }
        }

        public double M03
        {
            get
            {
                return this.tmatrix.M14;
            }
            set
            {
                this.tmatrix.M14 = Convert.ToSingle(value);
            }
        }

        /// <summary>
        /// /////////
        /// </summary>

        public double M10
        {
            get
            {
                return this.tmatrix.M21;
            }
            set
            {
                this.tmatrix.M21 = Convert.ToSingle(value);
            }
        }

        public double M11
        {
            get
            {
                return this.tmatrix.M22;
            }
            set
            {
                this.tmatrix.M22 = Convert.ToSingle(value);
            }
        }

        public double M12
        {
            get
            {
                return this.tmatrix.M23;
            }
            set
            {
                this.tmatrix.M23 = Convert.ToSingle(value);
            }
        }

        public double M13
        {
            get
            {
                return this.tmatrix.M24;
            }
            set
            {
                this.tmatrix.M24 = Convert.ToSingle(value);
            }
        }

        /// <summary>
        /// /////////
        /// </summary>

        public double M20
        {
            get
            {
                return this.tmatrix.M31;
            }
            set
            {
                this.tmatrix.M31 = Convert.ToSingle(value);
            }
        }

        public double M21
        {
            get
            {
                return this.tmatrix.M32;
            }
            set
            {
                this.tmatrix.M32 = Convert.ToSingle(value);
            }
        }

        public double M22
        {
            get
            {
                return this.tmatrix.M33;
            }
            set
            {
                this.tmatrix.M33 = Convert.ToSingle(value);
            }
        }

        public double M23
        {
            get
            {
                return this.tmatrix.M34;
            }
            set
            {
                this.tmatrix.M34 = Convert.ToSingle(value);
            }
        }

        /// <summary>
        /// /////////
        /// </summary>

        public double M30
        {
            get
            {
                return this.tmatrix.M41;
            }
            set
            {
                this.tmatrix.M41 = Convert.ToSingle(value);
            }
        }

        public double M31
        {
            get
            {
                return this.tmatrix.M42;
            }
            set
            {
                this.tmatrix.M42 = Convert.ToSingle(value);
            }
        }

        public double M32
        {
            get
            {
                return this.tmatrix.M43;
            }
            set
            {
                this.tmatrix.M43 = Convert.ToSingle(value);
            }
        }

        public double M33
        {
            get
            {
                return this.tmatrix.M44;
            }
            set
            {
                this.tmatrix.M44 = Convert.ToSingle(value);
            }
        }


        /// <summary>
        /// Performs an implicit conversion from <see cref="Point"/> to <see cref="Vector2"/>.
        /// </summary>
        /// <param name="value">The value.</param>
        /// <returns>The result of the conversion.</returns>
        public static implicit operator g.Matrix(Transform value)
        {
            return value.tmatrix;
        }


    }
}
