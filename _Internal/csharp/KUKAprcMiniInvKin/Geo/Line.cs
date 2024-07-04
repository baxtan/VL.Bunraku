using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using g = KUKAprcCoreMini.PRC_MathSharpDX;

namespace KUKAprcCoreMini.Geometry
{
    public class Line
    {
        private Line()
        { }

        /// <summary>
        /// Initializes a new instance of the <see cref="Point"/> struct.
        /// </summary>
        /// <param name="x">The x.</param>
        /// <param name="y">The y.</param>
        public Line(Point3d ptfrom, Point3d ptto)
        {
            From = ptfrom;
            To = ptto;
        }

        /// <summary>
        /// Left coordinate.
        /// </summary>
        public Point3d From;

        /// <summary>
        /// Top coordinate.
        /// </summary>
        public Point3d To;


        /// <summary>
        /// Determines whether the specified <see cref="System.Object"/> is equal to this instance.
        /// </summary>
        /// <param name="other">The <see cref="System.Object"/> to compare with this instance.</param>
        /// <returns>
        ///   <c>true</c> if the specified <see cref="System.Object"/> is equal to this instance; otherwise, <c>false</c>.
        /// </returns>
        public bool Equals(Line other)
        {
            return (Point3d.Equals(From, other.From) && Point3d.Equals(To, other.To));
        }

        /// <inheritdoc/>
        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj)) return false;
            if (obj.GetType() != typeof(Line)) return false;
            return Equals((Line)obj);
        }

        public Point3d ClosestPoint(Point3d point, bool limittofinite)
        {
            Vector3d linevec = this.To - this.From;
            Vector3d topointvec = point - this.From;

            double magnitude = linevec.SquareLength;
            double product = Vector3d.Dot(linevec, topointvec);
            double distance = product / magnitude;

            if (distance < 0 & limittofinite)
            {
                return this.From;
            }
            else if (distance > 1 & limittofinite)
            {
                return this.To;
            }
            else
            {
                return new Point3d(this.From + linevec * distance);
            }
        }

        public Point3d PointAt(double param)
        {
            return this.From + (this.To - this.From) * param;
        }

        public double ClosestParameter(Point3d pt)
        {
            Vector3d linevec = this.To - this.From;
            Vector3d topointvec = pt - this.From;

            double magnitude = linevec.SquareLength;
            double product = Vector3d.Dot(linevec, topointvec);
            double distance = product / magnitude;

            return distance;
        }

        public float Length
        {
            get
            {
                return (To - From).Length;
            }
        }

        /// <inheritdoc/>
        public override int GetHashCode()
        {
            unchecked
            {
                return 0;
            }
        }

        public void Transform(g.Matrix matrix)
        {
            From = g.Matrix.Multiply(From, matrix);

            To = g.Matrix.Multiply(To, matrix);

            //From = new Point3d(frommout);
            //To = new Point3d(toout);
            //From = frommout;
            //To = toout;
        }

        public override string ToString()
        {
            return "line";
        }


    }
}
