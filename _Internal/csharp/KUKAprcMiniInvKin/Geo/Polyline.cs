using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using g = KUKAprcCoreMini.PRC_MathSharpDX;

namespace KUKAprcCoreMini.Geometry
{
    public class Polyline : List<Point3d>
    {
        public Color color;
        public List<int> mainindices;

        ///// <summary>
        ///// Initializes a new instance of the <see cref="Point"/> struct.
        ///// </summary>
        ///// <param name="x">The x.</param>
        ///// <param name="y">The y.</param>
        //public Polyline(List<Point3d> polypts)
        //{
        //    pts = polypts;
        //}

        public Polyline()
        {
            this.Clear();
            color = Color.FromArgb(255, 0, 0, 0);
        }

        public Polyline(Point3d[] polypts)
        {
            this.Clear();
            this.AddRange(polypts);
            color = Color.FromArgb(255, 0, 0, 0);
        }

        public Polyline(List<Point3d> polypts)
        {
            this.Clear();
            this.AddRange(polypts);
            color = Color.FromArgb(255, 0, 0, 0);
        }

        public Polyline(Polyline polypts)
        {
            this.Clear();
            foreach (Point3d pt in polypts)
            {
                this.Add(new Point3d(pt));
            }
            color = new Color(polypts.color);
        }

        public Polyline(Color clr)
        {
            this.Clear();
            color = Color.FromArgb(clr.A, clr.R, clr.G, clr.B);
        }

        ///// <summary>
        ///// Left coordinate.
        ///// </summary>
        //public List<Point3d> pts;

        //public void Add(Point3d pt)
        //{
        //    pts.Add(pt);
        //}

        //public void AddRange(List<Point3d> ptlist)
        //{
        //    pts.AddRange(ptlist);
        //}

        //public void AddRange(Polyline pline)
        //{
        //    pts.AddRange(pline.pts);
        //}

        public Point3d? First
        {
            get
            {
                if (this != null && this.Count > 0)
                {
                    return this[0];
                }
                else
                {
                    return null;
                }
            }
        }

        public Point3d? Last
        {
            get
            {
                if (this != null && this.Count > 0)
                {
                    return this[this.Count - 1];
                }
                else
                {
                    return null;
                }
            }
        }

        public double Length
        {
            get
            {
                /*
                if (this != null && this.Count > 0)
                {
                    double length = 0.0;

                    for (int i = 0; i < this.Count - 2; i++)
                    {
                        length += Vector3d.Subtract(this[i], this[i + 1]).Length;
                    }
                    return length;
                }
                else
                {
                    return Double.NaN;
                }
                */
                return this.Count;
            }
        }

        public double ActualLength
        {
            get
            {
                
                if (this != null && this.Count > 0)
                {
                    double length = 0.0;

                    for (int i = 0; i < this.Count - 2; i++)
                    {
                        length += Vector3d.Subtract(this[i], this[i + 1]).Length;
                    }
                    return length;
                }
                else
                {
                    return Double.NaN;
                }
            }
        }

        public List<int> DeleteShortSegments(double tolerance, List<int> prioritylist = null)
        {
            List<int> mainindices = new List<int>();
            if (this != null && this.Count > 2)
            {
                Point3d refpt = this[0];
                List<Point3d> reduced = new List<Point3d>();
                reduced.Add(this[0]);
                mainindices.Add(reduced.Count - 1);

                for (int i = 1; i < this.Count - 1; i++)
                {
                    double dist = Vector3d.Subtract(new Vector3d(refpt.X, refpt.Y, refpt.Z), new Vector3d(this[i].X, this[i].Y, this[i].Z)).SquareLength - 1;
                    //double dist2 = Vector3d.Subtract(new Vector3d(refpt.X, refpt.Y, refpt.Z), new Vector3d(this[i].X, this[i].Y, this[i].Z)).Length;

                    bool isinlist = false;
                    if (prioritylist != null)
                    {
                        if (prioritylist.Contains(i))
                        {
                            isinlist = true;
                        }
                    }


                    if (dist > (tolerance) | isinlist)
                    {
                        reduced.Add(this[i]);
                        refpt = this[i];
                    }

                    if (isinlist)
                    {
                        mainindices.Add(reduced.Count - 1);
                    }
                }

                reduced.Add(this[this.Count - 1]);
                mainindices.Add(reduced.Count - 1);

                this.Clear();
                this.AddRange(reduced);
            }
            return mainindices;
        }

        public List<Boolean> ReduceSegments(double tolerance)
        {
            List<Point3d> points = new List<Point3d>(this);
            double sqTolerance = tolerance * tolerance;
            var len = points.Count;
            var markers = new int?[len];
            int? first = 0;
            int? last = len - 1;
            int? index = 0;
            var stack = new List<int?>();
            var newPoints = new List<Point3d>();
            List<bool> reducepattern = new List<bool>();

            markers[first.Value] = markers[last.Value] = 1;

            while (last != null)
            {
                var maxSqDist = 0.0d;

                for (int? i = first + 1; i < last; i++)
                {
                    var sqDist = GetSquareSegmentDistance(points[i.Value], points[first.Value], points[last.Value]);

                    if (sqDist > maxSqDist)
                    {
                        index = i;
                        maxSqDist = sqDist;
                    }
                }

                if (maxSqDist > sqTolerance)
                {
                    markers[index.Value] = 1;
                    stack.AddRange(new[] { first, index, index, last });
                }


                if (stack.Count > 0)
                {
                    last = stack[stack.Count - 1];
                    stack.RemoveAt(stack.Count - 1);
                }
                else
                    last = null;

                if (stack.Count > 0)
                {
                    first = stack[stack.Count - 1];
                    stack.RemoveAt(stack.Count - 1);
                }
                else
                    first = null;
            }

            for (var i = 0; i < len; i++)
            {
                if (markers[i] != null)
                {
                    newPoints.Add(points[i]);
                    reducepattern.Add(true);
                }
                else
                {
                    reducepattern.Add(false);
                }
            }

            this.Clear();
            this.AddRange(newPoints);
            return reducepattern;
        }

        public Point3d PointAt(double factor)
        {
            /*
            double length = 0.0;
            int i = 0;
            while (length < factor & i < this.Length)
            {
                length += Vector3d.Subtract(this[i], this[i + 1]).Length;
                i++;
            } */



            int val = Convert.ToInt32(Math.Floor(factor));
            double restfactor = factor - val;

            if (restfactor > 0)
            {
                if (val < this.Count -1)
                {
                    Vector3d vec = Vector3d.Subtract(this[val + 1], this[val]);
                    Point3d outpt = this[val] + vec * restfactor;
                    return outpt;
                }
                else
                {
                    return this[this.Count - 1];
                }
            }
            else
            {
                if (val < this.Count)
                {
                    return this[val];
                }
                else
                {
                    return this[0];
                }
            }

            //return this[i];

        }

        public double ClosestParameter(Point3d pt, bool accurate)
        {
            if (accurate)
            {
                if (this != null && this.Count > 1)
                {
                    double length = 0.0;
                    double bestlength = Vector3d.Subtract(pt, this[0]).SquareLength;


                    for (int i = 1; i < this.Count - 2; i++)
                    {


                        if (Vector3d.Subtract(pt, this[i]).SquareLength < bestlength)
                        {

                            bestlength = length;
                        }


                        length += Vector3d.Subtract(this[i], this[i + 1]).Length;

                    }

                    if (Vector3d.Subtract(pt, this[this.Count - 1]).SquareLength < bestlength)
                    {
                        bestlength = length;
                    }

                    return bestlength;
                }
                return Double.NaN;
            }
            else
            {
                float bestlength = Vector3d.Subtract(pt, this[0]).SquareLength;
                int bestfactor = 0;
                for (int i = 0; i < this.Count; i++)
                {
                    float length = Vector3d.Subtract(pt, this[i]).SquareLength;
                    if (length < bestlength)
                    {
                        bestfactor = i;
                        bestlength = length;
                        if (length == 0)
                        {
                            break;
                        }
                    }
                }
                return bestfactor;
            }
        }

        public static Polyline SetColor (Polyline pl, Color clr)
        {
            pl.color = clr;
            return pl;
        }

        //public static Polyline SetOVColor(Polyline pl, Color clr)
        //{
        //    pl.ovcolor = clr;
        //    return pl;
        //}

        public static List<Polyline> SetColor (List<Polyline> pls, Color clr)
        {
            foreach (Polyline pl in pls)
            {
                pl.color = clr;
            }

            return pls;
        }

        //public static List<Polyline> SetOVColor(List<Polyline> pls, Color clr)
        //{
        //    foreach (Polyline pl in pls)
        //    {
        //        pl.ovcolor = clr;
        //    }

        //    return pls;
        //}


        /// <inheritdoc/>
        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj)) return false;
            if (obj.GetType() != typeof(Polyline)) return false;
            return Equals((Polyline)obj);
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
            List<Point3d> movedpts = new List<Point3d>();

            foreach (Point3d pt in this)
            {
                movedpts.Add(new Point3d(g.Matrix.Multiply(pt, matrix)));
            }

            this.Clear();
            this.AddRange(movedpts);
        }

        public override string ToString()
        {
            return "line";
        }

        private double GetSquareSegmentDistance(Point3d p, Point3d p1, Point3d p2)
        {
            var x = p1.X;
            var y = p1.Y;
            var z = p1.Z;
            var dx = p2.X - x;
            var dy = p2.Y - y;
            var dz = p2.Z - z;

            if (!dx.Equals(0.0) || !dy.Equals(0.0) || !dz.Equals(0.0))
            {
                var t = ((p.X - x) * dx + (p.Y - y) * dy + (p.Z - z) * dz) / (dx * dx + dy * dy + dz * dz);

                if (t > 1)
                {
                    x = p2.X;
                    y = p2.Y;
                    z = p2.Z;
                }
                else if (t > 0)
                {
                    x += dx * t;
                    y += dy * t;
                    z += dz * t;
                }
            }

            dx = p.X - x;
            dy = p.Y - y;
            dz = p.Z - z;

            return (dx * dx) + (dy * dy) + (dz * dz);
        }
    }
}
