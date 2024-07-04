using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using g = KUKAprcCoreMini.PRC_MathSharpDX;

namespace KUKAprcCoreMini.Geometry
{
    public class Arc : Polyline
    {
        ///// <summary>
        ///// Initializes a new instance of the <see cref="Point"/> struct.
        ///// </summary>
        ///// <param name="x">The x.</param>
        ///// <param name="y">The y.</param>
        //public Polyline(List<Point3d> polypts)
        //{
        //    pts = polypts;
        //}

        public Arc()
        {
            this.Clear();
            color = Color.FromArgb(255, 0, 0, 0);
        }

        public Arc(List<Point3d> arcpts, double resolution)
        {
            this.Clear();
            if (arcpts.Count == 3)
            {
                List<Point3d> arcptslist = CreateArc(arcpts[0], arcpts[1], arcpts[2], resolution);
                if (arcptslist != null)
                {
                    this.AddRange(arcptslist);
                }
            }
            color = Color.FromArgb(255, 0, 0, 0);
        }

        //Set the points, the arc will go from P1 to P3 though P2.
        private List<Point3d> CreateArc(Vector3d P1, Vector3d P2, Vector3d P3, double resolution)
        {
            Vector3d Center = new Vector3d(0, 0, 0);
            double Radius = 0;
            double Angle;
            Vector3d FDirP1 = new Vector3d(0, 0, 0);
            Vector3d FDirP2 = new Vector3d(0, 0, 0);

            Vector3d O = (P2 + P3) * 0.5;
            Vector3d C = (P1 + P3) * 0.5;
            Vector3d X = (P2 - P1) * -0.5;

            Vector3d N = Vector3d.CrossProduct((P3 - P1), (P2 - P1));
            Vector3d D = Vector3d.CrossProduct(N.Unitize(), (P2 - O));
            Vector3d V = (P1 - C).Unitize();

            double check = Vector3d.Dot(D, V);
            Angle = Math.PI;
            var exist = false;

            if (check != 0)
            {
                double t = Vector3d.Dot(X, V) / check;
                Center = O + D * t;
                Radius = (Center - P1).Length;
                Vector3d V1 = (P1 - Center).Unitize();

                //vector from center to P1
                FDirP1 = V1;
                Vector3d V2 = (P3 - Center).Unitize();
                Angle = Math.Acos(Vector3d.Dot(V1, V2));

                if (Angle != 0)
                {
                    exist = true;
                    V1 = P2 - P1;
                    V2 = P2 - P3;
                    if (Vector3d.Dot(V1, V2) > 0)
                    {
                        Angle = Math.PI * 2 - Angle;
                    }
                }
            }

            //vector from center to P2
            FDirP2 = Vector3d.CrossProduct(N * (-1.0), (P1 - Center)).Unitize();

            if (double.IsNaN(Angle))
            {
                Angle = 0;
            }

            if (exist)
            {
                int divisions = Convert.ToInt32((Radius * Angle) / resolution);

                List<Point3d> ptlist = new List<Point3d>();
                for (int i = 0; i < divisions + 1; i++)
                {
                    double t = Convert.ToDouble(i) / Convert.ToDouble(divisions);
                    var x = t * Angle;
                    ptlist.Add(Center + Radius * Math.Cos(x) * FDirP1 + Radius * Math.Sin(x) * FDirP2);

                }
                return ptlist;

            }
            else
            {
                return null;
            }
        }




        /// <inheritdoc/>
        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj)) return false;
            if (obj.GetType() != typeof(Arc)) return false;
            return Equals((Arc)obj);
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
            return "arc";
        }
        
    }
}
