using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using g = KUKAprcCoreMini.PRC_MathSharpDX;

namespace KUKAprcCoreMini.Geometry
{
    public class Spline : Polyline
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

        public Spline()
        {
            this.Clear();
            color = Color.FromArgb(255, 0, 0, 0);
        }

        public Spline(List<Point3d> pts, double resolution)
        {
            this.Clear();
            this.AddRange(pts);
            this.Smooth(resolution);
            color = Color.FromArgb(255, 0, 0, 0);
        }

        public void Smooth(double resolution)
        {
            List<Point3d> controlPointsList = new List<Point3d>(this);

            Vector3d tangent1 = (controlPointsList[0] - controlPointsList[1]);
            tangent1 = tangent1.Unitize();

            Vector3d firstpt = controlPointsList[0] + tangent1;

            //Vector3d tangent2 = controlPointsList[controlPointsList.Count - 1] - controlPointsList[controlPointsList.Count - 2];
            //tangent2.Unitize();

            Vector3d lastpt = controlPointsList[controlPointsList.Count - 1];

            controlPointsList.Insert(0, firstpt);
            controlPointsList.Add(controlPointsList[controlPointsList.Count - 1]);

            List<Point3d> pts = new List<Point3d>();

            List<int> mainpts = new List<int>();
            //Draw the Catmull-Rom spline between the points
            bool isLooping = false;

            pts.Add(controlPointsList[0]);
            mainpts.Add(0);

            for (int i = 0; i < controlPointsList.Count; i++)
            {
                //Cant draw between the endpoints
                //Neither do we need to draw from the second to the last endpoint
                //...if we are not making a looping line
                if ((i == 0 || i == controlPointsList.Count - 2 || i == controlPointsList.Count - 1) && !isLooping)
                {
                    continue;
                }

                //pts.AddRange(DisplayCatmullRomSpline(resolution, i, controlPointsList));

                Polyline pl = new Polyline(DisplayCatmullRomSpline(1, i, controlPointsList));
                double desiredpts = pl.Length / resolution;



                double correctionfactor = Convert.ToDouble(pl.Count) / desiredpts;

                if (!double.IsNaN(correctionfactor))
                {
                    pts.AddRange(DisplayCatmullRomSpline(correctionfactor, i, controlPointsList));
                }
                else
                {
                    pts.Add(controlPointsList[i]);
                }
                mainpts.Add(pts.Count - 1);



            }
            

            this.Clear();
            this.AddRange(pts);
            List<int> mainindices = this.DeleteShortSegments(resolution + 0.1, mainpts);

            //List<int> modmainpts = new List<int>();

            //for (int i = 0; i < mainpts.Count; i++)
            //{
            //    int indexfitter = 0;
            //    if (reduced[i])
            //    {
            //        int searcher = 1;
            //        while (i + searcher < mainpts.Count)
            //        {
            //            if (reduced[i + searcher])
            //            {
            //                modmainpts.Add(mainpts[i + searcher]);
            //                break;
            //            }
            //            else
            //            {
            //                searcher++;
            //            }
            //        }
            //    }
            //    else
            //    {
            //        modmainpts.Add(mainpts[i]);
            //    }
            //}
            this.mainindices = mainindices;

        }


        private List<Point3d> DisplayCatmullRomSpline(double res, int pos, List<Point3d> controlPointsList)
        {
            List<Point3d> ptlist = new List<Point3d>();
            //The 4 points we need to form a spline between p1 and p2
            Vector3d p0 = controlPointsList[ClampListPos(pos - 1, controlPointsList)];
            Vector3d p1 = controlPointsList[pos];
            Vector3d p2 = controlPointsList[ClampListPos(pos + 1, controlPointsList)];
            Vector3d p3 = controlPointsList[ClampListPos(pos + 2, controlPointsList)];

            //The start position of the line
            Vector3d lastPos = p1;

            //The spline's resolution
            //Make sure it's is adding up to 1, so 0.3 will give a gap, but 0.2 will work
            Vector3d distance = Vector3d.Subtract(p1, p2);

            float resolution = Convert.ToSingle(res / distance.Length);

            //How many times should we loop?
            int loops = Convert.ToInt32(Math.Floor(1f / resolution));

            for (int i = 1; i <= loops; i++)
            {
                //Which t position are we at?
                float t = i * resolution;

                //Find the coordinate between the end points with a Catmull-Rom spline
                Vector3d newPos = GetCatmullRomPosition(t, p0, p1, p2, p3);

                //Draw this line segment
                //Gizmos.DrawLine(lastPos, newPos);
                ptlist.Add(lastPos);
                ptlist.Add(newPos);

                //Save this pos so we can draw the next line segment
                lastPos = newPos;
            }
            return ptlist;
        }

        //Clamp the list positions to allow looping
        private int ClampListPos(int pos, List<Point3d> controlPointsList)
        {
            if (pos < 0)
            {
                pos = controlPointsList.Count - 1;
            }

            if (pos > controlPointsList.Count)
            {
                pos = 1;
            }
            else if (pos > controlPointsList.Count - 1)
            {
                pos = 0;
            }

            return pos;
        }

        //Returns a position between 4 Vector3d with Catmull-Rom spline algorithm
        //http://www.iquilezles.org/www/articles/minispline/minispline.htm
        private Vector3d GetCatmullRomPosition(float t, Vector3d p0, Vector3d p1, Vector3d p2, Vector3d p3)
        {
            //The coefficients of the cubic polynomial (except the 0.5f * which I added later for performance)
            Vector3d a = 2f * p1;
            Vector3d b = p2 - p0;
            Vector3d c = 2f * p0 - 5f * p1 + 4f * p2 - p3;
            Vector3d d = (-1.0) * p0 + 3f * p1 - 3f * p2 + p3;

            //The cubic polynomial: a + b * t + c * t^2 + d * t^3
            Vector3d pos = 0.5f * (a + (b * t) + (c * t * t) + (d * t * t * t));

            return pos;
        }

        /// <inheritdoc/>
        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj)) return false;
            if (obj.GetType() != typeof(Spline)) return false;
            return Equals((Spline)obj);
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
            return "spline";
        }

        
    }
}
