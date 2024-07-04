using KUKAprcCoreMini.Geometry;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace KUKAprcCoreMini
{
    public class InverseKinematic
    {
        /// <summary>This method calculates the robot's axis position at a given Cartesian coordinate system.</summary>
        /// <param name="robot">Definition of the robot's kinematic chain. PRC_RobotMini contains a few presets to use, e.g. PRC_RobotMini.PRC_KR610R9002() for the values of a KUKA Agilus KR6-10 R900-2 robot.</param>
        /// <param name="tool">The robot's tool frame.</param>
        /// <param name="targetPlane">The robot's given Cartesian position.</param>
        /// <param name="status">The robot's posture, e.g. "010".</param>
        /// <param name="prevAxVals">The robot's previous axis value, if known. Otherwise set to "null"</param>


        
        public static PRC_AxVals PRC_InverseKinematic(PRC_RobotMini robot, PRC_RobotBase Base, PRC_ToolMini tool, Plane targetPlane, string status, PRC_AxVals prevAxVals)
        {
            Double a01length = robot.a01length;
            Double a01offset = robot.a01offset;
            Double a02length = robot.a02length;
            Double a03length = robot.a03length;
            Double a03offset = robot.a03offset;
            Double a04length = robot.a04length;

            a01offset = a01offset * (-1.0);
            a03offset = a03offset * (-1.0);


            if (prevAxVals == null)
            {
                prevAxVals = new PRC_AxVals(0, -90, 90, 0, 0, 0);
            }

            // Calculate Target

            Point3d ptorigin = new Point3d(0, 0, 0);


              plorigin = new Plane(new Point3d(0, 0, 0), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));

            Double basevalA = 0; // 40.0;
            Double basevalB = 0; // 30.0;
            Double basevalC = 0; // 0.0;
            Double basevalX = 0;
            Double basevalY = 0;
            Double basevalZ = 0;

            basevalA = Math.PI * basevalA / 180.0;
            basevalB = Math.PI * basevalB / 180.0;
            basevalC = Math.PI * basevalC / 180.0;

            Plane basepl = new Plane(new Point3d(basevalX, basevalY, basevalZ), new Vector3d(1.0, 0, 0), new Vector3d(0, 1.0, 0));
            Transform baserotA = Transform.Rotation(basevalA, basepl.ZAxis, basepl.Origin);
            basepl.Transform(baserotA);
            Transform baserotB = Transform.Rotation(basevalB, basepl.YAxis, basepl.Origin);
            basepl.Transform(baserotB);
            Transform baserotC = Transform.Rotation(basevalC, basepl.XAxis, basepl.Origin);
            basepl.Transform(baserotC);

            Plane initialbasepln = new Plane(basepl.Origin, basepl.XAxis, basepl.YAxis);

            Double A01rot = 0;
            Double A02rot = 0;
            Double A03rot = 0;
            Double A04rot = 0;
            Double A05rot = 0;
            Double A06rot = 0;

            Boolean outsiderange = false;


            Double toolvalX = tool.toolvalX;
            Double toolvalY = tool.toolvalY;
            Double toolvalZ = tool.toolvalZ;
            Double toolvalA = tool.toolvalA;
            Double toolvalB = tool.toolvalB;
            Double toolvalC = tool.toolvalC;

            Double toolangA = Math.PI * toolvalA / 180.0;
            Double toolangB = Math.PI * toolvalB / 180.0;
            Double toolangC = Math.PI * toolvalC / 180.0;

            Point3d ptTCP = new Point3d(toolvalX, toolvalY, toolvalZ);
            Plane toolplane = new Plane();


            toolplane = new Plane(ptTCP, new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));

            Transform rottoolA = Transform.Rotation(toolangA, toolplane.ZAxis, toolplane.Origin);
            toolplane.Transform(rottoolA);

            Transform rottoolB = Transform.Rotation(toolangB, toolplane.YAxis, toolplane.Origin);
            toolplane.Transform(rottoolB);

            Transform rottoolC = Transform.Rotation(toolangC, toolplane.XAxis, toolplane.Origin);
            toolplane.Transform(rottoolC);

            toolplane = new Plane(toolplane.Origin, toolplane.ZAxis, toolplane.YAxis);


            Plane a456plane = new Plane(new Point3d(0, 0, a04length * (-1.0)), new Point3d(1, 0, a04length * (-1.0)), new Point3d(0, 1, a04length * (-1.0)));


            Plane targetplanemoved = new Plane(toolplane.Origin, targetPlane.XAxis, targetPlane.YAxis);

            Transform targetplaneshift = Transform.PlaneToPlane(toolplane, targetplanemoved);
            a456plane.Transform(targetplaneshift);

            Vector3d planetranslatevec = new Vector3d(targetPlane.OriginX - toolplane.OriginX, targetPlane.OriginY - toolplane.OriginY, targetPlane.OriginZ - toolplane.OriginZ);
            Transform planetranslate = Transform.Translation(planetranslatevec);

            a456plane.Transform(planetranslate);

            targetPlane = a456plane;

            Transform baseshift = Transform.ChangeBasis(basepl, plorigin);

            targetPlane.Transform(baseshift);

            Point3d ptN = targetPlane.Origin;

            int statusbit0 = 0;
            int statusbit1 = 1;
            int statusbit2 = 0;

            char[] statusbitchars = status.ToCharArray();

            statusbit0 = Convert.ToInt32(statusbitchars[2].ToString());
            statusbit1 = Convert.ToInt32(statusbitchars[1].ToString());
            statusbit2 = Convert.ToInt32(statusbitchars[0].ToString());



            if (statusbit1 == 1 & statusbit0 == 0)
            {
                // Angle A01-A03 calculation
                A01rot = Math.Atan2(ptN.Y, ptN.X);
                Double pval = Math.Sqrt(Math.Pow(ptN.X, 2) + Math.Pow(ptN.Y, 2));
                Double cval = Math.Sqrt(Math.Pow((ptN.Z - a01length), 2) + Math.Pow((pval + a01offset), 2));
                Double epsilonang = Math.Atan2((ptN.Z - a01length), (pval + a01offset));
                Double bval = a02length;
                Double aval = Math.Sqrt(Math.Pow((a03offset), 2) + Math.Pow((a03length), 2));
                Double sval = (aval + bval + cval) / 2;
                Double rval = Math.Sqrt(((sval - aval) * (sval - bval) * (sval - cval)) / sval);
                Double alphaang = 2 * Math.Atan(rval / (sval - aval));
                Double gammaang = 2 * Math.Atan(rval / (sval - cval));
                Double phiang = Math.Atan((a03offset) / (a03length));
                A02rot = alphaang + epsilonang - (Math.PI / 2.0);
                A03rot = gammaang + phiang - (Math.PI / 2.0);
            }
            else if (statusbit1 == 0 & statusbit0 == 0)
            {
                // Angle A01-A03 calculation
                A01rot = Math.Atan2(ptN.Y, ptN.X);
                Double pval = Math.Sqrt(Math.Pow(ptN.X, 2) + Math.Pow(ptN.Y, 2));
                Double cval = Math.Sqrt(Math.Pow((ptN.Z - a01length), 2) + Math.Pow((pval + a01offset), 2));
                Double epsilonang = Math.Atan2((ptN.Z - a01length), (pval + a01offset));
                Double bval = a02length;
                Double aval = Math.Sqrt(Math.Pow((a03offset), 2) + Math.Pow((a03length), 2));
                Double sval = (aval + bval + cval) / 2;
                Double rval = Math.Sqrt(((sval - aval) * (sval - bval) * (sval - cval)) / sval);
                Double alphaang = 2 * Math.Atan(rval / (sval - aval));
                Double gammaang = 2 * Math.Atan(rval / (sval - cval));
                Double phiang = Math.Atan((a03offset) / (a03length));
                A02rot = 1.5 * Math.PI - alphaang + epsilonang;
                A03rot = 1.5 * Math.PI - gammaang + phiang;
            }
            else if (statusbit1 == 0 & statusbit0 == 1)
            {
                // Angle A01-A03 calculation
                A01rot = Math.Atan2(ptN.Y, ptN.X) + Math.PI;
                Double pval = Math.Sqrt(Math.Pow(ptN.X, 2) + Math.Pow(ptN.Y, 2));
                Double cval = Math.Sqrt(Math.Pow((ptN.Z - a01length), 2) + Math.Pow((pval - a01offset), 2));
                Double epsilonang = Math.Atan2((ptN.Z - a01length), (pval - a01offset));
                Double bval = a02length;
                Double aval = Math.Sqrt(Math.Pow((a03offset), 2) + Math.Pow((a03length), 2));
                Double sval = (aval + bval + cval) / 2;
                Double rval = Math.Sqrt(((sval - aval) * (sval - bval) * (sval - cval)) / sval);
                Double alphaang = 2 * Math.Atan(rval / (sval - aval));
                Double gammaang = 2 * Math.Atan(rval / (sval - cval));
                Double phiang = Math.Atan(((-1.0) * a03offset) / (a03length));
                A02rot = (Math.PI / 2) - (alphaang + epsilonang);
                A03rot = (1.5 * Math.PI) - gammaang - phiang;
            }
            else if (statusbit1 == 1 & statusbit0 == 1)
            {
                // Angle A01-A03 calculation
                A01rot = Math.Atan2(ptN.Y, ptN.X) + Math.PI;
                Double pval = Math.Sqrt(Math.Pow(ptN.X, 2) + Math.Pow(ptN.Y, 2));
                Double cval = Math.Sqrt(Math.Pow((ptN.Z - a01length), 2) + Math.Pow((pval - a01offset), 2));
                Double epsilonang = Math.Atan2((ptN.Z - a01length), (pval - a01offset));
                Double bval = a02length;
                Double aval = Math.Sqrt(Math.Pow((a03offset), 2) + Math.Pow((a03length), 2));
                Double sval = (aval + bval + cval) / 2;
                Double rval = Math.Sqrt(((sval - aval) * (sval - bval) * (sval - cval)) / sval);
                Double alphaang = 2 * Math.Atan(rval / (sval - aval));
                Double gammaang = 2 * Math.Atan(rval / (sval - cval));
                Double phiang = Math.Atan(((-1.0) * a03offset) / (a03length));
                A02rot = (Math.PI / 2) + alphaang - epsilonang; ;
                A03rot = ((-1.0) * Math.PI / 2) + gammaang - phiang;
            }



            // Check if outside range

            if (double.IsNaN(A02rot))
            {
                if (statusbit0 == 1)
                {
                    A01rot = A01rot + Math.PI;
                }
                outsiderange = true;
                A02rot = -90 * Math.PI / 180;
                A03rot = 90 * Math.PI / 180;
            }

            //Robottip Axes
            Plane robottip = new Plane(new Point3d(a03length + (a01offset * (-1.0)), 0.0, (a01length + a02length) - a03offset), new Vector3d(0, 0, -1), new Vector3d(0, 1, 0));


            //Point3d robottipO = new Point3d(a03length + (a01offset * (-1.0)), 0.0, (a01length + a02length) - a03offset);
            //Point3d robottipX = new Point3d(a03length + (a01offset * (-1.0)), 0.0, (a01length + a02length) - a03offset - 1.0);
            //Point3d robottipY = new Point3d(a03length + (a01offset * (-1.0)), 1.0, (a01length + a02length) - a03offset);
            //Point3d robottipZ = new Point3d(a03length + (a01offset * (-1.0)) + 1.0, 0.0, (a01length + a02length) - a03offset);
            //Line robottipXaxis = new Line(robottipO, robottipX);
            //Line robottipYaxis = new Line(robottipO, robottipY);
            //Line robottipZaxis = new Line(robottipO, robottipZ);


            // Parametric Rotation Axes
            Line[] rotaxes = new Line[6];
            rotaxes[0] = new Line(new Point3d(0, 0, 0), new Point3d(0, 0, a01length));
            rotaxes[1] = new Line(new Point3d((a01offset * (-1.0)), 0, a01length), new Point3d((a01offset * (-1.0)), -1.0, a01length));
            rotaxes[2] = new Line(new Point3d((a01offset * (-1.0)), 0, (a01length + a02length)), new Point3d((a01offset * (-1.0)), (-1.0), (a01length + a02length)));
            rotaxes[3] = new Line(new Point3d((a01offset * (-1.0)) + a03length, 0, a01length + a02length - a03offset), new Point3d((a01offset * (-1.0)) + a03length + a04length, 0, a01length + a02length - a03offset));
            rotaxes[4] = new Line(new Point3d((a01offset * (-1.0)) + a03length, 0, a01length + a02length - a03offset), new Point3d((a01offset * (-1.0)) + a03length, -1.0, a01length + a02length - a03offset));
            rotaxes[5] = new Line(new Point3d((a01offset * (-1.0)) + a03length + a04length, 0, a01length + a02length - a03offset), new Point3d((a01offset * (-1.0)) + a03length + a04length + 1.0, 0, a01length + a02length - a03offset));


            //A01 Transform
            Transform A01trans = Transform.Rotation(A01rot, new Vector3d(rotaxes[0].To.X - rotaxes[0].From.X, rotaxes[0].To.Y - rotaxes[0].From.Y, rotaxes[0].To.Z - rotaxes[0].From.Z), rotaxes[0].From);

            robottip.Transform(A01trans);

            for (int i = 0; i <= 5; i++)
            {
                rotaxes[i].Transform(A01trans);
            }

            //A02 Transform
            Transform A02trans = Transform.Rotation(A02rot, new Vector3d(rotaxes[1].To.X - rotaxes[1].From.X, rotaxes[1].To.Y - rotaxes[1].From.Y, rotaxes[1].To.Z - rotaxes[1].From.Z), rotaxes[1].From);
            robottip.Transform(A02trans);

            for (int i = 1; i <= 5; i++)
            {
                rotaxes[i].Transform(A02trans);
            }


            //A03 Transform
            Transform A03trans = Transform.Rotation(A03rot, new Vector3d(rotaxes[2].To.X - rotaxes[2].From.X, rotaxes[2].To.Y - rotaxes[2].From.Y, rotaxes[2].To.Z - rotaxes[2].From.Z), rotaxes[2].From);
            robottip.Transform(A03trans);

            //for (int i = 2; i <= 5; i++)
            //{
            //    rotaxes[i].Transform(A03trans);
            //}


            //Dot Products

            Double dot31 = (robottip.XAxis * targetPlane.ZAxis);
            Double dot32 = (robottip.YAxis * targetPlane.ZAxis);
            Double dot33 = (robottip.ZAxis * targetPlane.ZAxis);
            Double dot23 = (robottip.ZAxis * targetPlane.YAxis);
            Double dot13 = (robottip.ZAxis * targetPlane.XAxis);

            Double sinB = Math.Sqrt(Math.Pow(dot31, 2) + Math.Pow(dot32, 2));

            Double alpha = Math.Atan2(dot23 / sinB, dot13 / sinB);
            Double beta = Math.Atan2(Math.Sqrt(Math.Pow(dot31, 2) + Math.Pow(dot32, 2)), dot33);
            Double gamma = Math.Atan2(dot32 / sinB, dot31 / sinB);

            // Final Rotations


            A04rot = Math.PI + gamma;
            A05rot = beta;
            A06rot = alpha * (-1.0);

            //Status again
            if (statusbit2 == 1)
            {
                A04rot = Math.PI + gamma;
                A05rot = beta;
                A06rot = alpha * (-1.0);
            }
            else
            {
                A04rot = Math.PI + gamma + Math.PI;
                A05rot = beta * (-1.0);
                A06rot = Math.PI + alpha * (-1.0);
            }




            A04rot = A04rot * (-1.0);

            //degree


            A01rot = A01rot / Math.PI * 180 / (-1.0);
            A02rot = (A02rot / Math.PI * 180 / (-1.0)) - 90.0;
            A03rot = (A03rot / Math.PI * 180 / (-1.0)) + 90;
            A04rot = A04rot / Math.PI * 180;
            A05rot = A05rot / Math.PI * 180 / (-1.0);
            A06rot = A06rot / Math.PI * 180 / (-1.0);

            if (outsiderange == true)
            {
                A04rot = 0.0;
                A05rot = 0.00000001;
                A06rot = 0.0;
            }

            if (A01rot > 170)
            {
                if (A01rot > 180)
                {
                    A01rot = A01rot - 360;
                }
            }
            else if (A01rot < -170)
            {
                if (A01rot < -180)
                {
                    A01rot = A01rot + 360;
                }
            }

            if (A02rot < -270)
            {
                A02rot += 360;
            }
            else if (A02rot > 270)
            {
                A02rot -= 360;
            }

            return PRC_EqualizeKinematics(prevAxVals, new PRC_AxVals(A01rot, A02rot, A03rot, A04rot, A05rot, A06rot));
        }

        private static PRC_AxVals PRC_EqualizeKinematics(PRC_AxVals oldVals, PRC_AxVals newVals)
        {
            return new PRC_AxVals(PRC_AdjustAxval(newVals.a01_val, oldVals.a01_val, 1), newVals.a02_val, newVals.a03_val, PRC_AdjustAxval(newVals.a04_val, oldVals.a04_val, 1), newVals.a05_val, PRC_AdjustAxval(newVals.a06_val, oldVals.a06_val, 1));
        }

        private static double PRC_AdjustAxval(double currentaxval, double prevaxval, int type)
        {
            if (type == 1)
            {
                if (Math.Abs(Math.Abs(currentaxval) - Math.Abs(prevaxval)) == 180)
                {
                    return currentaxval;
                }

                //get absolut val
                double prevAbsfactor = Math.Floor(prevaxval / 360.0);
                double prevAbs = prevaxval - (prevAbsfactor * 360);

                double curAbsfactor = Math.Floor(currentaxval / 360.0);
                double curAbs = currentaxval - (curAbsfactor * 360);

                double newaxval = prevAbsfactor * 360 + curAbs;

                if (Math.Abs(newaxval - prevaxval) > 180)
                {
                    double inverseval = 0;
                    if (newaxval > 0)
                    {
                        inverseval = -360 + newaxval;
                    }
                    else if (newaxval < 0)
                    {
                        inverseval = newaxval + 360;
                    }
                    newaxval = inverseval;

                    if (Math.Abs(newaxval - prevaxval) > 180)
                    {
                        if (prevAbsfactor > 0)
                        {
                            newaxval = (prevAbsfactor + 1) * 360 + curAbs;
                        }
                        else if (prevAbsfactor < 0)
                        {
                            newaxval = (prevAbsfactor - 1) * 360 + curAbs;
                        }
                        else if (prevAbsfactor == 0)
                        {
                            newaxval = 360 + curAbs;
                        }
                    }


                }
                ////get equivalent inverse value
                //double inverseval = 0.0;
                //if (newaxval > 0)
                //{
                //    inverseval = 360 - newaxval;
                //}
                //else if (newaxval < 0)
                //{
                //    inverseval = newaxval + 360;
                //}

                ////choose closer version
                //if ((prevaxval - inverseval) > (prevaxval - newaxval))
                //{
                //    newaxval = inverseval;
                //}


                return newaxval;
            }
            else
            {
                return currentaxval;
            }

        }
    }

    public class PRC_AxVals
    {
        public double a01_val;
        public double a02_val;
        public double a03_val;
        public double a04_val;
        public double a05_val;
        public double a06_val;
        public double e01_val;
        public double e02_val;
        public double e03_val;
        public double e04_val;
        public int index_val;

        public PRC_AxVals(double a01_val, double a02_val, double a03_val, double a04_val, double a05_val, double a06_val, double e01_val, double e02_val, double e03_val, double e04_val, int index_val)
        {
            this.a01_val = a01_val;
            this.a02_val = a02_val;
            this.a03_val = a03_val;
            this.a04_val = a04_val;
            this.a05_val = a05_val;
            this.a06_val = a06_val;
            this.e01_val = e01_val;
            this.e02_val = e02_val;
            this.e03_val = e03_val;
            this.e04_val = e04_val;
            this.index_val = index_val;
        }

        public PRC_AxVals(double a01_val, double a02_val, double a03_val, double a04_val, double a05_val, double a06_val)
        {
            this.a01_val = a01_val;
            this.a02_val = a02_val;
            this.a03_val = a03_val;
            this.a04_val = a04_val;
            this.a05_val = a05_val;
            this.a06_val = a06_val;
            this.e01_val = 0;
            this.e02_val = 0;
            this.e03_val = 0;
            this.e04_val = 0;
            this.index_val = 0;
        }

        public PRC_AxVals()
        {
        }

        public PRC_AxVals(PRC_AxVals ToClone)
        {
            this.a01_val = ToClone.a01_val;
            this.a02_val = ToClone.a02_val;
            this.a03_val = ToClone.a03_val;
            this.a04_val = ToClone.a04_val;
            this.a05_val = ToClone.a05_val;
            this.a06_val = ToClone.a06_val;
            this.e01_val = ToClone.e01_val;
            this.e02_val = ToClone.e02_val;
            this.e03_val = ToClone.e03_val;
            this.e04_val = ToClone.e04_val;
            this.index_val = ToClone.index_val;
        }

        public PRC_AxVals(List<double> axvals)
        {

            if (axvals.Count == 10)
            {
                this.a01_val = axvals[0];
                this.a02_val = axvals[1];
                this.a03_val = axvals[2];
                this.a04_val = axvals[3];
                this.a05_val = axvals[4];
                this.a06_val = axvals[5];
                this.e01_val = axvals[6];
                this.e02_val = axvals[7];
                this.e03_val = axvals[8];
                this.e04_val = axvals[9];

            }
            else if (axvals.Count == 6)
            {
                this.a01_val = axvals[0];
                this.a02_val = axvals[1];
                this.a03_val = axvals[2];
                this.a04_val = axvals[3];
                this.a05_val = axvals[4];
                this.a06_val = axvals[5];
                this.e01_val = 0;
                this.e02_val = 0;
                this.e03_val = 0;
                this.e04_val = 0;
            }

        }

        public List<double> ToList()
        {
            List<double> axvallist = new List<double>();
            axvallist.Add(a01_val);
            axvallist.Add(a02_val);
            axvallist.Add(a03_val);
            axvallist.Add(a04_val);
            axvallist.Add(a05_val);
            axvallist.Add(a06_val);
            axvallist.Add(e01_val);
            axvallist.Add(e02_val);
            axvallist.Add(e03_val);
            axvallist.Add(e04_val);

            return axvallist;
        }

        public override string ToString()
        {
            string retstr = "A1 " + a01_val.ToString() + ", A2 " + a02_val.ToString() + ", A3 " + a03_val.ToString() + ", A4 " + a04_val.ToString() + ", A5 " + a05_val.ToString() + ", A6 " + a06_val.ToString();
            if (!double.IsNaN(e01_val))
            {
                retstr += ", E1 " + e01_val.ToString();
            }
            if (!double.IsNaN(e02_val))
            {
                retstr += ", E2 " + e02_val.ToString();
            }
            if (!double.IsNaN(e03_val))
            {
                retstr += ", E3 " + e03_val.ToString();
            }
            if (!double.IsNaN(e04_val))
            {
                retstr += ", E4 " + e04_val.ToString();
            }

            return retstr;
        }
    }

    public class PRC_RobotMini
    {
        public Double a01length = 400; // 815.0;
        public Double a01offset = 25; // 350;
        public Double a02length = 455; // 850.0;
        public Double a03length = 420; // 820.0;
        public Double a03offset = 25; // 145.0;
        public Double a04length = 90; // 170.0;

        public PRC_RobotMini(double a01length, double a01offset, double a02length, double a03length, double a03offset, double a04length)
        {
            this.a01length = a01length;
            this.a01offset = a01offset;
            this.a02length = a02length;
            this.a03length = a03length;
            this.a03offset = a03offset;
            this.a04length = a04length;
        }

        public static PRC_RobotMini PRC_KR610R9002()
        {
            return new PRC_RobotMini(400, 25, 455, 420, 25, 90);
        }

        public static PRC_RobotMini PRC_KR816R20102()
        {
            return new PRC_RobotMini(520, 160, 980, 860, 150, 168);
        }

        public static PRC_RobotMini PRC_KR6R700()
        {
            return new PRC_RobotMini(400, 25, 315, 365, 35, 80);
        }

    }

    public class PRC_RobotBase
    {
        public Double basevalA = 0; // 40.0;
        public Double basevalB = 0; // 30.0;
        public Double basevalC = 0; // 0.0;
        public Double basevalX = 0;
        public Double basevalY = 0;
        public Double basevalZ = 0;

        public PRC_RobotBase()
        {

        }

         public PRC_RobotBase(double basevalA, double basevalB, double basevalC, double basevalX, double basevalY, double basevalZ)
        {
            this.basevalA = basevalA;
            this.basevalB = basevalB;
            this.basevalC = basevalC;
            this.basevalX = basevalX;
            this.basevalY = basevalY;
            this.basevalZ = basevalZ;
        }
 
        
    }

    public class PRC_ToolMini
    {
        public Double toolvalX = 0;
        public Double toolvalY = 0;
        public Double toolvalZ = 0;
        public Double toolvalA = 0;
        public Double toolvalB = 0;
        public Double toolvalC = 0;

        public PRC_ToolMini()
        {

        }

        public PRC_ToolMini(double toolvalX, double toolvalY, double toolvalZ, double toolvalA, double toolvalB, double toolvalC)
        {
            this.toolvalX = toolvalX;
            this.toolvalY = toolvalY;
            this.toolvalZ = toolvalZ;
            this.toolvalA = toolvalA;
            this.toolvalB = toolvalB;
            this.toolvalC = toolvalC;
        }
    }

}

   
