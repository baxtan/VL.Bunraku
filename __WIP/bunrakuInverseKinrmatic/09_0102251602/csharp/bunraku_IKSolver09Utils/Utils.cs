using System;
using Stride.Core.Mathematics;  // Stride’s math library

namespace SolverCoreMini
{
    // A helper class to represent a coordinate frame.
    // This replaces your custom Plane class.
    public class Frame
    {
        public Vector3 Origin;
        public Vector3 XAxis;
        public Vector3 YAxis;

        // ZAxis is derived as the normal (cross product) of the two in‐plane axes.
        public Vector3 ZAxis
        {
            get { return Vector3.Normalize(Vector3.Cross(XAxis, YAxis)); }
        }

        public Frame(Vector3 origin, Vector3 xAxis, Vector3 yAxis)
        {
            Origin = origin;
            XAxis = xAxis;
            YAxis = yAxis;
        }

        // Applies a transformation matrix to the frame.
        public void Transform(Matrix transform)
        {
            Origin = Vector3.TransformCoordinate(Origin, transform);
            XAxis = Vector3.TransformNormal(XAxis, transform);
            YAxis = Vector3.TransformNormal(YAxis, transform);
        }
    }

    // A simple line class with two endpoints.
    public class Line
    {
        public Vector3 From;
        public Vector3 To;

        public Line(Vector3 from, Vector3 to)
        {
            From = from;
            To = to;
        }

        // Applies a transformation matrix to both endpoints.
        public void Transform(Matrix transform)
        {
            From = Vector3.TransformNormal(From, transform);
            To = Vector3.TransformNormal(To, transform);
        }
    }

    // A static helper class to mimic the original Transform methods but using Stride 3D types.
    public static class MyTransform
    {
        // Returns a matrix representing a rotation by 'angle' (in radians) about the given axis passing through origin.
        public static Matrix Rotation(double angle, Vector3 axis, Vector3 origin)
        {
            float ang = (float)angle;
            Matrix translationToOrigin = Matrix.Translation(-origin);
            Matrix rotation = Matrix.RotationAxis(axis, ang);
            Matrix translationBack = Matrix.Translation(origin);
            return translationToOrigin * rotation * translationBack;
        }

        // Returns a translation matrix for the given translation vector.
        public static Matrix Translation(Vector3 translation)
        {
            return Matrix.Translation(translation);
        }

        // Helper to convert a Frame to a transformation matrix.
        public static Matrix FrameToMatrix(Frame frame)
        {
            // Construct a matrix whose 3x3 rotation part is formed by the frame’s axes
            // and whose translation part is the frame’s origin.
            return new Matrix(
                frame.XAxis.X, frame.YAxis.X, frame.ZAxis.X, 0,
                frame.XAxis.Y, frame.YAxis.Y, frame.ZAxis.Y, 0,
                frame.XAxis.Z, frame.YAxis.Z, frame.ZAxis.Z, 0,
                frame.Origin.X, frame.Origin.Y, frame.Origin.Z, 1
            );
        }

        // Computes a transformation that maps coordinates from one frame to another.
        public static Matrix PlaneToPlane(Frame from, Frame to)
        {
            Matrix fromMat = FrameToMatrix(from);
            Matrix toMat = FrameToMatrix(to);
            return Matrix.Invert(fromMat) * toMat;
        }

        // For changing the basis – here equivalent to mapping from 'from' to a canonical frame 'to'.
        public static Matrix ChangeBasis(Frame from, Frame to)
        {
            Matrix fromMat = FrameToMatrix(from);
            Matrix toMat = FrameToMatrix(to);
            return Matrix.Invert(fromMat) * toMat;
        }
    }

    // The core Inverse Kinematics solver, now adapted to use Stride 3D.
    public class InverseKinematic
    {
        /// <summary>
        /// This method calculates the robot's axis position at a given Cartesian coordinate system.
        /// </summary>
        /// <param name="robot">Definition of the robot's kinematic chain. (Several presets are available.)</param>
        /// <param name="Base">The robot's base frame parameters.</param>
        /// <param name="tool">The robot's tool frame.</param>
        /// <param name="targetFrame">The robot's given Cartesian position (as a coordinate frame).</param>
        /// <param name="status">The robot's posture, e.g. "010".</param>
        /// <param name="prevAxVals">The robot's previous axis values, if known; otherwise pass null.</param>
        public static IK_Solver_AxVals IK_Solver_InverseKinematic(
            IK_Solver_RobotMini robot,
            IK_Solver_RobotBase Base,
            IK_Solver_ToolMini tool,
            Frame targetFrame,
            string status,
            IK_Solver_AxVals prevAxVals)
        {
            double a01length = robot.a01length;
            double a01offset = robot.a01offset;
            double a02length = robot.a02length;
            double a03length = robot.a03length;
            double a03offset = robot.a03offset;
            double a04length = robot.a04length;

            a01offset = a01offset * (-1.0);
            a03offset = a03offset * (-1.0);

            if (prevAxVals == null)
            {
                prevAxVals = new IK_Solver_AxVals(0, -90, 90, 0, 0, 0);
            }

            // Calculate Target

            // Define the origin as a Vector3.
            Vector3 ptorigin = new Vector3(0, 0, 0);

            // The canonical coordinate frame.
            Frame plorigin = new Frame(new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0));

            double basevalA = 0; // e.g. 40.0;
            double basevalB = 0; // e.g. 30.0;
            double basevalC = 0; // e.g. 0.0;
            double basevalX = 0;
            double basevalY = 0;
            double basevalZ = 0;

            basevalA = Math.PI * basevalA / 180.0;
            basevalB = Math.PI * basevalB / 180.0;
            basevalC = Math.PI * basevalC / 180.0;

            // Define the base frame using Stride’s Vector3.
            Frame basepl = new Frame(
                new Vector3((float)basevalX, (float)basevalY, (float)basevalZ),
                new Vector3(1, 0, 0),
                new Vector3(0, 1, 0)
            );

            Matrix baserotA = MyTransform.Rotation(basevalA, basepl.ZAxis, basepl.Origin);
            basepl.Transform(baserotA);
            Matrix baserotB = MyTransform.Rotation(basevalB, basepl.YAxis, basepl.Origin);
            basepl.Transform(baserotB);
            Matrix baserotC = MyTransform.Rotation(basevalC, basepl.XAxis, basepl.Origin);
            basepl.Transform(baserotC);

            Frame initialbasepln = new Frame(basepl.Origin, basepl.XAxis, basepl.YAxis);

            double A01rot = 0;
            double A02rot = 0;
            double A03rot = 0;
            double A04rot = 0;
            double A05rot = 0;
            double A06rot = 0;

            bool outsiderange = false;

            double toolvalX = tool.toolvalX;
            double toolvalY = tool.toolvalY;
            double toolvalZ = tool.toolvalZ;
            double toolvalA = tool.toolvalA;
            double toolvalB = tool.toolvalB;
            double toolvalC = tool.toolvalC;

            double toolangA = Math.PI * toolvalA / 180.0;
            double toolangB = Math.PI * toolvalB / 180.0;
            double toolangC = Math.PI * toolvalC / 180.0;

            Vector3 ptTCP = new Vector3((float)toolvalX, (float)toolvalY, (float)toolvalZ);
            Frame toolplane = new Frame(ptTCP, new Vector3(1, 0, 0), new Vector3(0, 1, 0));

            Matrix rottoolA = MyTransform.Rotation(toolangA, toolplane.ZAxis, toolplane.Origin);
            toolplane.Transform(rottoolA);

            Matrix rottoolB = MyTransform.Rotation(toolangB, toolplane.YAxis, toolplane.Origin);
            toolplane.Transform(rottoolB);

            Matrix rottoolC = MyTransform.Rotation(toolangC, toolplane.XAxis, toolplane.Origin);
            toolplane.Transform(rottoolC);

            // Rearranging axes if necessary.
            toolplane = new Frame(toolplane.Origin, toolplane.ZAxis, toolplane.YAxis);

            Frame a456plane = new Frame(
                new Vector3(0, 0, (float)(a04length * -1.0)),
                new Vector3(1, 0, (float)(a04length * -1.0)),
                new Vector3(0, 1, (float)(a04length * -1.0))
            );

            Frame targetFrameMoved = new Frame(toolplane.Origin, targetFrame.XAxis, targetFrame.YAxis);

            Matrix targetplaneshift = MyTransform.PlaneToPlane(toolplane, targetFrameMoved);
            a456plane.Transform(targetplaneshift);

            Vector3 planetranslatevec = targetFrame.Origin - toolplane.Origin;
            Matrix planetranslate = MyTransform.Translation(planetranslatevec);
            a456plane.Transform(planetranslate);

            // Update targetFrame with the transformed a456plane.
            targetFrame = a456plane;

            Matrix baseshift = MyTransform.ChangeBasis(basepl, plorigin);
            targetFrame.Transform(baseshift);

            Vector3 ptN = targetFrame.Origin;

            int statusbit0 = 0;
            int statusbit1 = 1;
            int statusbit2 = 0;

            char[] statusbitchars = status.ToCharArray();
            statusbit0 = Convert.ToInt32(statusbitchars[2].ToString());
            statusbit1 = Convert.ToInt32(statusbitchars[1].ToString());
            statusbit2 = Convert.ToInt32(statusbitchars[0].ToString());

            if (statusbit1 == 1 && statusbit0 == 0)
            {
                // Angle A01-A03 calculation
                A01rot = Math.Atan2(ptN.Y, ptN.X);
                double pval = Math.Sqrt(Math.Pow(ptN.X, 2) + Math.Pow(ptN.Y, 2));
                double cval = Math.Sqrt(Math.Pow((ptN.Z - a01length), 2) + Math.Pow((pval + a01offset), 2));
                double epsilonang = Math.Atan2((ptN.Z - a01length), (pval + a01offset));
                double bval = a02length;
                double aval = Math.Sqrt(Math.Pow(a03offset, 2) + Math.Pow(a03length, 2));
                double sval = (aval + bval + cval) / 2;
                double rval = Math.Sqrt(((sval - aval) * (sval - bval) * (sval - cval)) / sval);
                double alphaang = 2 * Math.Atan(rval / (sval - aval));
                double gammaang = 2 * Math.Atan(rval / (sval - cval));
                double phiang = Math.Atan(a03offset / a03length);
                A02rot = alphaang + epsilonang - (Math.PI / 2.0);
                A03rot = gammaang + phiang - (Math.PI / 2.0);
            }
            else if (statusbit1 == 0 && statusbit0 == 0)
            {
                A01rot = Math.Atan2(ptN.Y, ptN.X);
                double pval = Math.Sqrt(Math.Pow(ptN.X, 2) + Math.Pow(ptN.Y, 2));
                double cval = Math.Sqrt(Math.Pow((ptN.Z - a01length), 2) + Math.Pow((pval + a01offset), 2));
                double epsilonang = Math.Atan2((ptN.Z - a01length), (pval + a01offset));
                double bval = a02length;
                double aval = Math.Sqrt(Math.Pow(a03offset, 2) + Math.Pow(a03length, 2));
                double sval = (aval + bval + cval) / 2;
                double rval = Math.Sqrt(((sval - aval) * (sval - bval) * (sval - cval)) / sval);
                double alphaang = 2 * Math.Atan(rval / (sval - aval));
                double gammaang = 2 * Math.Atan(rval / (sval - cval));
                double phiang = Math.Atan(a03offset / a03length);
                A02rot = 1.5 * Math.PI - alphaang + epsilonang;
                A03rot = 1.5 * Math.PI - gammaang + phiang;
            }
            else if (statusbit1 == 0 && statusbit0 == 1)
            {
                A01rot = Math.Atan2(ptN.Y, ptN.X) + Math.PI;
                double pval = Math.Sqrt(Math.Pow(ptN.X, 2) + Math.Pow(ptN.Y, 2));
                double cval = Math.Sqrt(Math.Pow((ptN.Z - a01length), 2) + Math.Pow((pval - a01offset), 2));
                double epsilonang = Math.Atan2((ptN.Z - a01length), (pval - a01offset));
                double bval = a02length;
                double aval = Math.Sqrt(Math.Pow(a03offset, 2) + Math.Pow(a03length, 2));
                double sval = (aval + bval + cval) / 2;
                double rval = Math.Sqrt(((sval - aval) * (sval - bval) * (sval - cval)) / sval);
                double alphaang = 2 * Math.Atan(rval / (sval - aval));
                double gammaang = 2 * Math.Atan(rval / (sval - cval));
                double phiang = Math.Atan(((-1.0) * a03offset) / a03length);
                A02rot = (Math.PI / 2) - (alphaang + epsilonang);
                A03rot = (1.5 * Math.PI) - gammaang - phiang;
            }
            else if (statusbit1 == 1 && statusbit0 == 1)
            {
                A01rot = Math.Atan2(ptN.Y, ptN.X) + Math.PI;
                double pval = Math.Sqrt(Math.Pow(ptN.X, 2) + Math.Pow(ptN.Y, 2));
                double cval = Math.Sqrt(Math.Pow((ptN.Z - a01length), 2) + Math.Pow((pval - a01offset), 2));
                double epsilonang = Math.Atan2((ptN.Z - a01length), (pval - a01offset));
                double bval = a02length;
                double aval = Math.Sqrt(Math.Pow(a03offset, 2) + Math.Pow(a03length, 2));
                double sval = (aval + bval + cval) / 2;
                double rval = Math.Sqrt(((sval - aval) * (sval - bval) * (sval - cval)) / sval);
                double alphaang = 2 * Math.Atan(rval / (sval - aval));
                double gammaang = 2 * Math.Atan(rval / (sval - cval));
                double phiang = Math.Atan(((-1.0) * a03offset) / a03length);
                A02rot = (Math.PI / 2) + alphaang - epsilonang;
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

            // Robottip Axes (now as a Frame)
            Frame robottip = new Frame(
                new Vector3((float)(a03length + (a01offset * -1.0)), 0f, (float)((a01length + a02length) - a03offset)),
                new Vector3(0, 0, -1),
                new Vector3(0, 1, 0)
            );

            // Parametric Rotation Axes as Lines
            Line[] rotaxes = new Line[6];
            rotaxes[0] = new Line(new Vector3(0, 0, 0), new Vector3(0, 0, (float)a01length));
            rotaxes[1] = new Line(new Vector3((float)(a01offset * -1.0), 0, (float)a01length), new Vector3((float)(a01offset * -1.0), -1f, (float)a01length));
            rotaxes[2] = new Line(new Vector3((float)(a01offset * -1.0), 0, (float)(a01length + a02length)), new Vector3((float)(a01offset * -1.0), -1f, (float)(a01length + a02length)));
            rotaxes[3] = new Line(new Vector3((float)(a01offset * -1.0 + a03length), 0, (float)(a01length + a02length - a03offset)),
                                   new Vector3((float)(a01offset * -1.0 + a03length + a04length), 0, (float)(a01length + a02length - a03offset)));
            rotaxes[4] = new Line(new Vector3((float)(a01offset * -1.0 + a03length), 0, (float)(a01length + a02length - a03offset)),
                                   new Vector3((float)(a01offset * -1.0 + a03length), -1f, (float)(a01length + a02length - a03offset)));
            rotaxes[5] = new Line(new Vector3((float)(a01offset * -1.0 + a03length + a04length), 0, (float)(a01length + a02length - a03offset)),
                                   new Vector3((float)(a01offset * -1.0 + a03length + a04length + 1.0), 0, (float)(a01length + a02length - a03offset)));

            // A01 Transform
            Matrix A01trans = MyTransform.Rotation(A01rot, rotaxes[0].To - rotaxes[0].From, rotaxes[0].From);
            robottip.Transform(A01trans);
            for (int i = 0; i <= 5; i++)
            {
                rotaxes[i].Transform(A01trans);
            }

            // A02 Transform
            Matrix A02trans = MyTransform.Rotation(A02rot, rotaxes[1].To - rotaxes[1].From, rotaxes[1].From);
            robottip.Transform(A02trans);
            for (int i = 1; i <= 5; i++)
            {
                rotaxes[i].Transform(A02trans);
            }

            // A03 Transform
            Matrix A03trans = MyTransform.Rotation(A03rot, rotaxes[2].To - rotaxes[2].From, rotaxes[2].From);
            robottip.Transform(A03trans);
            // (Optional: you may choose whether to transform additional axes with A03trans)

            // Dot Products using Stride's Vector3.Dot
            double dot31 = Vector3.Dot(robottip.XAxis, targetFrame.ZAxis);
            double dot32 = Vector3.Dot(robottip.YAxis, targetFrame.ZAxis);
            double dot33 = Vector3.Dot(robottip.ZAxis, targetFrame.ZAxis);
            double dot23 = Vector3.Dot(robottip.ZAxis, targetFrame.YAxis);
            double dot13 = Vector3.Dot(robottip.ZAxis, targetFrame.XAxis);

            double sinB = Math.Sqrt(Math.Pow(dot31, 2) + Math.Pow(dot32, 2));

            double alpha = Math.Atan2(dot23 / sinB, dot13 / sinB);
            double beta = Math.Atan2(Math.Sqrt(Math.Pow(dot31, 2) + Math.Pow(dot32, 2)), dot33);
            double gamma = Math.Atan2(dot32 / sinB, dot31 / sinB);

            // Final Rotations
            A04rot = Math.PI + gamma;
            A05rot = beta;
            A06rot = alpha * (-1.0);

            // Status adjustment
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

            // Convert radians to degrees (with appropriate offsets)
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

            return IK_Solver_EqualizeKinematics(prevAxVals, new IK_Solver_AxVals(A01rot, A02rot, A03rot, A04rot, A05rot, A06rot));
        }

        private static IK_Solver_AxVals IK_Solver_EqualizeKinematics(IK_Solver_AxVals oldVals, IK_Solver_AxVals newVals)
        {
            return new IK_Solver_AxVals(
                IK_Solver_AdjustAxval(newVals.a01_val, oldVals.a01_val, 1),
                newVals.a02_val,
                newVals.a03_val,
                IK_Solver_AdjustAxval(newVals.a04_val, oldVals.a04_val, 1),
                newVals.a05_val,
                IK_Solver_AdjustAxval(newVals.a06_val, oldVals.a06_val, 1)
            );
        }

        private static double IK_Solver_AdjustAxval(double currentaxval, double prevaxval, int type)
        {
            if (type == 1)
            {
                if (Math.Abs(Math.Abs(currentaxval) - Math.Abs(prevaxval)) == 180)
                {
                    return currentaxval;
                }

                // Get absolute value factor
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
                return newaxval;
            }
            else
            {
                return currentaxval;
            }
        }
    }

    // The axis values and kinematics classes remain largely unchanged.
    public class IK_Solver_AxVals
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

        public IK_Solver_AxVals(double a01_val, double a02_val, double a03_val, double a04_val, double a05_val, double a06_val, double e01_val, double e02_val, double e03_val, double e04_val, int index_val)
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

        public IK_Solver_AxVals(double a01_val, double a02_val, double a03_val, double a04_val, double a05_val, double a06_val)
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
        }

        public IK_Solver_AxVals()
        {
        }

        public IK_Solver_AxVals(IK_Solver_AxVals ToClone)
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

        public IK_Solver_AxVals(List<double> axvals)
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

    public class IK_Solver_RobotMini
    {
        public double a01length = 400;
        public double a01offset = 25;
        public double a02length = 455;
        public double a03length = 420;
        public double a03offset = 25;
        public double a04length = 90;

        public IK_Solver_RobotMini(double a01length, double a01offset, double a02length, double a03length, double a03offset, double a04length)
        {
            this.a01length = a01length;
            this.a01offset = a01offset;
            this.a02length = a02length;
            this.a03length = a03length;
            this.a03offset = a03offset;
            this.a04length = a04length;
        }

        public static IK_Solver_RobotMini IK_Solver_KR610R9002()
        {
            return new IK_Solver_RobotMini(400, 25, 455, 420, 25, 90);
        }

        public static IK_Solver_RobotMini IK_Solver_KR816R20102()
        {
            return new IK_Solver_RobotMini(520, 160, 980, 860, 150, 168);
        }

        public static IK_Solver_RobotMini IK_Solver_KR6R700()
        {
            return new IK_Solver_RobotMini(400, 25, 315, 365, 35, 80);
        }
    }

    public class IK_Solver_RobotBase
    {
        public double basevalA = 0; // e.g. 40.0;
        public double basevalB = 0; // e.g. 30.0;
        public double basevalC = 0; // e.g. 0.0;
        public double basevalX = 0;
        public double basevalY = 0;
        public double basevalZ = 0;

        public IK_Solver_RobotBase()
        {
        }

        public IK_Solver_RobotBase(double basevalA, double basevalB, double basevalC, double basevalX, double basevalY, double basevalZ)
        {
            this.basevalA = basevalA;
            this.basevalB = basevalB;
            this.basevalC = basevalC;
            this.basevalX = basevalX;
            this.basevalY = basevalY;
            this.basevalZ = basevalZ;
        }
    }

    public class IK_Solver_ToolMini
    {
        public double toolvalX = 0;
        public double toolvalY = 0;
        public double toolvalZ = 0;
        public double toolvalA = 0;
        public double toolvalB = 0;
        public double toolvalC = 0;

        public IK_Solver_ToolMini()
        {
        }

        public IK_Solver_ToolMini(double toolvalX, double toolvalY, double toolvalZ, double toolvalA, double toolvalB, double toolvalC)
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
