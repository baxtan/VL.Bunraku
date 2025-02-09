using System;
using System.Collections.Generic;
using Stride.Core.Mathematics;
using SolverCoreMini.Geometry;
using static VL.Core.AppHost;
using Frame = SolverCoreMini.Geometry.Frame;

namespace SolverCoreMini
{
    public class InverseKinematic
    {
        /// <summary>
        /// Calculates the robot’s joint positions for a given Cartesian target.
        /// Note: This version uses the Frame class (a coordinate system) from our Stride integration.
        /// </summary>
        public static IK_Solver_AxVals IK_Solver_InverseKinematic(
            IK_Solver_RobotMini robot,
            IK_Solver_RobotBase Base,
            IK_Solver_ToolMini tool,
            Frame targetFrame, // our coordinate frame instead of a simple Plane
            string status,
            IK_Solver_AxVals prevAxVals)
        {
            #region Extract Robot Parameters
            double a01length = robot.a01length;
            double a01offset = robot.a01offset;
            double a02length = robot.a02length;
            double a03length = robot.a03length;
            double a03offset = robot.a03offset;
            double a04length = robot.a04length;
            a01offset *= -1.0;
            a03offset *= -1.0;
            #endregion

            #region Initialise Previous Axes Values
            if (prevAxVals == null)
            {
                prevAxVals = new IK_Solver_AxVals(0, -90, 90, 0, 0, 0);
            }
            #endregion

            #region Define World and Base Frames
            Frame worldFrame = new Frame(new Vector3(0, 0, 0), Vector3.UnitX, Vector3.UnitY);
            double basevalA = Base.basevalA;
            double basevalB = Base.basevalB;
            double basevalC = Base.basevalC;
            double basevalX = Base.basevalX;
            double basevalY = Base.basevalY;
            double basevalZ = Base.basevalZ;
            // Convert base rotation angles from degrees to radians.
            basevalA = MathUtil.DegreesToRadians(basevalA);
            basevalB = MathUtil.DegreesToRadians(basevalB);
            basevalC = MathUtil.DegreesToRadians(basevalC);
            Frame baseFrame = new Frame(new Vector3((float)basevalX, (float)basevalY, (float)basevalZ), Vector3.UnitX, Vector3.UnitY);
            Matrix baseRotA = TransformUtil.Rotation((float)basevalA, baseFrame.ZAxis, baseFrame.Origin);
            baseFrame.Transform(ref baseRotA);
            Matrix baseRotB = TransformUtil.Rotation((float)basevalB, baseFrame.YAxis, baseFrame.Origin);
            baseFrame.Transform(ref baseRotB);
            Matrix baseRotC = TransformUtil.Rotation((float)basevalC, baseFrame.XAxis, baseFrame.Origin);
            baseFrame.Transform(ref baseRotC);
            #endregion

            #region Initialise Joint Angle Variables
            double A01rot = 0, A02rot = 0, A03rot = 0, A04rot = 0, A05rot = 0, A06rot = 0;
            bool outsideRange = false;
            #endregion

            #region Setup Tool Frame
            double toolangA = MathUtil.DegreesToRadians(tool.toolvalA);
            double toolangB = MathUtil.DegreesToRadians(tool.toolvalB);
            double toolangC = MathUtil.DegreesToRadians(tool.toolvalC);
            Vector3 toolOrigin = new Vector3((float)tool.toolvalX, (float)tool.toolvalY, (float)tool.toolvalZ);
            Frame toolFrame = new Frame(toolOrigin, Vector3.UnitX, Vector3.UnitY);
            Matrix rotToolA = TransformUtil.Rotation((float)toolangA, toolFrame.ZAxis, toolFrame.Origin);
            toolFrame.Transform(ref rotToolA);
            Matrix rotToolB = TransformUtil.Rotation((float)toolangB, toolFrame.YAxis, toolFrame.Origin);
            toolFrame.Transform(ref rotToolB);
            Matrix rotToolC = TransformUtil.Rotation((float)toolangC, toolFrame.XAxis, toolFrame.Origin);
            toolFrame.Transform(ref rotToolC);
            // Reassign tool frame if needed.
            toolFrame = new Frame(toolFrame.Origin, toolFrame.ZAxis, toolFrame.YAxis);
            #endregion

            #region Compute Target Frame via Tool Transformation
            Frame a456Frame = new Frame(new Vector3(0, 0, (float)(a04length * -1.0)), new Vector3(1, 0, 0), new Vector3(0, 1, 0));
            Frame targetFrameMoved = new Frame(toolFrame.Origin, targetFrame.XAxis, targetFrame.YAxis);
            Matrix targetFrameShift = TransformUtil.PlaneToPlane(toolFrame, targetFrameMoved);
            a456Frame.Transform(ref targetFrameShift);
            Vector3 frameTranslateVec = targetFrame.Origin - toolFrame.Origin;
            Matrix frameTranslate = TransformUtil.Translation(frameTranslateVec);
            a456Frame.Transform(ref frameTranslate);
            targetFrame = a456Frame;
            Matrix baseShift = TransformUtil.ChangeBasis(baseFrame, worldFrame);
            targetFrame.Transform(ref baseShift);
            #endregion

            #region Extract Target Point and Process Status Bits
            Vector3 ptN = targetFrame.Origin;
            char[] statusChars = status.ToCharArray();
            int statusBit0 = Convert.ToInt32(statusChars[2].ToString());
            int statusBit1 = Convert.ToInt32(statusChars[1].ToString());
            int statusBit2 = Convert.ToInt32(statusChars[0].ToString());
            #endregion

            #region Compute Joint Angles A01, A02, and A03 (Based on Status)
            if (statusBit1 == 1 && statusBit0 == 0)
            {
                A01rot = Math.Atan2(ptN.Y, ptN.X);
                double pVal = Math.Sqrt(ptN.X * ptN.X + ptN.Y * ptN.Y);
                double cVal = Math.Sqrt(Math.Pow(ptN.Z - a01length, 2) + Math.Pow(pVal + a01offset, 2));
                double epsilonAng = Math.Atan2(ptN.Z - a01length, pVal + a01offset);
                double bVal = a02length;
                double aVal = Math.Sqrt(a03offset * a03offset + a03length * a03length);
                double sVal = (aVal + bVal + cVal) / 2.0;
                double rVal = Math.Sqrt(((sVal - aVal) * (sVal - bVal) * (sVal - cVal)) / sVal);
                double alphaAng = 2 * Math.Atan(rVal / (sVal - aVal));
                double gammaAng = 2 * Math.Atan(rVal / (sVal - cVal));
                double phiAng = Math.Atan(a03offset / a03length);
                A02rot = alphaAng + epsilonAng - (Math.PI / 2.0);
                A03rot = gammaAng + phiAng - (Math.PI / 2.0);
            }
            else if (statusBit1 == 0 && statusBit0 == 0)
            {
                A01rot = Math.Atan2(ptN.Y, ptN.X);
                double pVal = Math.Sqrt(ptN.X * ptN.X + ptN.Y * ptN.Y);
                double cVal = Math.Sqrt(Math.Pow(ptN.Z - a01length, 2) + Math.Pow(pVal + a01offset, 2));
                double epsilonAng = Math.Atan2(ptN.Z - a01length, pVal + a01offset);
                double bVal = a02length;
                double aVal = Math.Sqrt(a03offset * a03offset + a03length * a03length);
                double sVal = (aVal + bVal + cVal) / 2.0;
                double rVal = Math.Sqrt(((sVal - aVal) * (sVal - bVal) * (sVal - cVal)) / sVal);
                double alphaAng = 2 * Math.Atan(rVal / (sVal - aVal));
                double gammaAng = 2 * Math.Atan(rVal / (sVal - cVal));
                double phiAng = Math.Atan(a03offset / a03length);
                A02rot = 1.5 * Math.PI - alphaAng + epsilonAng;
                A03rot = 1.5 * Math.PI - gammaAng + phiAng;
            }
            else if (statusBit1 == 0 && statusBit0 == 1)
            {
                A01rot = Math.Atan2(ptN.Y, ptN.X) + Math.PI;
                double pVal = Math.Sqrt(ptN.X * ptN.X + ptN.Y * ptN.Y);
                double cVal = Math.Sqrt(Math.Pow(ptN.Z - a01length, 2) + Math.Pow(pVal - a01offset, 2));
                double epsilonAng = Math.Atan2(ptN.Z - a01length, pVal - a01offset);
                double bVal = a02length;
                double aVal = Math.Sqrt(a03offset * a03offset + a03length * a03length);
                double sVal = (aVal + bVal + cVal) / 2.0;
                double rVal = Math.Sqrt(((sVal - aVal) * (sVal - bVal) * (sVal - cVal)) / sVal);
                double alphaAng = 2 * Math.Atan(rVal / (sVal - aVal));
                double gammaAng = 2 * Math.Atan(rVal / (sVal - cVal));
                double phiAng = Math.Atan((-a03offset) / a03length);
                A02rot = (Math.PI / 2.0) - (alphaAng + epsilonAng);
                A03rot = (1.5 * Math.PI) - gammaAng - phiAng;
            }
            else if (statusBit1 == 1 && statusBit0 == 1)
            {
                A01rot = Math.Atan2(ptN.Y, ptN.X) + Math.PI;
                double pVal = Math.Sqrt(ptN.X * ptN.X + ptN.Y * ptN.Y);
                double cVal = Math.Sqrt(Math.Pow(ptN.Z - a01length, 2) + Math.Pow(pVal - a01offset, 2));
                double epsilonAng = Math.Atan2(ptN.Z - a01length, pVal - a01offset);
                double bVal = a02length;
                double aVal = Math.Sqrt(a03offset * a03offset + a03length * a03length);
                double sVal = (aVal + bVal + cVal) / 2.0;
                double rVal = Math.Sqrt(((sVal - aVal) * (sVal - bVal) * (sVal - cVal)) / sVal);
                double alphaAng = 2 * Math.Atan(rVal / (sVal - aVal));
                double gammaAng = 2 * Math.Atan(rVal / (sVal - cVal));
                double phiAng = Math.Atan((-a03offset) / a03length);
                A02rot = (Math.PI / 2.0) + alphaAng - epsilonAng;
                A03rot = ((-Math.PI) / 2.0) + gammaAng - phiAng;
            }
            #endregion

            #region Handle Out-of-Range Condition
            if (double.IsNaN(A02rot))
            {
                if (statusBit0 == 1)
                {
                    A01rot += Math.PI;
                }
                outsideRange = true;
                A02rot = -90 * Math.PI / 180.0;
                A03rot = 90 * Math.PI / 180.0;
            }
            #endregion

            #region Define Robot Tip and Rotational Axes
            Frame robotTip = new Frame(
                new Vector3((float)(a03length + a01offset), 0f, (float)((a01length + a02length) - a03offset)),
                new Vector3(1, 0, 0),
                new Vector3(0, 1, 0));

            Line[] rotAxes = new Line[6];
            rotAxes[0] = new Line(new Vector3(0, 0, 0), new Vector3(0, 0, (float)a01length));
            rotAxes[1] = new Line(new Vector3((float)a01offset, 0, (float)a01length),
                                  new Vector3((float)a01offset, -1, (float)a01length));
            rotAxes[2] = new Line(new Vector3((float)a01offset, 0, (float)(a01length + a02length)),
                                  new Vector3((float)a01offset, -1, (float)(a01length + a02length)));
            rotAxes[3] = new Line(new Vector3((float)(a01offset + a03length), 0, (float)(a01length + a02length - a03offset)),
                                  new Vector3((float)(a01offset + a03length + a04length), 0, (float)(a01length + a02length - a03offset)));
            rotAxes[4] = new Line(new Vector3((float)(a01offset + a03length), 0, (float)(a01length + a02length - a03offset)),
                                  new Vector3((float)(a01offset + a03length), -1, (float)(a01length + a02length - a03offset)));
            rotAxes[5] = new Line(new Vector3((float)(a01offset + a03length + a04length), 0, (float)(a01length + a02length - a03offset)),
                                  new Vector3((float)(a01offset + a03length + a04length + 1.0), 0, (float)(a01length + a02length - a03offset)));
            #endregion

            #region Apply Joint Rotations to Robot Tip and Axes
            Matrix A01Trans = TransformUtil.Rotation((float)A01rot, rotAxes[0].Direction, rotAxes[0].From);
            robotTip.Transform(ref A01Trans);
            for (int i = 0; i < 6; i++)
            {
                rotAxes[i].Transform(ref A01Trans);
            }
            Matrix A02Trans = TransformUtil.Rotation((float)A02rot, rotAxes[1].Direction, rotAxes[1].From);
            robotTip.Transform(ref A02Trans);
            for (int i = 1; i < 6; i++)
            {
                rotAxes[i].Transform(ref A02Trans);
            }
            Matrix A03Trans = TransformUtil.Rotation((float)A03rot, rotAxes[2].Direction, rotAxes[2].From);
            robotTip.Transform(ref A03Trans);
            #endregion

            #region Compute Dot Products for Final Orientation
            double dot31 = Vector3.Dot(robotTip.XAxis, targetFrame.ZAxis);
            double dot32 = Vector3.Dot(robotTip.YAxis, targetFrame.ZAxis);
            double dot33 = Vector3.Dot(robotTip.ZAxis, targetFrame.ZAxis);
            double dot23 = Vector3.Dot(robotTip.ZAxis, targetFrame.YAxis);
            double dot13 = Vector3.Dot(robotTip.ZAxis, targetFrame.XAxis);
            double sinB = Math.Sqrt(dot31 * dot31 + dot32 * dot32);
            double alpha = Math.Atan2(dot23 / sinB, dot13 / sinB);
            double beta = Math.Atan2(Math.Sqrt(dot31 * dot31 + dot32 * dot32), dot33);
            double gamma = Math.Atan2(dot32 / sinB, dot31 / sinB);
            #endregion

            #region Final Joint Rotations (A04, A05, A06)
            A04rot = Math.PI + gamma;
            A05rot = beta;
            A06rot = -alpha;
            if (statusBit2 == 1)
            {
                A04rot = Math.PI + gamma;
                A05rot = beta;
                A06rot = -alpha;
            }
            else
            {
                A04rot = Math.PI + gamma + Math.PI;
                A05rot = -beta;
                A06rot = Math.PI - alpha;
            }
            A04rot *= -1.0;
            #endregion

            #region Convert Radians to Degrees and Normalise
            A01rot = A01rot / Math.PI * 180 / -1.0;
            A02rot = (A02rot / Math.PI * 180 / -1.0) - 90.0;
            A03rot = (A03rot / Math.PI * 180 / -1.0) + 90.0;
            A04rot = A04rot / Math.PI * 180;
            A05rot = A05rot / Math.PI * 180 / -1.0;
            A06rot = A06rot / Math.PI * 180 / -1.0;
            if (A01rot > 170)
            {
                if (A01rot > 180)
                {
                    A01rot -= 360;
                }
            }
            else if (A01rot < -170)
            {
                if (A01rot < -180)
                {
                    A01rot += 360;
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
            if (outsideRange)
            {
                A04rot = 0.0;
                A05rot = 0.00000001;
                A06rot = 0.0;
            }
            #endregion

            return IK_Solver_EqualiseKinematics(prevAxVals, new IK_Solver_AxVals(A01rot, A02rot, A03rot, A04rot, A05rot, A06rot));
        }

        private static IK_Solver_AxVals IK_Solver_EqualiseKinematics(IK_Solver_AxVals oldVals, IK_Solver_AxVals newVals)
        {
            return new IK_Solver_AxVals(
                IK_Solver_AdjustAxval(newVals.a01_val, oldVals.a01_val, 1),
                newVals.a02_val,
                newVals.a03_val,
                IK_Solver_AdjustAxval(newVals.a04_val, oldVals.a04_val, 1),
                newVals.a05_val,
                IK_Solver_AdjustAxval(newVals.a06_val, oldVals.a06_val, 1));
        }

        private static double IK_Solver_AdjustAxval(double currentAxVal, double prevAxVal, int type)
        {
            if (type == 1)
            {
                if (Math.Abs(Math.Abs(currentAxVal) - Math.Abs(prevAxVal)) == 180)
                {
                    return currentAxVal;
                }
                double prevAbsFactor = Math.Floor(prevAxVal / 360.0);
                double prevAbs = prevAxVal - (prevAbsFactor * 360);
                double curAbsFactor = Math.Floor(currentAxVal / 360.0);
                double curAbs = currentAxVal - (curAbsFactor * 360);
                double newAxVal = prevAbsFactor * 360 + curAbs;
                if (Math.Abs(newAxVal - prevAxVal) > 180)
                {
                    double inverseVal = 0;
                    if (newAxVal > 0)
                    {
                        inverseVal = newAxVal - 360;
                    }
                    else if (newAxVal < 0)
                    {
                        inverseVal = newAxVal + 360;
                    }
                    newAxVal = inverseVal;
                    if (Math.Abs(newAxVal - prevAxVal) > 180)
                    {
                        if (prevAbsFactor > 0)
                        {
                            newAxVal = (prevAbsFactor + 1) * 360 + curAbs;
                        }
                        else if (prevAbsFactor < 0)
                        {
                            newAxVal = (prevAbsFactor - 1) * 360 + curAbs;
                        }
                        else
                        {
                            newAxVal = 360 + curAbs;
                        }
                    }
                }
                return newAxVal;
            }
            else
            {
                return currentAxVal;
            }
        }
    }

    // The following classes remain largely unchanged.
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
            : this(a01_val, a02_val, a03_val, a04_val, a05_val, a06_val, 0, 0, 0, 0, 0)
        { }

        public IK_Solver_AxVals() { }

        public IK_Solver_AxVals(IK_Solver_AxVals toClone)
        {
            a01_val = toClone.a01_val;
            a02_val = toClone.a02_val;
            a03_val = toClone.a03_val;
            a04_val = toClone.a04_val;
            a05_val = toClone.a05_val;
            a06_val = toClone.a06_val;
            e01_val = toClone.e01_val;
            e02_val = toClone.e02_val;
            e03_val = toClone.e03_val;
            e04_val = toClone.e04_val;
            index_val = toClone.index_val;
        }

        public IK_Solver_AxVals(List<double> axVals)
        {
            if (axVals.Count == 10)
            {
                a01_val = axVals[0];
                a02_val = axVals[1];
                a03_val = axVals[2];
                a04_val = axVals[3];
                a05_val = axVals[4];
                a06_val = axVals[5];
                e01_val = axVals[6];
                e02_val = axVals[7];
                e03_val = axVals[8];
                e04_val = axVals[9];
            }
            else if (axVals.Count == 6)
            {
                a01_val = axVals[0];
                a02_val = axVals[1];
                a03_val = axVals[2];
                a04_val = axVals[3];
                a05_val = axVals[4];
                a06_val = axVals[5];
                e01_val = 0;
                e02_val = 0;
                e03_val = 0;
                e04_val = 0;
            }
        }

        public List<double> ToList()
        {
            return new List<double> { a01_val, a02_val, a03_val, a04_val, a05_val, a06_val, e01_val, e02_val, e03_val, e04_val };
        }

        public override string ToString()
        {
            string retStr = $"A1 {a01_val}, A2 {a02_val}, A3 {a03_val}, A4 {a04_val}, A5 {a05_val}, A6 {a06_val}";
            if (!double.IsNaN(e01_val))
            {
                retStr += $", E1 {e01_val}";
            }
            if (!double.IsNaN(e02_val))
            {
                retStr += $", E2 {e02_val}";
            }
            if (!double.IsNaN(e03_val))
            {
                retStr += $", E3 {e03_val}";
            }
            if (!double.IsNaN(e04_val))
            {
                retStr += $", E4 {e04_val}";
            }
            return retStr;
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
        public double basevalA = 0;
        public double basevalB = 0;
        public double basevalC = 0;
        public double basevalX = 0;
        public double basevalY = 0;
        public double basevalZ = 0;

        public IK_Solver_RobotBase() { }

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

        public IK_Solver_ToolMini() { }

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

    /// <summary>
    /// Provides a helper method for converting degrees to radians.
    /// </summary>
    public static class MathUtil
    {
        public static float DegreesToRadians(double degrees)
        {
            return (float)(Math.PI * degrees / 180.0);
        }
    }
}
