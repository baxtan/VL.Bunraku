using System;
using Stride.Core.Mathematics;
using SolverCoreMini.Geometry;
using static VL.Core.AppHost;
using Frame = SolverCoreMini.Geometry.Frame;

namespace SolverCoreMini
{
    /// <summary>
    /// Inverse kinematics solver for a 6-axis robot arm.
    /// </summary>
    public static class InverseKinematic
    {
        public static IK_Solver_AxVals IK_Solver_InverseKinematic(
            IK_Solver_RobotMini robot,
            IK_Solver_RobotBase robotBase,
            IK_Solver_ToolMini tool,
            Frame targetFrame,
            string status,
            IK_Solver_AxVals prevAxVals)
        {
            // --- validation & continuity seed ---
            if (robot is null) throw new ArgumentNullException(nameof(robot));
            if (robotBase is null) throw new ArgumentNullException(nameof(robotBase));
            if (tool is null) throw new ArgumentNullException(nameof(tool));
            if (string.IsNullOrEmpty(status))
                throw new ArgumentNullException(nameof(status));
            if (status.Length < 3)
                throw new ArgumentException("Status must be at least 3 characters", nameof(status));

            prevAxVals ??= new IK_Solver_AxVals(0, -90, 90, 0, 0, 0);

            // --- link lengths & signed offsets ---
            double a01len = robot.a01length;
            double a01off = -robot.a01offset;   // note leading minus
            double a02len = robot.a02length;
            double a03len = robot.a03length;
            double a03off = -robot.a03offset;   // leading minus
            double a04len = robot.a04length;

            // --- WORLD FRAME ---
            Frame worldFrame = new Frame(Vector3.Zero, Vector3.UnitX, Vector3.UnitY);

            // --- BASE FRAME: rotate Z→Y→X in degrees, *without* reassigning ---
            Frame baseFrame = new Frame(
                new Vector3((float)robotBase.basevalX, (float)robotBase.basevalY, (float)robotBase.basevalZ),
                Vector3.UnitX, Vector3.UnitY
            );
            {
                float a = MathUtil.DegreesToRadians(robotBase.basevalA),
                      b = MathUtil.DegreesToRadians(robotBase.basevalB),
                      c = MathUtil.DegreesToRadians(robotBase.basevalC);

                var rz = TransformUtil.Rotation(a, baseFrame.ZAxis, baseFrame.Origin);
                baseFrame.Transform(ref rz);
                var ry = TransformUtil.Rotation(b, baseFrame.YAxis, baseFrame.Origin);
                baseFrame.Transform(ref ry);
                var rx = TransformUtil.Rotation(c, baseFrame.XAxis, baseFrame.Origin);
                baseFrame.Transform(ref rx);
            }

            // --- TOOL FRAME: rotate + then reassign axes as in original ---
            Frame toolFrame = new Frame(
                new Vector3((float)tool.toolvalX, (float)tool.toolvalY, (float)tool.toolvalZ),
                Vector3.UnitX, Vector3.UnitY
            );
            {
                float a = MathUtil.DegreesToRadians(tool.toolvalA),
                      b = MathUtil.DegreesToRadians(tool.toolvalB),
                      c = MathUtil.DegreesToRadians(tool.toolvalC);

                var rz = TransformUtil.Rotation(a, toolFrame.ZAxis, toolFrame.Origin);
                toolFrame.Transform(ref rz);
                var ry = TransformUtil.Rotation(b, toolFrame.YAxis, toolFrame.Origin);
                toolFrame.Transform(ref ry);
                var rx = TransformUtil.Rotation(c, toolFrame.XAxis, toolFrame.Origin);
                toolFrame.Transform(ref rx);

                // exactly as in your reference
                toolFrame = new Frame(toolFrame.Origin, toolFrame.ZAxis, toolFrame.YAxis);
            }

            // --- APPLY TOOL → TARGET → BASE transforms to get the joint-3 frame ---
            var a456 = new Frame(new Vector3(0, 0, (float)-a04len), Vector3.UnitX, Vector3.UnitY);

            Matrix m1 = TransformUtil.PlaneToPlane(
                toolFrame,
                new Frame(toolFrame.Origin, targetFrame.XAxis, targetFrame.YAxis)
            );
            a456.Transform(ref m1);

            Matrix m2 = TransformUtil.Translation(targetFrame.Origin - toolFrame.Origin);
            a456.Transform(ref m2);

            Matrix m3 = TransformUtil.ChangeBasis(baseFrame, worldFrame);
            a456.Transform(ref m3);

            targetFrame = a456;

            // --- parse status bits C,B,A ---
            int bitC = status[0] - '0';
            int bitB = status[1] - '0';
            int bitA = status[2] - '0';

            // --- precompute ε, α, γ, φ for the planar elbow triangle ---
            Vector3 pt = targetFrame.Origin;
            (double eps, double alpha, double gamma, double phi) =
                ComputeElbowParams(pt, a01len, a01off, a02len, a03len, a03off, bitA);

            // --- SHOULDER A1 & ELBOW A2,A3 via tuple-switch ---
            double A01 = Math.Atan2(pt.Y, pt.X) + (bitA == 1 ? Math.PI : 0);
            double A02 = 0, A03 = 0;

            switch ((bitB, bitA))
            {
                case (1, 0): // elbow-up / elbow-down variants
                    A02 = alpha + eps - Math.PI / 2;
                    A03 = gamma + phi - Math.PI / 2;
                    break;
                case (0, 0):
                    A02 = 1.5 * Math.PI - alpha + eps;
                    A03 = 1.5 * Math.PI - gamma + phi;
                    break;
                case (0, 1):
                    A02 = Math.PI / 2 - (alpha + eps);
                    A03 = 1.5 * Math.PI - gamma + phi;
                    break;
                case (1, 1):
                    A02 = Math.PI / 2 + alpha - eps;
                    A03 = -Math.PI / 2 + gamma + phi;
                    break;
            }

            bool outsideRange = false;
            if (double.IsNaN(A02))
            {
                outsideRange = true;
                if (bitA == 1) A01 += Math.PI;
                A02 = -Math.PI / 2;
                A03 = Math.PI / 2;
            }

            // --- clamp A2 into [–270°, +270°] exactly as original ---
            double A02deg = -A02 * 180.0 / Math.PI - 90.0;
            if (A02deg < -270) A02deg += 360;
            else if (A02deg > 270) A02deg -= 360;

            // --- build robot tip and rotate joints 1–3 ---
            Frame tip = new Frame(
                new Vector3((float)(a03len - a01off), 0, (float)(a01len + a02len - a03off)),
                new Vector3(0, 0, -1),
                Vector3.UnitY
            );
            var axes = GenerateAxes(a01len, a01off, a02len, a03len, a03off, a04len);
            ApplyJointRotation(ref tip, axes, 0, (float)A01);
            ApplyJointRotation(ref tip, axes, 1, (float)(A02));
            ApplyJointRotation(ref tip, axes, 2, (float)(A03));

            // --- WRIST A4,A5,A6 via dot-products ---
            (double a4o, double b5o, double g6o) = ComputeOrientation(tip, targetFrame);

            double A04 = -((bitC == 1 ? Math.PI + g6o : 2 * Math.PI + g6o));
            double A05 = (bitC == 1 ? b5o : -b5o);
            double A06 = (bitC == 1 ? -a4o : Math.PI - a4o);

            // --- convert to degrees & normalise/flip signs as original ---
            double A01deg = NormalizeAngle(-A01 * 180.0 / Math.PI);
            double A03deg = NormalizeAngle(-A03 * 180.0 / Math.PI + 90.0);
            double A04deg = A04 * 180.0 / Math.PI;
            double A05deg = NormalizeAngle(-A05 * 180.0 / Math.PI);
            double A06deg = NormalizeAngle(-A06 * 180.0 / Math.PI);

            // use clamped A02deg
            if (outsideRange)
            {
                A04deg = 0.0;
                A05deg = 1e-8;
                A06deg = 0.0;
            }

            // --- equalise for minimal joint motion on A1,A4,A6 ---
            return IK_Solver_EqualiseKinematics(
                prevAxVals,
                new IK_Solver_AxVals(
                    A01deg,
                    A02deg,
                    A03deg,
                    A04deg,
                    A05deg,
                    A06deg
                )
            );
        }

        // ————————————————— helper methods ——————————————————————————

        private static (double eps, double alpha, double gamma, double phi)
        ComputeElbowParams(
            Vector3 pt,
            double a01len,
            double a01off,
            double a02len,
            double a03len,
            double a03off,
            int statusBit0)
        {
            double p = Math.Sqrt(pt.X * pt.X + pt.Y * pt.Y);
            double adj = p + (statusBit0 == 0 ? a01off : -a01off);
            double c = Math.Sqrt((pt.Z - a01len) * (pt.Z - a01len) + adj * adj);
            double eps = Math.Atan2(pt.Z - a01len, adj);
            double aVal = Math.Sqrt(a03off * a03off + a03len * a03len);
            double s = (aVal + a02len + c) / 2.0;
            double r = Math.Sqrt(((s - aVal) * (s - a02len) * (s - c)) / s);
            double alpha = 2.0 * Math.Atan(r / (s - aVal));
            double gamma = 2.0 * Math.Atan(r / (s - c));
            double phi = Math.Atan(a03off / a03len);
            return (eps, alpha, gamma, phi);
        }

        private static void ApplyJointRotation(ref Frame tip, Line[] axes, int idx, float ang)
        {
            var m = TransformUtil.Rotation(ang, axes[idx].Direction, axes[idx].From);
            tip.Transform(ref m);
            for (int i = idx; i < axes.Length; i++)
                axes[i].Transform(ref m);
        }

        private static (double a4, double b5, double g6) ComputeOrientation(Frame src, Frame tgt)
        {
            double d31 = Vector3.Dot(src.XAxis, tgt.ZAxis),
                   d32 = Vector3.Dot(src.YAxis, tgt.ZAxis),
                   d33 = Vector3.Dot(src.ZAxis, tgt.ZAxis),
                   d23 = Vector3.Dot(src.ZAxis, tgt.YAxis),
                   d13 = Vector3.Dot(src.ZAxis, tgt.XAxis);
            double sinB = Math.Sqrt(d31 * d31 + d32 * d32);
            return (
                Math.Atan2(d23 / sinB, d13 / sinB),
                Math.Atan2(sinB, d33),
                Math.Atan2(d32 / sinB, d31 / sinB)
            );
        }

        private static Line[] GenerateAxes(
            double a1, double off1,
            double a2,
            double a3, double off3,
            double a4)
        {
            return new[]
            {
                new Line(Vector3.Zero, new Vector3(0,0,(float)a1)),
                new Line(new Vector3((float)off1,0,(float)a1), new Vector3((float)off1,-1,(float)a1)),
                new Line(new Vector3((float)off1,0,(float)(a1+a2)), new Vector3((float)off1,-1,(float)(a1+a2))),
                new Line(new Vector3((float)(off1+a3),0,(float)(a1+a2-off3)),
                         new Vector3((float)(off1+a3+a4),0,(float)(a1+a2-off3))),
                new Line(new Vector3((float)(off1+a3),0,(float)(a1+a2-off3)),
                         new Vector3((float)(off1+a3),-1,(float)(a1+a2-off3))),
                new Line(new Vector3((float)(off1+a3+a4),0,(float)(a1+a2-off3)),
                         new Vector3((float)(off1+a3+a4+1),0,(float)(a1+a2-off3)))
            };
        }

        private static IK_Solver_AxVals IK_Solver_EqualiseKinematics(
            IK_Solver_AxVals oldV,
            IK_Solver_AxVals newV)
        {
            return new IK_Solver_AxVals(
                NormalizeAxis(newV.a01_val, oldV.a01_val),
                newV.a02_val,
                newV.a03_val,
                NormalizeAxis(newV.a04_val, oldV.a04_val),
                newV.a05_val,
                NormalizeAxis(newV.a06_val, oldV.a06_val)
            );
        }

        private static double NormalizeAxis(double cur, double prev)
        {
            double d = (cur - prev + 180) % 360 - 180;
            return prev + (d < -180 ? d + 360 : d > 180 ? d - 360 : d);
        }

        private static double NormalizeAngle(double angle)
        {
            return ((angle + 180) % 360 + 360) % 360 - 180;
        }
    }

    // ————————————————— data classes ——————————————————————————————

    public class IK_Solver_AxVals
    {
        public double a01_val, a02_val, a03_val, a04_val, a05_val, a06_val;
        public IK_Solver_AxVals(double a1, double a2, double a3, double a4, double a5, double a6)
        {
            a01_val = a1; a02_val = a2; a03_val = a3; a04_val = a4; a05_val = a5; a06_val = a6;
        }
    }

    public class IK_Solver_RobotMini
    {
        public double a01length = 400, a01offset = 25,
                      a02length = 455,
                      a03length = 420, a03offset = 25,
                      a04length = 90;
        public IK_Solver_RobotMini(double l1, double o1, double l2, double l3, double o3, double l4)
        {
            a01length = l1; a01offset = o1;
            a02length = l2;
            a03length = l3; a03offset = o3;
            a04length = l4;
        }
    }

    public class IK_Solver_RobotBase
    {
        public double basevalA, basevalB, basevalC, basevalX, basevalY, basevalZ;
        public IK_Solver_RobotBase() { }
        public IK_Solver_RobotBase(double A, double B, double C, double X, double Y, double Z)
        {
            basevalA = A; basevalB = B; basevalC = C;
            basevalX = X; basevalY = Y; basevalZ = Z;
        }
    }

    public class IK_Solver_ToolMini
    {
        public double toolvalX, toolvalY, toolvalZ, toolvalA, toolvalB, toolvalC;
        public IK_Solver_ToolMini() { }
        public IK_Solver_ToolMini(double x, double y, double z, double a, double b, double c)
        {
            toolvalX = x; toolvalY = y; toolvalZ = z;
            toolvalA = a; toolvalB = b; toolvalC = c;
        }
    }

    public static class MathUtil
    {
        public static float DegreesToRadians(double deg)
            => (float)(Math.PI * deg / 180.0);
    }
}
