using System;
using System.Collections.Generic;
using Bunraku.Geometry;

namespace InverseKinematics
{
    public class InverseKinematicSolver
    {
        public PRC_RobotMini Robot { get; set; } = PRC_RobotMini.PRC_KR610R9002();
        public PRC_RobotBase BaseConfig { get; set; } = new PRC_RobotBase();
        public PRC_ToolMini Tool { get; set; } = new PRC_ToolMini();
        public PlaneB TargetPlaneBB { get; set; } = new PlaneB(new Point3d(0, 0, 0), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));
        public string Status { get; set; } = "010";
        public PRC_AxVals PreviousAxes { get; set; } = null;

        public PRC_AxVals JointAngles { get; private set; }

        public void Update()
        {
            JointAngles = Solve(Robot, BaseConfig, Tool, ValidateTargetPlaneBBB(TargetPlaneBB), Status, PreviousAxes);
        }

        private static PRC_AxVals Solve(PRC_RobotMini robot, PRC_RobotBase baseConfig, PRC_ToolMini tool, PlaneB targetPlaneBBB, string status, PRC_AxVals previousAxes = null)
        {
            previousAxes ??= new PRC_AxVals(0, -90, 90, 0, 0, 0);

            var toolPlaneBBB = AdjustToolPlaneBBB(tool);
            var basePlaneBBB = AdjustBasePlaneBBB(baseConfig);
            var transformedTarget = TransformTargetPlaneBBB(targetPlaneBBB, toolPlaneBBB, basePlaneBBB);

            var (a01, a02, a03) = SolveFirstThreeAxes(robot, transformedTarget, status);
            var (a04, a05, a06) = SolveLastThreeAxes(transformedTarget);

            if (double.IsNaN(a02) || double.IsNaN(a03))
            {
                return HandleOutOfRange(previousAxes);
            }

            var result = ConvertAndNormaliseAngles(a01, a02, a03, a04, a05, a06);

            return PRC_EqualiseKinematics(previousAxes, result);
        }

        private static PlaneB AdjustToolPlaneBBB(PRC_ToolMini tool)
        {
            var toolPlaneBBB = new PlaneB(new Point3d(tool.toolvalX, tool.toolvalY, tool.toolvalZ),
                                      new Vector3d(1, 0, 0),
                                      new Vector3d(0, 1, 0));
            toolPlaneBBB.Transform(Transform.Rotation(Math.PI * tool.toolvalA / 180.0, toolPlaneBBB.ZAxis, toolPlaneBBB.Origin));
            toolPlaneBBB.Transform(Transform.Rotation(Math.PI * tool.toolvalB / 180.0, toolPlaneBBB.YAxis, toolPlaneBBB.Origin));
            toolPlaneBBB.Transform(Transform.Rotation(Math.PI * tool.toolvalC / 180.0, toolPlaneBBB.XAxis, toolPlaneBBB.Origin));
            return toolPlaneBBB;
        }

        private static PlaneB AdjustBasePlaneBBB(PRC_RobotBase baseConfig)
        {
            var basePlaneBBB = new PlaneB(new Point3d(baseConfig.basevalX, baseConfig.basevalY, baseConfig.basevalZ),
                                      new Vector3d(1, 0, 0),
                                      new Vector3d(0, 1, 0));
            basePlaneBBB.Transform(Transform.Rotation(Math.PI * baseConfig.basevalA / 180.0, basePlaneBBB.ZAxis, basePlaneBBB.Origin));
            basePlaneBBB.Transform(Transform.Rotation(Math.PI * baseConfig.basevalB / 180.0, basePlaneBBB.YAxis, basePlaneBBB.Origin));
            basePlaneBBB.Transform(Transform.Rotation(Math.PI * baseConfig.basevalC / 180.0, basePlaneBBB.XAxis, basePlaneBBB.Origin));
            return basePlaneBBB;
        }

        private static PlaneB TransformTargetPlaneBBB(PlaneB targetPlaneBBB, PlaneB toolPlaneBBB, PlaneB basePlaneBBB)
        {
            var shiftedTool = Transform.PlaneBToPlaneB(toolPlaneBBB, targetPlaneBBB);
            toolPlaneBBB.Transform(shiftedTool);

            var translation = Transform.Translation(new Vector3d(targetPlaneBBB.Origin.X - toolPlaneBBB.Origin.X,
                                                                  targetPlaneBBB.Origin.Y - toolPlaneBBB.Origin.Y,
                                                                  targetPlaneBBB.Origin.Z - toolPlaneBBB.Origin.Z));
            toolPlaneBBB.Transform(translation);

            toolPlaneBBB.Transform(Transform.ChangeBasis(basePlaneBBB, new PlaneB(new Point3d(0, 0, 0), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0))));

            return toolPlaneBBB;
        }

        private static (double, double, double) SolveFirstThreeAxes(PRC_RobotMini robot, PlaneB targetPlaneBBB, string status)
        {
            var statusBits = Array.ConvertAll(status.ToCharArray(), bit => int.Parse(bit.ToString()));
            var offsetSign = statusBits[1] == 1 ? 1 : -1;
            var a01 = Math.Atan2(targetPlaneBBB.Origin.Y, targetPlaneBBB.Origin.X) + (statusBits[0] == 1 ? Math.PI : 0);

            var pVal = Math.Sqrt(Math.Pow(targetPlaneBBB.Origin.X, 2) + Math.Pow(targetPlaneBBB.Origin.Y, 2));
            var cVal = Math.Sqrt(Math.Pow(targetPlaneBBB.Origin.Z - robot.a01length, 2) + Math.Pow(pVal + offsetSign * robot.a01offset, 2));

            if (pVal < 1e-6 || cVal < 1e-6)
                throw new InvalidOperationException("Invalid target position: potential singularity or unreachable point.");

            var epsilon = Math.Atan2(targetPlaneBBB.Origin.Z - robot.a01length, pVal + offsetSign * robot.a01offset);

            var a02 = CalculateA02(robot, cVal, epsilon);
            var a03 = CalculateA03(robot, cVal);

            return (a01, a02, a03);
        }

        private static (double, double, double) SolveLastThreeAxes(PlaneB targetPlaneBBB)
        {
            var dot31 = targetPlaneBBB.XAxis.Dot(targetPlaneBBB.ZAxis);
            var dot32 = targetPlaneBBB.YAxis.Dot(targetPlaneBBB.ZAxis);
            var dot33 = targetPlaneBBB.ZAxis.Dot(targetPlaneBBB.ZAxis);
            var dot23 = targetPlaneBBB.ZAxis.Dot(targetPlaneBBB.YAxis);
            var dot13 = targetPlaneBBB.ZAxis.Dot(targetPlaneBBB.XAxis);

            var sinB = Math.Sqrt(dot31 * dot31 + dot32 * dot32);
            if (sinB < 1e-6)
                throw new InvalidOperationException("Invalid orientation: degenerate PlaneBBB configuration.");

            var alpha = Math.Atan2(dot23 / sinB, dot13 / sinB);
            var beta = Math.Atan2(sinB, dot33);
            var gamma = Math.Atan2(dot32 / sinB, dot31 / sinB);

            return (gamma, beta, -alpha);
        }

        private static double CalculateA02(PRC_RobotMini robot, double cVal, double epsilon)
        {
            var bVal = robot.a02length;
            var aVal = Math.Sqrt(Math.Pow(robot.a03offset, 2) + Math.Pow(robot.a03length, 2));
            var sVal = (aVal + bVal + cVal) / 2;
            var rVal = Math.Sqrt((sVal - aVal) * (sVal - bVal) * (sVal - cVal) / sVal);
            var alpha = 2 * Math.Atan(rVal / (sVal - aVal));
            return alpha + epsilon - Math.PI / 2;
        }

        private static double CalculateA03(PRC_RobotMini robot, double cVal)
        {
            var bVal = robot.a02length;
            var aVal = Math.Sqrt(Math.Pow(robot.a03offset, 2) + Math.Pow(robot.a03length, 2));
            var sVal = (aVal + bVal + cVal) / 2;
            var rVal = Math.Sqrt((sVal - aVal) * (sVal - bVal) * (sVal - cVal) / sVal);
            var gamma = 2 * Math.Atan(rVal / (sVal - cVal));
            return gamma + Math.Atan(robot.a03offset / robot.a03length) - Math.PI / 2;
        }

        private static PRC_AxVals ConvertAndNormaliseAngles(double a01, double a02, double a03, double a04, double a05, double a06)
        {
            double NormalizeAngle(double angle)
            {
                if (double.IsNaN(angle)) return 0;
                return angle * 180 / Math.PI;
            }

            return new PRC_AxVals(
                NormalizeAngle(a01),
                NormalizeAngle(a02 - Math.PI / 2),
                NormalizeAngle(a03 + Math.PI / 2),
                NormalizeAngle(a04),
                NormalizeAngle(a05),
                NormalizeAngle(a06)
            );
        }

        private static PRC_AxVals HandleOutOfRange(PRC_AxVals previousAxes)
        {
            return new PRC_AxVals(previousAxes)
            {
                a02_val = -90,
                a03_val = 90
            };
        }

        private static PRC_AxVals PRC_EqualiseKinematics(PRC_AxVals oldVals, PRC_AxVals newVals)
        {
            return new PRC_AxVals(
                AdjustAngle(newVals.a01_val, oldVals.a01_val),
                newVals.a02_val,
                newVals.a03_val,
                AdjustAngle(newVals.a04_val, oldVals.a04_val),
                newVals.a05_val,
                AdjustAngle(newVals.a06_val, oldVals.a06_val)
            );
        }

        private static double AdjustAngle(double current, double previous)
        {
            var diff = current - previous;
            if (diff > 180) current -= 360;
            if (diff < -180) current += 360;
            return current;
        }

        private static PlaneB ValidateTargetPlaneBBB(PlaneB targetPlaneBBB)
        {
            try
            {
                return new PlaneB(targetPlaneBBB.Origin, targetPlaneBBB.XAxis, targetPlaneBBB.YAxis);
            }
            catch
            {
                return new PlaneB(new Point3d(0, 0, 0), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));
            }
        }
    }

    public class PRC_RobotMini
    {
        public double a01length { get; set; }
        public double a01offset { get; set; }
        public double a02length { get; set; }
        public double a03length { get; set; }
        public double a03offset { get; set; }

        public PRC_RobotMini(double a01length, double a01offset, double a02length, double a03length, double a03offset)
        {
            this.a01length = a01length;
            this.a01offset = a01offset;
            this.a02length = a02length;
            this.a03length = a03length;
            this.a03offset = a03offset;
        }

        public static PRC_RobotMini PRC_KR610R9002()
        {
            return new PRC_RobotMini(400, 25, 455, 420, 25);
        }
    }

    public class PRC_RobotBase
    {
        public double basevalX { get; set; } = 0;
        public double basevalY { get; set; } = 0;
        public double basevalZ { get; set; } = 0;
        public double basevalA { get; set; } = 0;
        public double basevalB { get; set; } = 0;
        public double basevalC { get; set; } = 0;
    }

    public class PRC_ToolMini
    {
        public double toolvalX { get; set; } = 0;
        public double toolvalY { get; set; } = 0;
        public double toolvalZ { get; set; } = 0;
        public double toolvalA { get; set; } = 0;
        public double toolvalB { get; set; } = 0;
        public double toolvalC { get; set; } = 0;
    }

    public class PRC_AxVals
    {
        public double a01_val { get; set; }
        public double a02_val { get; set; }
        public double a03_val { get; set; }
        public double a04_val { get; set; }
        public double a05_val { get; set; }
        public double a06_val { get; set; }

        public PRC_AxVals(double a01_val, double a02_val, double a03_val, double a04_val, double a05_val, double a06_val)
        {
            this.a01_val = a01_val;
            this.a02_val = a02_val;
            this.a03_val = a03_val;
            this.a04_val = a04_val;
            this.a05_val = a05_val;
            this.a06_val = a06_val;
        }

        public PRC_AxVals(PRC_AxVals toClone)
        {
            this.a01_val = toClone.a01_val;
            this.a02_val = toClone.a02_val;
            this.a03_val = toClone.a03_val;
            this.a04_val = toClone.a04_val;
            this.a05_val = toClone.a05_val;
            this.a06_val = toClone.a06_val;
        }
    }
}
