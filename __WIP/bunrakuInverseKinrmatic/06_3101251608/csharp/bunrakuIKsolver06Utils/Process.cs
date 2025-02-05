// Fixed Process.cs errors and aligned with InverseKinematic.cs
using Bunraku.Geometry;

namespace Main;

[ProcessNode]
public class InverseKinematic
{
    public static PRC_AxVals PRC_InverseKinematic(PRC_RobotMini robot, PRC_RobotBase baseConfig, PRC_ToolMini tool, Bunraku.Geometry.Plane targetPlane, string status, PRC_AxVals prevAxVals)
    {
        double a01length = robot.a01length;
        double a01offset = robot.a01offset * (-1.0);
        double a02length = robot.a02length;
        double a03length = robot.a03length;
        double a03offset = robot.a03offset * (-1.0);
        double a04length = robot.a04length;

        if (prevAxVals == null)
        {
            prevAxVals = new PRC_AxVals(0, -90, 90, 0, 0, 0, new Point3d(0, 0, 0), new Vector3d(0, 0, 0), new Vector3d(0, 0, 0), new Vector3d(0, 0, 0), new Point3d(0, 0, 0), new Vector3d(0, 0, 0), new Vector3d(0, 0, 0), new Vector3d(0, 0, 0), new Vector3d(0, 0, 0), new Vector3d(0, 0, 0), new Vector3d(0, 0, 0));
        }

        // Parse status input
        char[] statusbitchars = status.ToCharArray();
        int statusbit0 = Convert.ToInt32(statusbitchars[2].ToString());
        int statusbit1 = Convert.ToInt32(statusbitchars[1].ToString());
        int statusbit2 = Convert.ToInt32(statusbitchars[0].ToString());

        // Base transformation




        Bunraku.Geometry.Plane toolPlane = new Bunraku.Geometry.Plane(new Point3d(tool.toolvalX, tool.toolvalY, tool.toolvalZ), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));
        toolPlane.Transform(Transform.Translation(new Vector3d(tool.toolvalX, tool.toolvalY, tool.toolvalZ)) *
            Transform.Rotation(tool.toolvalA * Math.PI / 180.0, new Vector3d(0, 0, 1), toolPlane.Origin) *
            Transform.Rotation(tool.toolvalB * Math.PI / 180.0, new Vector3d(0, 1, 0), toolPlane.Origin) *
            Transform.Rotation(tool.toolvalC * Math.PI / 180.0, new Vector3d(1, 0, 0), toolPlane.Origin));

        Bunraku.Geometry.Plane basePlane = new Bunraku.Geometry.Plane(new Point3d(baseConfig.baseX, baseConfig.baseY, baseConfig.baseZ), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));




        basePlane.Transform(Transform.Rotation(baseConfig.baseA * Math.PI / 180.0, basePlane.ZAxis, basePlane.Origin));
        basePlane.Transform(Transform.Rotation(baseConfig.baseB * Math.PI / 180.0, basePlane.YAxis, basePlane.Origin));
        basePlane.Transform(Transform.Rotation(baseConfig.baseC * Math.PI / 180.0, basePlane.XAxis, basePlane.Origin));

        Transform baseTransform = Transform.Translation(new Vector3d(basePlane.Origin.X, basePlane.Origin.Y, basePlane.Origin.Z)) *
Transform.Rotation(baseConfig.baseA * Math.PI / 180.0, new Vector3d(0, 0, 1), basePlane.Origin) *
Transform.Rotation(baseConfig.baseB * Math.PI / 180.0, new Vector3d(0, 1, 0), basePlane.Origin) *
Transform.Rotation(baseConfig.baseC * Math.PI / 180.0, new Vector3d(1, 0, 0), basePlane.Origin);
        Transform toolTransform = Transform.Translation(new Vector3d(toolPlane.Origin.X, toolPlane.Origin.Y, toolPlane.Origin.Z)) *
            Transform.Rotation(tool.toolvalA * Math.PI / 180.0, new Vector3d(0, 0, 1), toolPlane.Origin) *
            Transform.Rotation(tool.toolvalB * Math.PI / 180.0, new Vector3d(0, 1, 0), toolPlane.Origin) *
            Transform.Rotation(tool.toolvalC * Math.PI / 180.0, new Vector3d(1, 0, 0), toolPlane.Origin);

        // Apply Tool Transformation FIRST
        targetPlane.Transform(toolTransform);

        // Apply Base Transformation AFTER
        targetPlane.Transform(baseTransform);








        // Solve joint angles




        double A01rot = Math.Atan2(targetPlane.Origin.Y, targetPlane.Origin.X); // Adjusted base rotation
        double pval = Math.Sqrt(targetPlane.Origin.X * targetPlane.Origin.X + targetPlane.Origin.Y * targetPlane.Origin.Y);
        double cval = Math.Sqrt(Math.Pow(targetPlane.Origin.Z - a01length, 2) + Math.Pow(pval + a01offset, 2));
        double epsilon = Math.Atan2(targetPlane.Origin.Z - a01length, pval + a01offset);
        double alpha = Math.Acos((a02length * a02length + cval * cval - a03length * a03length) / (2 * a02length * cval));
        double gammaCos = (a02length * a02length + a03length * a03length - cval * cval) / (2 * a02length * a03length);
        gammaCos = Math.Max(-1.0, Math.Min(1.0, gammaCos)); // Clamp value
        double gamma = Math.Acos(gammaCos);
        double phi = Math.Atan(a03offset / a03length);

        double A02rot, A03rot;
        if (statusbit1 == 1 && statusbit0 == 0)
        {
            A02rot = alpha + epsilon - Math.PI / 2.0;
            A03rot = gamma + phi - Math.PI / 2.0; // Adjusted sign
        }
        else if (statusbit1 == 0 && statusbit0 == 0)
        {
            A02rot = 1.5 * Math.PI - alpha + epsilon;
            A03rot = 1.5 * Math.PI - gamma + phi;
        }
        else if (statusbit1 == 0 && statusbit0 == 1)
        {
            A01rot += Math.PI;
            A02rot = Math.PI / 2.0 - (alpha + epsilon);
            A03rot = 1.5 * Math.PI - gamma - phi;
        }
        else
        {
            A01rot += Math.PI;
            A02rot = Math.PI / 2.0 + alpha - epsilon;
            A03rot = -Math.PI / 2.0 + gamma - phi;
        }

        // Solve wrist angles (A04, A05, A06)













        Vector3d robottipZ = targetPlane.ZAxis;
        Vector3d robottipX = targetPlane.XAxis;
        Vector3d robottipY = targetPlane.YAxis;

        double A04rot = Math.Atan2(robottipY.X, robottipX.X); // Adjusted quadrant
        double A05rot = Math.Acos(Math.Max(-1.0, Math.Min(1.0, robottipZ.Z))); // Clamped acos
        double A06rot = Math.Atan2(-robottipX.Y, robottipX.X); // Adjusted quadrant

        // Adjust angles based on status bit 2
        if (statusbit2 == 0)
        {
            A04rot += Math.PI;
            A05rot = -A05rot;
            A06rot += Math.PI;
        }

        // Convert to degrees
        A01rot *= -180.0 / Math.PI;
        A02rot = (A02rot * -180.0 / Math.PI) - 90.0;
        A03rot = (A03rot * -180.0 / Math.PI) + 90;
        A04rot *= 180.0 / Math.PI;
        A05rot *= -180.0 / Math.PI;
        A06rot *= -180.0 / Math.PI;

        Console.WriteLine($"A1: {A01rot}, A2: {A02rot}, A3: {A03rot}, A4: {A04rot}, A5: {A05rot}, A6: {A06rot}");
        return new PRC_AxVals(A01rot, A02rot, A03rot, A04rot, A05rot, A06rot, basePlane.Origin, basePlane.XAxis, basePlane.YAxis, basePlane.ZAxis, targetPlane.Origin, targetPlane.XAxis, targetPlane.YAxis, targetPlane.ZAxis, robottipX, robottipY, robottipZ);
    }
}

// Define missing classes
public class PRC_AxVals
{
    public double A01, A02, A03, A04, A05, A06;
    public Point3d BaseOrigin, TargetOrigin;
    public Vector3d BaseXAxis, BaseYAxis, BaseZAxis;
    public Vector3d TargetXAxis, TargetYAxis, TargetZAxis;
    public Vector3d RobotTipX, RobotTipY, RobotTipZ;
    public PRC_AxVals(double a01, double a02, double a03, double a04, double a05, double a06, Point3d baseOrigin, Vector3d baseXAxis, Vector3d baseYAxis, Vector3d baseZAxis, Point3d targetOrigin, Vector3d targetXAxis, Vector3d targetYAxis, Vector3d targetZAxis, Vector3d robotTipX, Vector3d robotTipY, Vector3d robotTipZ)
    {
        A01 = a01; A02 = a02; A03 = a03;
        A04 = a04; A05 = a05; A06 = a06;
        BaseOrigin = baseOrigin;
        BaseXAxis = baseXAxis;
        BaseYAxis = baseYAxis;
        BaseZAxis = baseZAxis;
        TargetOrigin = targetOrigin;
        TargetXAxis = targetXAxis;
        TargetYAxis = targetYAxis;
        TargetZAxis = targetZAxis;
        RobotTipX = robotTipX;
        RobotTipY = robotTipY;
        RobotTipZ = robotTipZ;
    }
}

public class PRC_RobotMini
{
    public double a01length, a01offset, a02length, a03length, a03offset, a04length;
}

public class PRC_RobotBase
{
    public double baseX, baseY, baseZ, baseA, baseB, baseC;
}

public class PRC_ToolMini
{
    public double toolvalX, toolvalY, toolvalZ, toolvalA, toolvalB, toolvalC;
}
