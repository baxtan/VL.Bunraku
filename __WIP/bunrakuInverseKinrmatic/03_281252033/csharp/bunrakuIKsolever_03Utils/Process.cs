// For examples, see:
// https://thegraybook.vvvv.org/reference/extending/writing-nodes.html#examples

using Bunraku.Geometry;

namespace Bunraku;

[ProcessNode]


public class InverseKinematic
{
    public static Bunraku_AxisValues Bunraku_InverseKinematic(Bunraku_Robot robot, Bunraku_RobotBase baseConfig, Bunraku_RobotTool tool, Bunraku.Geometry.Plane targetPlane, string status, Bunraku_AxisValues prevAxisValues)
    {
        double a01length = robot.a01length;
        double a01offset = robot.a01offset;
        double a02length = robot.a02length;
        double a03length = robot.a03length;
        double a03offset = robot.a03offset;
        double a04length = robot.a04length;

        a01offset = a01offset * (-1.0);
       a03offset = a03offset * (-1.0);

        if (prevAxisValues == null)
        {
            prevAxisValues = new Bunraku_AxisValues(0, -90, 90, 0, 0, 0);
        }

        // Base transformation
        Transform baseTransform = Transform.Rotation(baseConfig.BaseA, targetPlane.ZAxis, targetPlane.Origin)
            * Transform.Rotation(baseConfig.BaseB, targetPlane.YAxis, targetPlane.Origin)
            * Transform.Rotation(baseConfig.BaseC, targetPlane.XAxis, targetPlane.Origin);

        targetPlane.Transform(baseTransform);

        // Tool transformation
        double toolvalX = tool.toolvalX;
        double toolvalY = tool.toolvalY;
        double toolvalZ = tool.toolvalZ;
        double toolvalA = tool.toolvalA;
        double toolvalB = tool.toolvalB;
        double toolvalC = tool.toolvalC;

        double toolangA = Math.PI * toolvalA / 180.0;
        double toolangB = Math.PI * toolvalB / 180.0;
        double toolangC = Math.PI * toolvalC / 180.0;

        Point3d ptTCP = new Point3d(toolvalX, toolvalY, toolvalZ);
        Bunraku.Geometry.Plane toolplane = new Bunraku.Geometry.Plane(ptTCP, new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));

        toolplane.Transform(Transform.Rotation(toolangA, toolplane.ZAxis, toolplane.Origin));
        toolplane.Transform(Transform.Rotation(toolangB, toolplane.YAxis, toolplane.Origin));
        toolplane.Transform(Transform.Rotation(toolangC, toolplane.XAxis, toolplane.Origin));

        // Adjust target plane
        Vector3d translation = new Vector3d(
    targetPlane.Origin.X - toolplane.Origin.X,
    targetPlane.Origin.Y - toolplane.Origin.Y,
    targetPlane.Origin.Z - toolplane.Origin.Z
);
        targetPlane.Transform(Transform.Translation(translation));

        // Calculate joint angles for A01-A03
        double A01rot = Math.Atan2(targetPlane.Origin.Y, targetPlane.Origin.X);
        double planarDistance = Math.Sqrt(targetPlane.Origin.X * targetPlane.Origin.X + targetPlane.Origin.Y * targetPlane.Origin.Y);
        double hypotenuse = Math.Sqrt(planarDistance * planarDistance + (targetPlane.Origin.Z - a01length) * (targetPlane.Origin.Z - a01length));

        double epsilon = Math.Atan2(targetPlane.Origin.Z - a01length, planarDistance);
        double beta = Math.Acos((a02length * a02length + hypotenuse * hypotenuse - a03length * a03length) / (2 * a02length * hypotenuse));

        double A02rot = Math.PI / 2 - (epsilon + beta);
        double A03rot = Math.Acos((a02length * a02length + a03length * a03length - hypotenuse * hypotenuse) / (2 * a02length * a03length)) - Math.PI / 2;

        // Calculate joint angles for A04-A06
        Vector3d robottipZ = new Vector3d(
            Math.Cos(A01rot) * Math.Cos(A02rot + A03rot),
            Math.Sin(A01rot) * Math.Cos(A02rot + A03rot),
            Math.Sin(A02rot + A03rot)
        );

        Vector3d robottipY = new Vector3d(-Math.Sin(A01rot), Math.Cos(A01rot), 0);

        Vector3d robottipX = robottipY.Cross(robottipZ);

        Bunraku.Geometry.Plane robottipPlane = new Bunraku.Geometry.Plane(new Point3d(0, 0, 0), robottipX, robottipY);

        double dot31 = robottipPlane.XAxis.Dot(targetPlane.ZAxis);
        double dot32 = robottipPlane.YAxis.Dot(targetPlane.ZAxis);
        double dot33 = robottipPlane.ZAxis.Dot(targetPlane.ZAxis);
        double dot23 = robottipPlane.ZAxis.Dot(targetPlane.YAxis);
        double dot13 = robottipPlane.ZAxis.Dot(targetPlane.XAxis);

        double sinB = Math.Sqrt(dot31 * dot31 + dot32 * dot32);

        double A04rot = Math.Atan2(dot32 / sinB, dot31 / sinB);
        double A05rot = Math.Atan2(sinB, dot33);
        double A06rot = Math.Atan2(dot23 / sinB, dot13 / sinB);

        // Convert to degrees
        A01rot *= 180.0 / Math.PI;
        A02rot *= 180.0 / Math.PI;
        A03rot *= 180.0 / Math.PI;
        A04rot *= 180.0 / Math.PI;
        A05rot *= 180.0 / Math.PI;
        A06rot *= 180.0 / Math.PI;

        return new Bunraku_AxisValues(A01rot, A02rot, A03rot, A04rot, A05rot, A06rot);
    }
}

public class Bunraku_AxisValues
{
    public double A01 { get; set; }
    public double A02 { get; set; }
    public double A03 { get; set; }
    public double A04 { get; set; }
    public double A05 { get; set; }
    public double A06 { get; set; }

    public Bunraku_AxisValues(double a01, double a02, double a03, double a04, double a05, double a06)
    {
        A01 = a01;
        A02 = a02;
        A03 = a03;
        A04 = a04;
        A05 = a05;
        A06 = a06;
    }
}

public class Bunraku_Robot
{
    public double a01length;
    public double a01offset;
    public double a02length;
    public double a03length;
    public double a03offset;
    public double a04length;

    public Bunraku_Robot(double a01length, double a01offset, double a02length, double a03length, double a03offset, double a04length)
    {
        this.a01length = a01length;
        this.a01offset = a01offset;
        this.a02length = a02length;
        this.a03length = a03length;
        this.a03offset = a03offset;
        this.a04length = a04length;
    }

    public static Bunraku_Robot Bunraku_KR610R9002()
    {
        return new Bunraku_Robot(400, 25, 455, 420, 25, 90);
    }
}

public class Bunraku_RobotBase
{
    public double BaseA { get; set; } = 0;
    public double BaseB { get; set; } = 0;
    public double BaseC { get; set; } = 0;

    public Bunraku_RobotBase() { }

    public Bunraku_RobotBase(double baseA, double baseB, double baseC)
    {
        BaseA = baseA;
        BaseB = baseB;
        BaseC = baseC;
    }
}

public class Bunraku_RobotTool
{
    public double toolvalX { get; set; } = 0;
    public double toolvalY { get; set; } = 0;
    public double toolvalZ { get; set; } = 0;
    public double toolvalA { get; set; } = 0;
    public double toolvalB { get; set; } = 0;
    public double toolvalC { get; set; } = 0;

    public Bunraku_RobotTool() { }

    public Bunraku_RobotTool(double toolvalX, double toolvalY, double toolvalZ, double toolvalA, double toolvalB, double toolvalC)
    {
        this.toolvalX = toolvalX;
        this.toolvalY = toolvalY;
        this.toolvalZ = toolvalZ;
        this.toolvalA = toolvalA;
        this.toolvalB = toolvalB;
        this.toolvalC = toolvalC;
    }
}
