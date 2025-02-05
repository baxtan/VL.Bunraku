// For examples, see:
// https://thegraybook.vvvv.org/reference/extending/writing-nodes.html#examples

using Bunraku.Geometry;

namespace Main;

[ProcessNode]
public class InverseKinematic
{
    public static Bunraku_AxisValues Bunraku_InverseKinematic(
        Bunraku_Robot robot,
        Bunraku_RobotBase baseConfig,
        Bunraku_Tool tool,
        Bunraku.Geometry.Plane targetPlane,
        string status,
        Bunraku_AxisValues prevAxisValues)
    {
        double a01length = robot.a01length;
        double a01offset = robot.a01offset;
        double a02length = robot.a02length;
        double a03length = robot.a03length;
        double a03offset = robot.a03offset;
        double a04length = robot.a04length;

        // Invert offsets as required by the model.
        a01offset = a01offset * (-1.0);
        a03offset = a03offset * (-1.0);

        if (prevAxisValues == null)
        {
            prevAxisValues = new Bunraku_AxisValues(0, -90, 90, 0, 0, 0);
        }

        // Parse status input.
        char[] statusbitchars = status.ToCharArray();
        int statusbit0 = Convert.ToInt32(statusbitchars[2].ToString());
        int statusbit1 = Convert.ToInt32(statusbitchars[1].ToString());
        int statusbit2 = Convert.ToInt32(statusbitchars[0].ToString());

        // Base transformation.
        // The target plane is transformed according to the base configuration.
        Transform baseTransform = Transform.Rotation(baseConfig.BaseA, targetPlane.ZAxis, targetPlane.Origin)
            * Transform.Rotation(baseConfig.BaseB, targetPlane.YAxis, targetPlane.Origin)
            * Transform.Rotation(baseConfig.BaseC, targetPlane.XAxis, targetPlane.Origin);

        targetPlane.Transform(baseTransform);

        // Tool transformation.
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
        Bunraku.Geometry.Plane toolplane = new Bunraku.Geometry.Plane(
            ptTCP,
            new Vector3d(1, 0, 0),
            new Vector3d(0, 1, 0)
        );

        toolplane.Transform(Transform.Rotation(toolangA, toolplane.ZAxis, toolplane.Origin));
        toolplane.Transform(Transform.Rotation(toolangB, toolplane.YAxis, toolplane.Origin));
        toolplane.Transform(Transform.Rotation(toolangC, toolplane.XAxis, toolplane.Origin));

        // Instead of applying only a translation to the target plane,
        // compute a transformation that maps the tool plane onto the target plane.
        Transform toolToTarget = Transform.PlaneToPlane(toolplane, targetPlane);
        targetPlane.Transform(toolToTarget);

        // Calculate joint angles for A01–A03.
        double A01rot = Math.Atan2(targetPlane.Origin.Y, targetPlane.Origin.X);
        double planarDistance = Math.Sqrt(
            targetPlane.Origin.X * targetPlane.Origin.X +
            targetPlane.Origin.Y * targetPlane.Origin.Y
        );
        double hypotenuse = Math.Sqrt(planarDistance * planarDistance + Math.Pow(targetPlane.Origin.Z - a01length, 2));
        double epsilon = Math.Atan2(targetPlane.Origin.Z - a01length, planarDistance);
        double cosBeta = (a02length * a02length + hypotenuse * hypotenuse - a03length * a03length) / (2 * a02length * hypotenuse);
        cosBeta = Math.Max(-1.0, Math.Min(1.0, cosBeta)); // Clamp to avoid NaN.
        double beta = Math.Acos(cosBeta);

        double A02rot = Math.PI / 2 - (epsilon + beta);

        double cosGamma = (a02length * a02length + a03length * a03length - hypotenuse * hypotenuse) / (2 * a02length * a03length);
        cosGamma = Math.Max(-1.0, Math.Min(1.0, cosGamma)); // Clamp to avoid NaN.
        double A03rot = Math.Acos(cosGamma) - Math.PI / 2;

        // Adjust A02rot and A03rot based on status bits.
        if (statusbit1 == 0 && statusbit0 == 0)
        {
            A02rot = -Math.PI / 2;
            A03rot = Math.PI / 2;
        }
        else if (statusbit1 == 0 && statusbit0 == 1)
        {
            A01rot += Math.PI;
            A02rot = Math.PI / 2;
            A03rot = -Math.PI / 2;
        }

        // Calculate joint angles for A04–A06.
        // These angles are computed from the orientation (axes) of the target plane.
        Vector3d robottipZ = targetPlane.ZAxis;
        Vector3d robottipX = targetPlane.XAxis;
        Vector3d robottipY = targetPlane.YAxis;

        double A04rot = Math.Atan2(robottipY.Z, robottipX.Z);
        double A05rot = Math.Acos(robottipZ.Z);
        double A06rot = -Math.Atan2(robottipZ.Y, -robottipZ.X);

        // Convert all joint angles from radians to degrees.
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

public class Bunraku_Tool
{
    public double toolvalX { get; set; } = 0;
    public double toolvalY { get; set; } = 0;
    public double toolvalZ { get; set; } = 0;
    public double toolvalA { get; set; } = 0;
    public double toolvalB { get; set; } = 0;
    public double toolvalC { get; set; } = 0;

    public Bunraku_Tool() { }

    public Bunraku_Tool(double toolvalX, double toolvalY, double toolvalZ, double toolvalA, double toolvalB, double toolvalC)
    {
        this.toolvalX = toolvalX;
        this.toolvalY = toolvalY;
        this.toolvalZ = toolvalZ;
        this.toolvalA = toolvalA;
        this.toolvalB = toolvalB;
        this.toolvalC = toolvalC;
    }
}
