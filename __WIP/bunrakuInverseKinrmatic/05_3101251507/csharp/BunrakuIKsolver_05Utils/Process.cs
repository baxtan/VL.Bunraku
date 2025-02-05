// Fixed Process.cs errors and aligned with InverseKinematic.cs
using Bunraku.Geometry;

namespace Main;

[ProcessNode]
public class InverseKinematic
{
    public static Bunraku_AxVals Bunraku_InverseKinematic(Bunraku_RobotMini robot, Bunraku_RobotBase baseConfig, Bunraku_ToolMini tool, Bunraku.Geometry.Plane targetPlane, string status, Bunraku_AxVals prevAxVals)
    {
        double a01length = robot.a01length;
        double a01offset = robot.a01offset * (-1.0);
        double a02length = robot.a02length;
        double a03length = robot.a03length;
        double a03offset = robot.a03offset * (-1.0);
        double a04length = robot.a04length;

        if (prevAxVals == null)
        {
            prevAxVals = new Bunraku_AxVals(0, -90, 90, 0, 0, 0);
        }

        // Parse status input
        char[] statusbitchars = status.ToCharArray();
        int statusbit0 = Convert.ToInt32(statusbitchars[2].ToString());
        int statusbit1 = Convert.ToInt32(statusbitchars[1].ToString());
        int statusbit2 = Convert.ToInt32(statusbitchars[0].ToString());

        // Base transformation
        Bunraku.Geometry.Plane basePlane = new Bunraku.Geometry.Plane(new Point3d(baseConfig.baseX, baseConfig.baseY, baseConfig.baseZ), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));
        basePlane.Transform(Transform.Rotation(baseConfig.baseA * Math.PI / 180.0, basePlane.ZAxis, basePlane.Origin));
        basePlane.Transform(Transform.Rotation(baseConfig.baseB * Math.PI / 180.0, basePlane.YAxis, basePlane.Origin));
        basePlane.Transform(Transform.Rotation(baseConfig.baseC * Math.PI / 180.0, basePlane.XAxis, basePlane.Origin));

        Transform baseTransform = Transform.Translation(new Vector3d(basePlane.Origin.X, basePlane.Origin.Y, basePlane.Origin.Z));
        targetPlane.Transform(baseTransform);

        // Adjust for tool offset
        Bunraku.Geometry.Plane toolPlane = new Bunraku.Geometry.Plane(new Point3d(tool.toolvalX, tool.toolvalY, tool.toolvalZ), new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));
        toolPlane.Transform(Transform.Rotation(Math.PI * tool.toolvalA / 180.0, toolPlane.ZAxis, toolPlane.Origin));
        toolPlane.Transform(Transform.Rotation(Math.PI * tool.toolvalB / 180.0, toolPlane.YAxis, toolPlane.Origin));
        toolPlane.Transform(Transform.Rotation(Math.PI * tool.toolvalC / 180.0, toolPlane.XAxis, toolPlane.Origin));

        Vector3d translation = new Vector3d(targetPlane.Origin.X - toolPlane.Origin.X, targetPlane.Origin.Y - toolPlane.Origin.Y, targetPlane.Origin.Z - toolPlane.Origin.Z);
        targetPlane.Transform(Transform.Translation(translation));

        // Solve joint angles
        double A01rot = Math.Atan2(targetPlane.Origin.Y, targetPlane.Origin.X);
        double pval = Math.Sqrt(targetPlane.Origin.X * targetPlane.Origin.X + targetPlane.Origin.Y * targetPlane.Origin.Y);
        double cval = Math.Sqrt(Math.Pow(targetPlane.Origin.Z - a01length, 2) + Math.Pow(pval + a01offset, 2));
        double epsilon = Math.Atan2(targetPlane.Origin.Z - a01length, pval + a01offset);
        double alpha = Math.Acos((a02length * a02length + cval * cval - a03length * a03length) / (2 * a02length * cval));
        double gamma = Math.Acos((a02length * a02length + a03length * a03length - cval * cval) / (2 * a02length * a03length));
        double phi = Math.Atan(a03offset / a03length);

        double A02rot, A03rot;
        if (statusbit1 == 1 && statusbit0 == 0)
        {
            A02rot = alpha + epsilon - Math.PI / 2.0;
            A03rot = gamma + phi - Math.PI / 2.0;
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

        double A04rot = Math.Atan2(robottipY.Z, robottipX.Z);
        double A05rot = Math.Acos(robottipZ.Z);
        double A06rot = -Math.Atan2(robottipZ.Y, -robottipZ.X);

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

        return new Bunraku_AxVals(A01rot, A02rot, A03rot, A04rot, A05rot, A06rot);
    }
}

// Define missing classes
public class Bunraku_AxVals
{
    public double A01, A02, A03, A04, A05, A06;
    public Bunraku_AxVals(double a01, double a02, double a03, double a04, double a05, double a06)
    {
        A01 = a01; A02 = a02; A03 = a03;
        A04 = a04; A05 = a05; A06 = a06;
    }
}

public class Bunraku_RobotMini
{
    public double a01length, a01offset, a02length, a03length, a03offset, a04length;
}

public class Bunraku_RobotBase
{
    public double baseX, baseY, baseZ, baseA, baseB, baseC;
}

public class Bunraku_ToolMini
{
    public double toolvalX, toolvalY, toolvalZ, toolvalA, toolvalB, toolvalC;
}
