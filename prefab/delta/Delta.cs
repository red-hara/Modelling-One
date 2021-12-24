using Godot;
using System;

[Tool]
public class Delta : Spatial
{

    // A constant variable "platformRadius" of the float type is created.
    public const float platformRadius = 120;

    // A constant variable "baseRadius" of the float type is created.
    public const float baseRadius = 350;

    // A constant variable "baseLift" of the float type is created.
    public const float baseLift = 50;

    // A constant variable "armLength" of the float type is created.
    public const float armLength = 400;

    // A constant variable "connectorRadius" of the float type is created.
    public const float connectorRadius = 800;



    // Creating and Adding parameters "AxisA" to "Script Variables".
    // Float type
    [Export]
    public float axisA;

    // Creating and Adding parameters "AxisB" to "Script Variables".
    // Float type
    [Export]
    public float axisB;

    // Creating and Adding parameters "AxisC" to "Script Variables".
    // Float type
    [Export]
    public float axisC;

    // Adding "platformPath" to the "Script Variables", to provide the
    // robot, which controls the settings of the work.
    [Export]
    public NodePath platformPath;

    // An object of the Spatial type is being created.
    private Spatial platform;

    // Adding "armAPath" to the "Script Variables", to provide the
    // robot, which controls the settings of the work.
    [Export]
    public NodePath armAPath;

    // An object of the Spatial type is being created.
    private Spatial armA;

    // Adding "armBPath" to the "Script Variables", to provide the
    // robot, which controls the settings of the work.
    [Export]
    public NodePath armBPath;

    // An object of the Spatial type is being created.
    private Spatial armB;

    // Adding "armCPath" to the "Script Variables", to provide the
    // robot, which controls the settings of the work.
    [Export]
    public NodePath armCPath;

    // An object of the Spatial type is being created.
    private Spatial armC;

    // Adding "connectorPathAA" to the "Script Variables", to provide the
    // robot, which controls the settings of the work.
    [Export]
    private NodePath connectorPathAA;

    // Adding "connectorPathAB" to the "Script Variables", to provide the
    // robot, which controls the settings of the work.
    [Export]
    private NodePath connectorPathAB;

    // Adding "connectorPathAC" to the "Script Variables", to provide the
    // robot, which controls the settings of the work.
    [Export]
    private NodePath connectorPathBA;

    // Adding "connectorPathBB" to the "Script Variables", to provide the
    // robot, which controls the settings of the work.
    [Export]
    private NodePath connectorPathBB;

    // Adding "connectorPathCA" to the "Script Variables", to provide the
    // robot, which controls the settings of the work.
    [Export]
    private NodePath connectorPathCA;

    // Adding "connectorPathCB" to the "Script Variables", to provide the
    // robot, which controls the settings of the work.
    [Export]
    private NodePath connectorPathCB;

    // An array of Spatial objects is created.
    private Spatial[] connectors;

    // An object of the Vector type is being created.
    private Vector3 position;

    // current flange position.
    public Vector3 Position
    {
        get
        {
            return position;
        }
    }

    // Function for the possibility of installation "platformPath","armXPath x-A,B,C".
    // Function for the possibility of installation "ConnectorPath XY:X-A,B,C.Y-A,B,C".
    public override void _Ready()
    {
        platform = GetNode<Spatial>(platformPath);
        armA = GetNode<Spatial>(armAPath);
        armB = GetNode<Spatial>(armBPath);
        armC = GetNode<Spatial>(armCPath);

        connectors = new Spatial[]{
            GetNode<Spatial>(connectorPathAA),
            GetNode<Spatial>(connectorPathAB),
            GetNode<Spatial>(connectorPathBA),
            GetNode<Spatial>(connectorPathBB),
            GetNode<Spatial>(connectorPathCA),
            GetNode<Spatial>(connectorPathCB),
        };
    }

    // Position setting function.
    // Accepts the <delta> parameter.
    public override void _Process(float delta)
    {
        armA.RotationDegrees = new Vector3(0.0f, axisA, 0.0f);
        armB.RotationDegrees = new Vector3(0.0f, axisB, 0.0f);
        armC.RotationDegrees = new Vector3(0.0f, axisC, 0.0f);


        position = Forward() ?? platform.Translation;
        platform.Translation = position;
        ManageConnectors();
    }

    // Robot connection management function.
    private void ManageConnectors()
    {
        float[] axis = new float[]{
            Mathf.Deg2Rad(axisA),
            Mathf.Deg2Rad(axisB),
            Mathf.Deg2Rad(axisC)
        };
        for (int index = 0; index < 3; index++)
        {
            Vector3 armPosition = ArmPosition(axis[index], index);
            Vector3 delta = platform.Translation - armPosition;
            float angleZ = Mathf.Atan2(delta.y, delta.x) - index * Mathf.Pi * 2 / 3;
            float angleY = Mathf.Atan2(delta.z, Mathf.Sqrt(delta.x * delta.x + delta.y * delta.y));
            connectors[index * 2].Transform = new Transform(
                new Quat(Vector3.Up, -axis[index]) * new Quat(Vector3.Back, angleZ) * new Quat(Vector3.Up, -angleY - Mathf.Pi / 2),
                new Vector3(0, 50, -armLength)
            );
            connectors[index * 2 + 1].Transform = new Transform(
                new Quat(Vector3.Up, -axis[index]) * new Quat(Vector3.Back, angleZ) * new Quat(Vector3.Up, -angleY - Mathf.Pi / 2),
                new Vector3(0, -50, -armLength)
            );
        }
    }

    // Solve forward(direct) kinematics problem.
    private Vector3? Forward()
    {
        float a = Mathf.Deg2Rad(axisA);
        float b = Mathf.Deg2Rad(axisB);
        float c = Mathf.Deg2Rad(axisC);

        Vector3 centerA = ArmPosition(a, 0);
        Vector3 centerB = ArmPosition(b, 1);
        Vector3 centerC = ArmPosition(c, 2);

        Vector3[] solution = SphereIntersection3(centerA, centerB, centerC, connectorRadius);
        if (solution.Length == 0)
        {
            return null;
        }
        if (solution[0].z < solution[1].z)
        {
            return solution[0];
        }
        return solution[1];
    }

    // Determine the rotation of the manipulator.
    // Accepted parameter of <float axis> and <int index>
    private Vector3 ArmPosition(float axis, int index)
    {
        Transform rotationOrigin = new Transform(new Quat(Vector3.Back, Mathf.Deg2Rad(index * 120)), Vector3.Zero) *
            new Transform(Basis.Identity, new Vector3(baseRadius - platformRadius, 0, baseLift));
        Quat rotation = new Quat(Vector3.Up, axis);
        Vector3 end = rotation.Xform(new Vector3(0, 0, -armLength));
        return rotationOrigin.Xform(end);
    }

    // Intersect three spheres.
    // Accepts parameters of vector types <a,b,c> and float <radius>.
    // Return array point.
    private static Vector3[] SphereIntersection3(Vector3 a, Vector3 b, Vector3 c, float radius)
    {
        var maybeCircle = SphereIntersection2(a, b, radius);
        if (maybeCircle is null)
        {
            return new Vector3[0];
        }
        var circle = maybeCircle ?? default;

        var maybePosition = SphereCircleIntersection(radius, c, circle.Center, circle.AxisU, circle.AxisV, circle.Radius);
        if (maybePosition is null)
        {
            return new Vector3[0];
        }
        var position = maybePosition ?? default;

        return new Vector3[]{
            position.PositionA,
            position.PositionB,
        };
    }

    // The function of calculating the intersection between a sphere and a circle.
    // Accepts parameters of vector types <sphereCenter, circleCenter, AsixU, AxisV> and float <radius, circleRadius>
    public static (Vector3 PositionA, Vector3 PositionB)? SphereCircleIntersection(float radius, Vector3 sphereCenter, Vector3 circleCenter, Vector3 AxisU, Vector3 AxisV, float circleRadius)
    {
        var maybeSolution = SphereCircleIntersectionAngles(radius, sphereCenter, circleCenter, AxisU, AxisV, circleRadius);
        if (maybeSolution is null)
        {
            return null;
        }
        var solution = maybeSolution ?? default;

        Vector3 positionA = circleCenter + circleRadius * Mathf.Cos(solution.SolutionA) * AxisU + circleRadius * Mathf.Sin(solution.SolutionA) * AxisV;
        Vector3 positionB = circleCenter + circleRadius * Mathf.Cos(solution.SolutionB) * AxisU + circleRadius * Mathf.Sin(solution.SolutionB) * AxisV;

        return (positionA, positionB);
    }

    // Intersect three spheres.
    // Accepts parameters of vector types <a,b> and float <radius>.
    // Return circle geometry
    public static (float Radius, Vector3 Center, Vector3 AxisV, Vector3 AxisU)? SphereIntersection2(Vector3 a, Vector3 b, float radius)
    {
        Vector3 ab = b - a;
        float halfDistance = ab.Length() / 2;
        if (halfDistance > radius)
        {
            return null;
        }
        Vector3 abNorm = ab.Normalized();
        Vector3 AxisV = OrthogonalVector(abNorm).Normalized();
        Vector3 AxisU = abNorm.Cross(AxisV).Normalized();
        Vector3 center = a + ab / 2;
        float circeRadius = Mathf.Sqrt(radius * radius - halfDistance * halfDistance);
        return (circeRadius, center, AxisV, AxisU);
    }
    // Calculate the orthogonal vector.
    // Accepted parameter <Vector3 vector>
    public static Vector3 OrthogonalVector(Vector3 vector)
    {
        float a = vector.x;
        float b = vector.y;
        float c = vector.z;
        return new Vector3(
            b + c,
            c - a,
            -a - b
        );
    }

    // Calculate intersection angles on the provided circle and sphere
    // Accepts parameters of vector types <sphereCenter, circleCenter, AsixU, AxisV> and float <radius, circleRadius>
    public static (float SolutionA, float SolutionB)? SphereCircleIntersectionAngles(float radius,
        Vector3 sphereCenter,
        Vector3 circleCenter,
        Vector3 axisU,
        Vector3 axisV,
        float circleRadius
    )
    {
        Vector3 delta = circleCenter - sphereCenter;
        float gamma = radius * radius - delta.LengthSquared() - circleRadius * circleRadius;
        float alpha = 2 * delta.Dot(axisU) * circleRadius;
        float beta = 2 * delta.Dot(axisV) * circleRadius;

        return SolveEquation(alpha, beta, gamma);
    }

    // Solve circle intersection equation.
    // Accept parameters type of float <alpha, beta, gamma>.
    public static (float SolutionA, float SolutionB)? SolveEquation(float alpha, float beta, float gamma)
    {
        float c = gamma - alpha;
        float b = -2 * beta;
        float a = gamma + alpha;
        float disc = b * b - 4 * a * c;
        if (disc < 0)
        {
            return null;
        }
        float tt0 = (-b + Mathf.Sqrt(disc)) / 2 / a;
        float tt1 = (-b - Mathf.Sqrt(disc)) / 2 / a;

        tt0 = 2 * Mathf.Atan(tt0);
        tt1 = 2 * Mathf.Atan(tt1);

        return (tt0, tt1);
    }
}
