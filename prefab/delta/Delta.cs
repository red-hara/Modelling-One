using Godot;

/// <summary>Delta robot controller. Solves forward kinematics
/// problem.</summary>
[Tool] // Mark this script to be executed in the editor.
public class Delta : Spatial
{

    /// <summary>Distance from the platform center to connectors.</summary>
    public const float platformRadius = 120;

    /// <summary>Distance from the base center to individual controllable
    /// joints.</summary>
    public const float baseRadius = 350;

    /// <summary>Distance from the horizontal zero plane to the individual
    /// controllable joints.</summary>
    public const float baseLift = 50;

    /// <summary>The individual arm (black part) length.</summary>
    public const float armLength = 400;

    /// <summary>Individual connector (gray part) length.</summary>
    public const float connectorRadius = 800;

    /// <summary>First joint angle in degrees.</summary>
    [Export] // Export this field to the editor Inspector.
    public float axisA;

    /// <summary>Second joint angle in degrees.</summary>
    [Export]
    public float axisB;

    /// <summary>Third joint angle in degrees.</summary>
    [Export]
    public float axisC;

    /// <summary>Scene graph path to the platform node.</summary>
    [Export]
    public NodePath platformPath;

    /// <summary>Platform spatial representation.</summary>
    private Spatial platform;

    /// <summary>Scene graph path to the first arm.</summary>
    [Export]
    public NodePath armAPath;

    /// <summary>The first arm spatial representation.</summary>
    private Spatial armA;

    /// <summary>Scene graph path to the second arm.</summary>
    [Export]
    public NodePath armBPath;

    /// <summary>The second arm spatial representation.</summary>
    private Spatial armB;

    /// <summary>Scene graph path to the third arm.</summary>
    [Export]
    public NodePath armCPath;

    /// <summary>The third arm spatial representation.</summary>
    private Spatial armC;

    /// <summary>Scene graph path to the first connector of the first
    /// arm.</summary>
    [Export]
    private NodePath connectorPathAA;

    /// <summary>Scene graph path to the second connector of the first
    /// arm.</summary>
    [Export]
    private NodePath connectorPathAB;

    /// <summary>Scene graph path to the first connector of the second
    /// arm.</summary>
    [Export]
    private NodePath connectorPathBA;

    /// <summary>Scene graph path to the second connector of the second
    /// arm.</summary>
    [Export]
    private NodePath connectorPathBB;

    /// <summary>Scene graph path to the first connector of the third
    /// arm.</summary>
    [Export]
    private NodePath connectorPathCA;

    /// <summary>Scene graph path to the second connector of the third
    /// arm.</summary>
    [Export]
    private NodePath connectorPathCB;

    /// <summary>Spatial representations of the delta (grey parts)
    /// connectors.<summary>
    private Spatial[] connectors;

    /// <summary>The delta's platform position relative to the robot's zero
    /// frame.<summary> 
    private Vector3 position;


    /// <summary>The delta's platform position relative to the robot's zero
    /// frame.<summary> 
    public Vector3 Position
    {
        get // Mark it as getter, forbid setting the value.
        {
            return position;
        }
    }

    // Ready is called automatically when the object with this script is added
    // to the scene or the scene is initialized.
    public override void _Ready()
    {
        // Assign spatial representations based on their paths.
        platform = GetNode<Spatial>(platformPath);
        armA = GetNode<Spatial>(armAPath);
        armB = GetNode<Spatial>(armBPath);
        armC = GetNode<Spatial>(armCPath);

        // Collect all connectors spatial representations in single array for
        // ease of use.
        connectors = new Spatial[]{
            GetNode<Spatial>(connectorPathAA),
            GetNode<Spatial>(connectorPathAB),
            GetNode<Spatial>(connectorPathBA),
            GetNode<Spatial>(connectorPathBB),
            GetNode<Spatial>(connectorPathCA),
            GetNode<Spatial>(connectorPathCB),
        };
    }

    // Process is called automatically during every scene graph update. The
    // delta parameter represents the time passed since the previous update in
    // seconds.
    public override void _Process(float delta)
    {
        // Explicitly set arm rotations.
        armA.RotationDegrees = new Vector3(0.0f, axisA, 0.0f);
        armB.RotationDegrees = new Vector3(0.0f, axisB, 0.0f);
        armC.RotationDegrees = new Vector3(0.0f, axisC, 0.0f);

        // Try solving the fordard kinematics problem and if the result is
        // `null` keep the current platform position.
        position = Forward() ?? platform.Translation;
        platform.Translation = position;
        // Update each connector to follow the platform.
        ManageConnectors();
    }

    /// <summary>Update arm connectors to point in the platform
    /// direction.<summary>
    private void ManageConnectors()
    {
        // Recalculate axes into radian form.
        float[] axis = new float[]{
            Mathf.Deg2Rad(axisA),
            Mathf.Deg2Rad(axisB),
            Mathf.Deg2Rad(axisC)
        };
        // Perform update for each connector.
        for (int index = 0; index < 3; index++)
        {
            // Determine current position of the arm end. Note that this method
            // calculates the arm position with respect to the platform center,
            // i.e., it calculates not the real position, but virtual,
            // translated towards the platform center.
            Vector3 armPosition = ArmPosition(axis[index], index);
            // Estimate direction towards the platform center.
            Vector3 delta = platform.Translation - armPosition;
            // Calculate the vertical rotation angle towards the platform center
            // in respect to the connector position.
            float angleZ = Mathf.Atan2(delta.y, delta.x) -
                index * Mathf.Pi * 2 / 3;
            // Calculate the horizontal rotation angle towards the platform
            // center in respect to the connector position.
            float angleY = Mathf.Atan2(
                delta.z,
                Mathf.Sqrt(delta.x * delta.x + delta.y * delta.y)
            );
            // Update the first connector rotation using precalculated angles
            // and the value of the robot joint. Note that the position is also
            // set to shorten the code.
            connectors[index * 2].Transform = new Transform(
                new Quat(Vector3.Up, -axis[index]) *
                    new Quat(Vector3.Back, angleZ) *
                    new Quat(Vector3.Up, -angleY - Mathf.Pi / 2),
                new Vector3(0, 50, -armLength)
            );
            // Update the second connector rotation using precalculated angles
            // and the value of the robot joint. Note that the position is also
            // set to shorten the code.
            connectors[index * 2 + 1].Transform = new Transform(
                new Quat(Vector3.Up, -axis[index]) *
                    new Quat(Vector3.Back, angleZ) *
                    new Quat(Vector3.Up, -angleY - Mathf.Pi / 2),
                new Vector3(0, -50, -armLength)
            );
        }
    }

    /// <summary>Solve the forward kinematics problem. If multiple solutions
    /// present takes the one with minimal Z value</summary>
    /// <returns>The forward kinematics problem solution or <c>null</c> if none
    /// found.</returns>
    private Vector3? Forward()
    {
        // Recalculate joint values to radians.
        float a = Mathf.Deg2Rad(axisA);
        float b = Mathf.Deg2Rad(axisB);
        float c = Mathf.Deg2Rad(axisC);

        // Calculate centers of spheres.
        Vector3 centerA = ArmPosition(a, 0);
        Vector3 centerB = ArmPosition(b, 1);
        Vector3 centerC = ArmPosition(c, 2);

        // The solution is based on the three spheres intersection approach.
        // Since each connector (grey part) end is located on the sphere with
        // center in the arm (black part) end, the solutions are the positions
        // of the spheres intersections.
        Vector3[] solution = SphereIntersection3(
            centerA,
            centerB,
            centerC,
            connectorRadius
        );
        // If there is no solutions return `null`.
        if (solution.Length == 0)
        {
            return null;
        }
        // Choose solution with lower Z value.
        if (solution[0].z < solution[1].z)
        {
            return solution[0];
        }
        return solution[1];
    }

    /// <summary>Calculate the arm (black part) end position based on its index
    /// and angle. Note that the calculation is performed as if the platform has
    /// zero radius and arms are moved closer to the base center.<summary>
    /// <param name="axis">the arm rotation in degrees.</param>
    /// <param name="index">the arm index.</param>
    /// <returns>The spatial location of the arm end in relation to the base
    /// center.</returns>
    private Vector3 ArmPosition(float axis, int index)
    {
        // Calculate the arm start location.
        Transform rotationOrigin =
            new Transform(
                new Quat(Vector3.Back, Mathf.Deg2Rad(index * 120)),
                Vector3.Zero) *
            new Transform(
                Basis.Identity,
                new Vector3(baseRadius - platformRadius, 0, baseLift)
            );
        // Get the rotation around the arm axis.
        Quat rotation = new Quat(Vector3.Up, axis);
        // Calculate the arm's end position in respect to it's local coordinate
        // system.
        Vector3 end = rotation.Xform(new Vector3(0, 0, -armLength));
        // Apply the arm's start transformation to the calculated end position.
        return rotationOrigin.Xform(end);
    }

    /// <summary>Calculate intersection of three spheres of equal
    /// radius. The result is either empty array or array with two
    /// solutions.</summary>
    /// <param name="a">the center of the first sphere.</param>
    /// <param name="b">the center of the second sphere.</param>
    /// <param name="c">the center of the third sphere.</param>
    /// <returns>The array of solutions.</returns>
    private static Vector3[] SphereIntersection3(
        Vector3 a,
        Vector3 b,
        Vector3 c,
        float radius
    )
    {
        // Intersect two spheres to calculate the intersection circle.
        var maybeCircle = SphereIntersection2(a, b, radius);
        // Return empty array if there is no intersection.
        if (maybeCircle is null)
        {
            return new Vector3[0];
        }
        // Extract the circle data from the nullable field.
        var circle = maybeCircle.Value;

        // Find intersection of the circle and the third sphere.
        var maybePosition = SphereCircleIntersection(
            radius,
            c,
            circle.Center,
            circle.AxisU,
            circle.AxisV,
            circle.Radius
        );
        if (maybePosition is null)
        {
            return new Vector3[0];
        }
        var position = maybePosition.Value;

        return new Vector3[]{
            position.PositionA,
            position.PositionB,
        };
    }

    /// <summary>Calculate the intersection between sphere and circle. The
    /// circle is represented by it's center and two orthogonal axes in it's
    /// plane.</summary>
    /// <param name="radius">the sphere radius.</param>
    /// <param name="sphereCenter">the sphere center position.</param>
    /// <param name="axisU">the first circle axis.</param>
    /// <param name="axisV">the second circle axis.</param>
    /// <param name="circleRadius">the circle radius.</param>
    /// <returns>The nullable solution tuple.<returns>
    public static (Vector3 PositionA, Vector3 PositionB)?
    SphereCircleIntersection(
        float radius,
        Vector3 sphereCenter,
        Vector3 circleCenter,
        Vector3 axisU,
        Vector3 axisV,
        float circleRadius
    )
    {
        // Get intersection angles.
        var maybeSolution = SphereCircleIntersectionAngles(
            radius,
            sphereCenter,
            circleCenter,
            axisU,
            axisV,
            circleRadius
        );
        if (maybeSolution is null)
        {
            return null;
        }
        var solution = maybeSolution.Value;

        // Calculate two positions by scaling the axes.
        Vector3 positionA = circleCenter +
            circleRadius * Mathf.Cos(solution.SolutionA) * axisU +
            circleRadius * Mathf.Sin(solution.SolutionA) * axisV;
        Vector3 positionB = circleCenter +
            circleRadius * Mathf.Cos(solution.SolutionB) * axisU +
            circleRadius * Mathf.Sin(solution.SolutionB) * axisV;

        return (positionA, positionB);
    }

    /// <summary>Intersect two spheres to get circle. The circle is represented
    /// by it's center and two orthogonal axes in it's</summary>
    /// <param name="a">the first sphere center.</param>
    /// <param name="b">the second sphere center.</param>
    /// <param name="radius">the radius of spheres.</param>
    /// <returns>The circle representation.</returns>
    public static (float Radius, Vector3 Center, Vector3 AxisV, Vector3 AxisU)?
    SphereIntersection2(Vector3 a, Vector3 b, float radius)
    {
        // The circle if present will be located at the center between centers
        // of the spheres. The plane of such circle will be orthogonal to the
        // connector vector.

        // Calculate the vector to connect centers of spheres.
        Vector3 ab = b - a;
        float halfDistance = ab.Length() / 2;
        // If the sphere radius is less than the half of the distance, there is
        // no intersection.
        if (halfDistance > radius)
        {
            return null;
        }
        // Calculate the direction along the ab vector.
        Vector3 abNorm = ab.Normalized();
        // Get any orthogonal vector to the direction vector.
        Vector3 AxisV = OrthogonalVector(abNorm).Normalized();
        // Use cross product to calculate the second axis vector.
        Vector3 AxisU = abNorm.Cross(AxisV).Normalized();
        // Get circle radius.
        Vector3 center = a + ab / 2;
        // Praise the Pythagoras.
        float circeRadius = Mathf.Sqrt(
            radius * radius -
            halfDistance * halfDistance
        );
        return (circeRadius, center, AxisV, AxisU);
    }

    /// <summary>Calculate orthogonal vector to the given one. Does not check
    /// the provided vector for zero length.</summary>
    /// <param name="vector">the vector to calculate the orthogonal to.</param>
    /// <returns>The orthogonal vector.</returns>
    public static Vector3 OrthogonalVector(Vector3 vector)
    {
        // The calculation is based on the idea that if v1 and v2 are
        // orthogonal, then their dot product has to be zero. That way if we
        // multiply given and calculate vectors we will get `ab + ac+ bc - ab -
        // ac - bc`.
        float a = vector.x;
        float b = vector.y;
        float c = vector.z;
        return new Vector3(
            b + c,
            c - a,
            -a - b
        );
    }

    /// <summary>Calculate angles of the sphere-circle intersection points in
    /// to the circle center. The angle to the axisU is zero, the angle to the
    /// axisU is 90 degrees.</summary>
    /// <param name="radius">the sphere radius.</param>
    /// <param name="sphereCenter">the sphere center position.</param>
    /// <param name="axisU">the first circle axis.</param>
    /// <param name="axisV">the second circle axis.</param>
    /// <param name="circleRadius">the circle radius.</param>
    /// <returns>The nullable solution tuple, in radians.<returns>
    public static (float SolutionA, float SolutionB)?
    SphereCircleIntersectionAngles(
        float radius,
        Vector3 sphereCenter,
        Vector3 circleCenter,
        Vector3 axisU,
        Vector3 axisV,
        float circleRadius
    )
    {
        // Resolve the problem in the polynominal form.
        Vector3 delta = circleCenter - sphereCenter;
        float gamma = radius * radius -
            delta.LengthSquared() -
            circleRadius * circleRadius;
        float alpha = 2 * delta.Dot(axisU) * circleRadius;
        float beta = 2 * delta.Dot(axisV) * circleRadius;

        return SolveEquation(alpha, beta, gamma);
    }

    /// <summary>Solve specific type of polynominal equation for the
    /// sphere-circle intersection case.</summary>
    /// <param name="alpha">the first equation argument.</param>
    /// <param name="beta">the second equation argument.</param>
    /// <param name="gamma">the third equation argument.</param>
    /// <returns>Two solutions in radians.</returns>
    public static (float SolutionA, float SolutionB)? SolveEquation(
        float alpha,
        float beta,
        float gamma
    )
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
