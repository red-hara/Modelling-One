using Godot;

/// <summary>Delta-robot controller and inverse kinematics solver.</summary>
[Tool] // Mark this script to be executed in the editor.
public class ControlledDelta : Spatial, Positionable
{

    /// <summary>Path to the controlled delta-robot in the scene
    /// graph.</summary>
    [Export] // Export this field to the editor Inspector.
    public NodePath deltaPath;

    /// <summary>An instance of the Delta robot pointed by the
    /// <c>deltaPath</c>.</summary>
    private Delta robot;


    /// <summary>Spatial target to "teleport" the robot to.</summary>
    [Export]
    public Vector3 target
    {
        // Marks that getting and setting the robot position actions are
        // performed by various methods.
        get
        {
            return robot.Position;
        }
        set
        {
            SetPosition(value);
        }
    }

    /// <summary>Maximum acceptable positive degree value of controllable robot
    /// joints.</summary>
    [Export]
    public float axisLimitPositive = 10;

    /// <summary>Minimum acceptable positive degree value of controllable robot
    /// joints.</summary>
    [Export]
    public float axisLimitNegative = -120;

    // Ready is called automatically when the object with this script is added
    // to the scene or the scene is initialized.
    public override void _Ready()
    {
        robot = GetNode<Delta>(deltaPath);
    }

    /// <summary>Calculate optional inverse kinematics solution for the given
    /// target.</summary>
    /// <param name="target">the target position to solve the inverse kinematics
    /// for.</param>
    /// <returns>Optional solution tuple.</returns>
    private (float AxisA, float AxisB, float AxisC)? Inverse(Vector3 target)
    {
        // Get solutions for individual axes.
        float? a = InverseAxis(target, 0);
        float? b = InverseAxis(target, 1);
        float? c = InverseAxis(target, 2);
        // Check that all values are not null, i.e., there are solutions for all
        // three axes.
        if (a is null || b is null || c is null)
        {
            return null;
        }
        // Recalculate solution values from radians to degrees, also extract
        // them from the nullable type.
        float axisA = Mathf.Rad2Deg(a.Value);
        float axisB = Mathf.Rad2Deg(b.Value);
        float axisC = Mathf.Rad2Deg(c.Value);
        // Check that all values are in proper range.
        if (axisA > axisLimitPositive || axisA < axisLimitNegative)
        {
            return null;
        }
        if (axisB > axisLimitPositive || axisB < axisLimitNegative)
        {
            return null;
        }
        if (axisC > axisLimitPositive || axisC < axisLimitNegative)
        {
            return null;
        }
        return (axisA, axisB, axisC);
    }

    /// <summary>Solve the inverse kinematics problem for an individual
    /// axis.</summary>
    /// <param name="position">the target position to solve the inverse
    /// kinematics for.</param>
    /// <param name="index">the index of the joint on the robot.<param>
    /// <returns>The axis value if the <c>target</c> is reachable.</returns>
    private static float? InverseAxis(Vector3 position, int index)
    {
        // The solution is base on the idea that with fixed platform position
        // the ends of connectors (grey parts) form sphere. Such sphere has to
        // intersect with the circles on which the end of arms (black parts) are
        // located.

        // Calculate the joint axis rotation around the Z axis.
        Quat rotation = new Quat(Vector3.Back, Mathf.Pi * 2 / 3 * index);
        // Set the first circle axis, the vertical one.
        Vector3 axisU = Vector3.Forward;
        // Set the second circle axis, the horizontal one. It is affected by the
        // rotation of the joint axis.
        Vector3 axisV = rotation.Xform(Vector3.Left);
        // Set the circle center in the center of the joint.
        Vector3 circleCenter = rotation.Xform(
            new Vector3(
                Delta.baseRadius - Delta.platformRadius,
                0,
                Delta.baseLift
            )
        );
        float circleRadius = Delta.armLength;
        var maybeSolution = Delta.SphereCircleIntersectionAngles(
            Delta.connectorRadius,
            position,
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

        // Select better solution.
        if (
            Mathf.Wrap(solution.SolutionA, -Mathf.Pi, Mathf.Pi) <
            Mathf.Wrap(solution.SolutionB, -Mathf.Pi, Mathf.Pi)
        )
        {
            return solution.SolutionA;
        }
        return solution.SolutionB;
    }

    /// <summary>Set robot spatial position.</summary>
    /// <param name="position">the target robot position.</param>
    /// <returns>True if the position setting was successful.</returns>
    public bool SetPosition(Vector3 position)
    {
        var maybeSolution = Inverse(position);
        if (!(maybeSolution is null) && !(robot is null))
        {
            var solution = maybeSolution ?? (0, 0, 0);
            robot.axisA = solution.AxisA;
            robot.axisB = solution.AxisB;
            robot.axisC = solution.AxisC;
            return true;
        }
        return false;
    }
}
