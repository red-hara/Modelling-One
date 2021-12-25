using Godot;
using System;

// A class controlling a delta robot.

[Tool]
public class ControlledDelta : Spatial, Positionable
{
    // Adding "deltaPath" to the "Script Variables", to fix the
    // robot, which controls the settings of the work.
    [Export]
    public NodePath deltaPath;
    // An object of the "delta" type is being created.
    private Delta robot;

    // Purpose of the function execution.
    // GET-Setting the robot's position.SET-The position value is set.
    [Export]
    public Vector3 target
    {
        get
        {
            return robot.Position;
        }
        set
        {
            SetPosition(value);
        }
    }

    // Setting the "Axis Limit Positive".
    [Export]
    public float axisLimitPositive = 10;

    // Setting the "Axis Limit Negative".
    [Export]
    public float axisLimitNegative = -120;

    // Function for the possibility of installation "delta".
    public override void _Ready()
    {
        robot = GetNode<Delta>(deltaPath);
    }

    // Error exclusion function, when going beyond the permissible limits.
    // Accepts the <target> parameter.
    private (float AxisA, float AxisB, float AxisC)? Inverse(Vector3 target)
    {
        float? a = InverseAxis(target, 0);
        float? b = InverseAxis(target, 1);
        float? c = InverseAxis(target, 2);
        if (a is null || b is null || c is null)
        {
            return null;
        }
        float axisA = Mathf.Rad2Deg(a ?? default);
        float axisB = Mathf.Rad2Deg(b ?? default);
        float axisC = Mathf.Rad2Deg(c ?? default);
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

    // Solve inverse kinematics problem for individual axis.
    // Accepted parameters <Vector3 position> and <int index>
    private static float? InverseAxis(Vector3 position, int index)
    {
        Quat rotation = new Quat(Vector3.Back, Mathf.Pi * 2 / 3 * index);
        Vector3 axisU = Vector3.Forward;
        Vector3 axisV = rotation.Xform(Vector3.Left);
        Vector3 circleCenter = rotation.Xform(new Vector3(Delta.baseRadius - Delta.platformRadius, 0, Delta.baseLift));
        float circleRadius = Delta.armLength;
        var maybeSolution = Delta.SphereCircleIntersectionAngles(Delta.connectorRadius, position, circleCenter, axisU, axisV, circleRadius);
        if (maybeSolution is null)
        {
            return null;
        }
        var solution = maybeSolution ?? default;
        if (Mathf.Wrap(solution.SolutionA, -Mathf.Pi, Mathf.Pi) < Mathf.Wrap(solution.SolutionB, -Mathf.Pi, Mathf.Pi))
        {
            return solution.SolutionA;
        }
        return solution.SolutionB;
    }

    // The function of setting the position of the axes of the manipulator.
    // Accepts the <Vector3 position> parameter.
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
