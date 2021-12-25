using Godot;
using System;

[Tool]
public class Lamp : Spatial
{
    [Export]
    public float q1;

    [Export]
    public NodePath columnPath;

    public override void _Process(float delta)
    {
        Spatial column = GetNode<Spatial>(columnPath);
        column.RotationDegrees = new Vector3(0, q1, 0);
    }
}