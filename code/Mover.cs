using Godot;
using System;

public class Mover : Node
{
    [Export]
    public Vector3 offset = new Vector3();

    [Export]
    public NodePath positionablePath;
    private Positionable positionable;

    private float counter = 0;

    public override void _Ready()
    {
        positionable = GetNode<Positionable>(positionablePath);
    }

    public override void _Process(float delta)
    {
        counter += delta;
        float radius = 200 * Mathf.Cos(counter / 4);
        positionable.SetPosition(
            radius * new Vector3(Mathf.Cos(counter), Mathf.Sin(counter), 0) +
            offset
        );
    }
}
