using Godot;

[Tool]
public class Lamp : Spatial
{
    [Export] // Export this field to the editor Inspector. 
    public float q1; // You'll need more generalized coordinates.

    [Export]
    public NodePath columnPath; // You'll need paths to each controlled part.

    public override void _Process(float delta)
    {
        // Extract column spatial based on the path.
        Spatial column = GetNode<Spatial>(columnPath);
        // Set it's rotation degrees (Y, X, Z order).
        column.RotationDegrees = new Vector3(0, q1, 0);
    }
}
