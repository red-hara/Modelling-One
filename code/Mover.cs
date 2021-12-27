using Godot;

/// <summary>The mover moves any <c>Positionable</c> object.</summary>
public class Mover : Node
{

    /// <summary>The offset applied to the hard-coded trajectory.</summary>
    [Export]
    public Vector3 offset = new Vector3();

    /// <summary>The path to the positionable in the scene graph.</summary>
    [Export]
    public NodePath positionablePath;

    /// <summary>The positionable object to be controlled.</summary>
    private Positionable positionable;

    /// <summary>Internal counter to measure time from the start.</summary>
    private float counter = 0;

    // Ready is called automatically when the object with this script is added
    // to the scene or the scene is initialized.
    public override void _Ready()
    {
        // Set positionable using the path to it.
        positionable = GetNode<Positionable>(positionablePath);
    }

    // Process is called automatically during every scene graph update. The
    // delta parameter represents the time passed since the previous update in
    // seconds.
    public override void _Process(float delta)
    {
        // Increment the counter;
        counter += delta;
        // Calculate the rotation radius.
        float radius = 200 * Mathf.Cos(counter / 4);
        // Set the desired position.
        positionable.SetPosition(
            radius * new Vector3(
                Mathf.Cos(counter),
                Mathf.Sin(counter),
                0
            ) + offset
        );
    }
}
