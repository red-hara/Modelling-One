using Godot;
using System;

// Class setting hand position.
public class Mover : Node
{

    // Adding offset points (x,y,z) to the "Script Variables" for the position of the manipulator with parallel kinematics.
    [Export]
    public Vector3 offset = new Vector3();

    // Adding a positional path to the "Script Variables" in order to providing the 
    // manipulator and control its position using coordinates.
    [Export]
    public NodePath positionablePath;

    // A positional object is being created.
    private Positionable positionable;
    
    // A variable "counter" of the float type is created.
    private float counter = 0;

    // Function for the possibility of installation "positionable".
    public override void _Ready()
    {

        positionable = GetNode<Positionable>(positionablePath);
    }

    // The function of the movement of the manipulator around the circle.
    // Accepts the <delta> parameter, which provides a frame-by-frame image of the movement.
    public override void _Process(float delta)
    {
        counter += delta;
        float radius = 200 * Mathf.Cos(counter / 4);
        positionable.SetPosition(radius * new Vector3(Mathf.Cos(counter), Mathf.Sin(counter), 0) + offset);
    }
}
