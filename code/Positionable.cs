using Godot;
using System;

// Interface to set spatial position.
public interface Positionable
{
    // Function establishing a position.
    bool SetPosition(Vector3 position);
}
