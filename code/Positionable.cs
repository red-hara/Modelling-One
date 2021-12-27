using Godot;

/// <summary><c>Positionable</c> instance can be told to move it's end-effector
/// to specific spatial position in relation to it's zero frame.</summary>
// The interface is a common behavior promice, i.e., one can ask any
// `Positionable` object ot `SetPosition` and hope that it will behave
// correctly.
public interface Positionable
{
    /// <summary>Try seting the end-effector position.</summary>
    /// <param name="position">the spatial position in relation to the
    /// <c>Positionable</c> zero frame.</param>
    /// <returns><c>true</c> if the operation was successfull.</returns>
    bool SetPosition(Vector3 position);
}
