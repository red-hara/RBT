using Godot;
using System;

public class HotkeyLauncher : Node
{
    public override void _Ready()
    {

    }

    public override void _Process(float delta)
    {
        if (Input.IsKeyPressed((int)KeyList.R) && Input.IsKeyPressed((int)KeyList.Control)) {
            EmitSignal(nameof(Start));
        }
    }

    [Signal]
    delegate void Start();
}
