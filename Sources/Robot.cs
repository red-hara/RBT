using Godot;
using System;

[Tool]
public class Robot : Node
{
	public Spatial[] bones = { null, null, null, null, null, null, null };

	[Export]
	private NodePath bone0;

	[Export]
	private NodePath bone1;

	[Export]
	private NodePath bone2;

	[Export]
	private NodePath bone3;

	[Export]
	private NodePath bone4;

	[Export]
	private NodePath bone5;

	[Export]
	private NodePath bone6;

	[Export]
	public float[] q = {0, 0, 0, 0, 0, 0};

	[Export]
	public NodePath flangePath;
	public Spatial flange;

	[Export]
	public NodePath controllerPath;
	public Controller controller;

	[Export]
	public NodePath tcpPath;
	public Spatial tcp;

	[Export]
	public NodePath originPath;
	public Spatial origin;


	public override void _Ready()
	{
		bones[0] = GetNode(bone0) as Spatial;
		bones[1] = GetNode(bone1) as Spatial;
		bones[2] = GetNode(bone2) as Spatial;
		bones[3] = GetNode(bone3) as Spatial;
		bones[4] = GetNode(bone4) as Spatial;
		bones[5] = GetNode(bone5) as Spatial;
		bones[6] = GetNode(bone6) as Spatial;
		flange = GetNode(flangePath) as Spatial;
		controller = GetNode(controllerPath) as Controller;
		tcp = GetNode(tcpPath) as Spatial;
		origin = GetNode(originPath) as Spatial;
	}

	public override void _Process(float delta)
	{
		ImplementJoints();
	}

	private void ImplementJoints()
	{
		bones[1].RotationDegrees = new Vector3(0, 0, q[0]);
		bones[2].RotationDegrees = new Vector3(0, q[1], 0);
		bones[3].RotationDegrees = new Vector3(0, q[2], 0);
		bones[4].RotationDegrees = new Vector3(q[3], 0, 0);
		bones[5].RotationDegrees = new Vector3(0, q[4], 0);
		bones[6].RotationDegrees = new Vector3(q[5], 0, 0);
	}
}
