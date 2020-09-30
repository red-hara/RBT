using Godot;
using System;
using System.Collections.Generic;

[Tool]
public class Controller : Node
{
    private const float maximum_angular = 180;

    private const float l1 = 400;
    private const float p2 = 25;
    private const float l2 = 455;
    private const float l3 = 420;
    private const float p3 = 25;
    private const float l4 = 90;

    private float[] max = { 170, 45 + 90, 156 - 90, 185, 120, 350 };
    private float[] min = { 170, 100, 120 + 90, 185, 120, 350 };

    [Export]
    public float[] q = { 0, 0, 0, 0, 0, 0 };

    [Export]
    public int[] i = { 1, 1, 1 };

    [Export]
    private NodePath robot;
    private Robot _robot;

    public Position tcp = new Position();

    public bool isMoving = false;

    private Interpolation currentInterpolation = null;

    [Export]
    public bool run;

    public bool finishedWithError = false;

    public String currentError = null;

    public override void _Ready()
    {
        _robot = GetNode(robot) as Robot;
    }

    public override void _Process(float delta)
    {
        _robot.tcp.Transform = new Transform(tcp.rot, tcp.pos);

        if (currentInterpolation != null)
        {
            currentInterpolation.Step(delta);
            currentInterpolation.Apply(this);
        }
        else
        {
            isMoving = false;
        }

        Array.Copy(this.q, _robot.q, 6);
    }

    public void Move(MotionCommand motion)
    {
        currentError = null;
        isMoving = true;
        LinearCommand lin = motion as LinearCommand;
        if (lin != null)
        {
            i = CalculateI(q);
            Position current = CurrentPosition(q);
            currentInterpolation = new LinearInterpolation(
                current,
                lin.target.AsPosition(current),
                lin.velocity * 1000.0f,
                i
            );
        }

        JointCommand jnt = motion as JointCommand;
        if (jnt != null)
        {
            try
            {
                float[] target = jnt.target.AsArray(q);
                VerifySolution(target);

                currentInterpolation = new JointInterpolation(
                    q,
                    jnt.target.AsArray(q),
                    jnt.velocity
                );
            }
            catch (Exception ex)
            {
                currentError = ex.Message;
                finishedWithError = true;
            }
        }

        PtpCommand ptp = motion as PtpCommand;
        if (ptp != null)
        {
            try
            {
                i = CalculateI(q);
                float[] sol = Solve(ptp.target.AsPosition(
                    CurrentPosition(q)),
                    ptp.target.i0(i[0]),
                    ptp.target.i1(i[1]),
                    ptp.target.i2(i[2])
                    ).AsArray();
                currentInterpolation = new JointInterpolation(
                    q,
                    sol,
                    ptp.velocity
            );
            }
            catch (Exception ex)
            {
                currentError = ex.Message;
                finishedWithError = true;
            }

        }
    }

    public void Stop()
    {
        currentInterpolation = null;
    }

    public Position CurrentPosition(float[] q)
    {
        Position p0 = new Position(new Vector3(0, 0, l1), new Quat(Vector3.Back, Mathf.Deg2Rad(q[0])));
        Position p1 = new Position(new Vector3(Controller.p2, 0, 0), new Quat(Vector3.Up, Mathf.Deg2Rad(q[1])));
        Position p2 = new Position(new Vector3(0, 0, l2), new Quat(Vector3.Up, Mathf.Deg2Rad(q[2])));
        Position p3 = new Position(new Vector3(l3, 0, Controller.p3), new Quat(Vector3.Right, Mathf.Deg2Rad(q[3])));
        Position p4 = new Position(new Vector3(0, 0, 0), new Quat(Vector3.Up, Mathf.Deg2Rad(q[4])));
        Position p5 = new Position(new Vector3(l4, 0, 0), new Quat(Vector3.Right, Mathf.Deg2Rad(q[5])));
        Position p6 = new Position(new Vector3(0, 0, 0), new Quat(Vector3.Up, Mathf.Pi / 2) * new Quat(Vector3.Back, Mathf.Pi / 2));
        Position result = p0 * p1 * p2 * p3 * p4 * p5 * p6 * tcp;

        return result;
    }

    public int[] CalculateI(float[] q)
    {
        Position p0 = new Position(new Vector3(0, 0, l1), new Quat(Vector3.Back, 0));
        Position p1 = new Position(new Vector3(Controller.p2, 0, 0), new Quat(Vector3.Up, Mathf.Deg2Rad(q[1])));
        Position p2 = new Position(new Vector3(0, 0, l2), new Quat(Vector3.Up, Mathf.Deg2Rad(q[2])));
        Position p3 = new Position(new Vector3(l3, 0, Controller.p3), new Quat(Vector3.Right, 0));
        Position r = p0 * p1 * p2 * p3;
        int[] result = { 1, 1, 1 };
        if (r.pos.x < 0)
        {
            result[0] *= -1;
        }
        float q3a = 0.5f * Mathf.Pi - Mathf.Atan2(Controller.l3, Controller.p3);
        q3a = Mathf.Rad2Deg(q3a);
        if ((q[2] + 90 - q3a) * result[0] < 0)
        {
            result[1] *= -1;
        }
        if (q[4] < 0)
        {
            result[2] *= -1;
        }
        GD.Print(result[0], result[1], result[2]);
        return result;
    }

    public Solution Solve(Position target, int i0, int i1, int i2)
    {
        GD.Print(i0, i1, i2);
        target = target * tcp.Inverse();
        Position fifth = target * new Position(new Vector3(0, 0, -l4), Quat.Identity);
        Vector3 projection = new Vector3(fifth.pos.x, fifth.pos.y, 0);
        float q1 = AngleTo(Vector3.Right, projection, Vector3.Back) + Mathf.Pi / 2.0f - Mathf.Pi / 2.0f * i0;
        Vector3 fifthPos = fifth.pos - new Vector3(0, 0, l1);
        float projectionD = Mathf.Sqrt(fifthPos.x * fifthPos.x + fifthPos.y * fifthPos.y) - p2 * i0;
        float d = Mathf.Sqrt(projectionD * projectionD + fifthPos.z * fifthPos.z);
        float l = Mathf.Sqrt(p3 * p3 + l3 * l3);
        float q3a = 0.5f * Mathf.Pi - Mathf.Atan2(l3, p3);
        float q3 = q3a - Mathf.Acos((l2 * l2 + l * l - d * d) / (2.0f * l2 * l)) * i1 * i0 - 3.0f * Mathf.Pi / 2;
        float q2 = i0 * -(i1 * Mathf.Acos((l2 * l2 + d * d - l * l) / (2.0f * l2 * d)) + Mathf.Atan2(fifthPos.z, projectionD) - Mathf.Pi / 2.0f);

        if (Mathf.IsNaN(q2) || Mathf.IsNaN(q3))
        {
            throw new SolutionException("Workzone error");
        }

        Quat quat0 = new Quat(Vector3.Back, q1);
        Quat quat1 = new Quat(Vector3.Up, q2);
        Quat quat2 = new Quat(Vector3.Up, q3);
        Quat quat = quat0 * quat1 * quat2;

        Vector3 forthX = quat.Xform(Vector3.Right);
        Vector3 forthY = quat.Xform(Vector3.Up);
        Vector3 targZ = target.rot.Xform(Vector3.Back);
        Vector3 fifthY = forthX.Cross(targZ);
        Vector3 targX = target.rot.Xform(Vector3.Right);
        float q4 = AngleTo(forthY, fifthY, forthX) + Mathf.Pi / 2 - Mathf.Pi / 2 * i2;
        float q5 = AngleTo(forthX, targZ, fifthY) * i2;
        float q6 = AngleTo(fifthY, targX, targZ) + Mathf.Pi / 2 - Mathf.Pi / 2 * i2;

        float[] sol = { Mathf.Rad2Deg(q1), Mathf.Rad2Deg(q2), Mathf.Rad2Deg(q3), Mathf.Rad2Deg(q4), Mathf.Rad2Deg(q5), Mathf.Rad2Deg(q6) };

        for (int i = 0; i < 6; i++)
        {
            sol[i] = Mathf.Wrap(sol[i], -180, 180);
        }
        VerifySolution(sol);

        return new Solution(sol);
    }

    public void VerifySolution(float[] sol)
    {
        for (int i = 0; i < 6; i++)
        {
            if (sol[i] > max[i])
            {
                throw new SolutionException("Axis A" + (i + 1) + " out of limit: " + sol[i]);
            }
            if (sol[i] < -min[i])
            {
                throw new SolutionException("Axis -A" + (i + 1) + " out of limit: " + sol[i]);
            }
        }
    }

    public static float AngleTo(Vector3 v0, Vector3 v1, Vector3 n)
    {
        Vector3 cross = v0.Cross(v1);
        float angle = v0.AngleTo(v1);
        return cross.Dot(n) > 0 ? angle : -angle;
    }

    public static float AngleTo(Quat from, Quat to)
    {
        float result = Mathf.Acos((from.Inverse() * to).Normalized().w) * 2.0f;
        return result;
    }

    public class Solution
    {
        public float q1;
        public float q2;
        public float q3;
        public float q4;
        public float q5;
        public float q6;

        public Solution(float q1, float q2, float q3, float q4, float q5, float q6)
        {
            this.q1 = q1;
            this.q2 = q2;
            this.q3 = q3;
            this.q4 = q4;
            this.q5 = q5;
            this.q6 = q6;
        }

        public Solution(float[] q)
        {
            this.q1 = q[0];
            this.q2 = q[1];
            this.q3 = q[2];
            this.q4 = q[3];
            this.q5 = q[4];
            this.q6 = q[5];
        }

        public float[] AsArray()
        {
            return new float[] { q1, q2, q3, q4, q5, q6 };
        }
    }

    public class SolutionException : Exception
    {
        public SolutionException(String reason) : base(reason) { }
    }

    [Serializable]
    public class MotionCommand { }

    [Serializable]
    public class LinearCommand : MotionCommand
    {
        public CartesianTarget target = new CartesianTarget();
        public float velocity = 0.5f;
    }

    [Serializable]
    public class PtpCommand : MotionCommand
    {
        public CartesianTarget target = new CartesianTarget();
        public float velocity = 1.0f;
    }

    [Serializable]
    public class JointCommand : MotionCommand
    {
        public JointTarget target = new JointTarget();
        public float velocity = 1.0f;
    }

    [Serializable]
    public class CircularCommand : MotionCommand
    {
        public CartesianTarget inter;
        public CartesianTarget target;
    }

    [Serializable]
    public class Target { }

    [Serializable]
    public class JointTarget : Target
    {
        public Dictionary<Axis, float> q = new Dictionary<Axis, float>();

        public float[] AsArray(float[] current)
        {
            float[] result = new float[6];
            Array.Copy(current, result, 6);
            if (q.ContainsKey(Axis.A1)) { result[0] = q[Axis.A1]; }
            if (q.ContainsKey(Axis.A2)) { result[1] = q[Axis.A2]; }
            if (q.ContainsKey(Axis.A3)) { result[2] = q[Axis.A3]; }
            if (q.ContainsKey(Axis.A4)) { result[3] = q[Axis.A4]; }
            if (q.ContainsKey(Axis.A5)) { result[4] = q[Axis.A5]; }
            if (q.ContainsKey(Axis.A6)) { result[5] = q[Axis.A6]; }
            return result;
        }
    }

    [Serializable]
    public enum Axis
    {
        A1,
        A2,
        A3,
        A4,
        A5,
        A6
    }

    [Serializable]
    public class CartesianTarget : Target
    {
        public int i = -1;
        public Dictionary<CartesianAxis, float> t = new Dictionary<CartesianAxis, float>();

        public Position AsPosition(Position current = null)
        {
            if (current == null)
            {
                current = new Position();
            }
            Vector3 pos = current.pos;
            Vector3 euler = Quat2Abc(current.rot);
            if (t.ContainsKey(CartesianAxis.X)) { pos.x = t[CartesianAxis.X]; }
            if (t.ContainsKey(CartesianAxis.Y)) { pos.y = t[CartesianAxis.Y]; }
            if (t.ContainsKey(CartesianAxis.Z)) { pos.z = t[CartesianAxis.Z]; }
            if (t.ContainsKey(CartesianAxis.A)) { euler.z = Mathf.Deg2Rad(t[CartesianAxis.A]); }
            if (t.ContainsKey(CartesianAxis.B)) { euler.y = Mathf.Deg2Rad(t[CartesianAxis.B]); }
            if (t.ContainsKey(CartesianAxis.C)) { euler.x = Mathf.Deg2Rad(t[CartesianAxis.C]); }
            Quat rot = Abc2Quat(euler);
            return new Position(pos, rot);
        }

        public int i0(int def)
        {
            if (i == -1)
            {
                return def;
            }
            return ((i & 0b100) == 0b100) ? 1 : -1;
        }

        public int i1(int def)
        {
            if (i == -1)
            {
                return def;
            }
            return ((i & 0b010) == 0b010) ? 1 : -1;
        }

        public int i2(int def)
        {
            if (i == -1)
            {
                return def;
            }
            return ((i & 0b001) == 0b001) ? 1 : -1;
        }
    }

    [Serializable]
    public enum CartesianAxis
    {
        X,
        Y,
        Z,
        A,
        B,
        C,
        I,
    }

    [Serializable]
    public class Tool
    {
        public Position tcp;
    }

    private class Interpolation
    {
        public float t = 0;
        public virtual void Step(float delta)
        {
            t = Mathf.Min(t + delta, 1.0f);
        }

        public virtual void Apply(Controller controller)
        {
            if (t >= 1.0f)
            {
                controller.currentInterpolation = null;
            }
        }
    }

    private class LinearInterpolation : Interpolation
    {
        public Position start;
        public Position end;
        public float k;
        public Position current;
        public int i0;
        public int i1;
        public int i2;

        public LinearInterpolation(Position start, Position end, float velocity, int[] i)
        {
            this.start = start;
            this.end = end;
            this.current = start;
            this.start.rot = this.start.rot.Normalized();
            this.end.rot = this.end.rot.Normalized();
            i0 = i[0];
            i1 = i[1];
            i2 = i[2];

            float distance = (end.pos - start.pos).Length();
            float angle = Mathf.Abs(Mathf.Wrap(Mathf.Rad2Deg(AngleTo(start.rot, end.rot)), -180, 180));

            this.k = Mathf.Max(distance / velocity, angle / maximum_angular);
        }

        public override void Step(float delta)
        {
            base.Step(delta / k);
            Vector3 pos = start.pos.LinearInterpolate(end.pos, t);
            Quat rot = start.rot.Slerp(end.rot, t);
            current = new Position(pos, rot);
        }

        public override void Apply(Controller controller)
        {
            base.Apply(controller);
            try
            {
                Solution solution = controller.Solve(current, i0, i1, i2);
                controller.q = solution.AsArray();
            }
            catch (SolutionException ex)
            {
                controller.currentError = ex.Message;
                controller.currentInterpolation = null;
                controller.finishedWithError = true;
            }
        }
    }

    private class JointInterpolation : Interpolation
    {
        public float[] start;
        public float[] end;
        public float k;
        public float[] current;

        public JointInterpolation(float[] start, float[] end, float velocity)
        {
            this.start = start;
            this.end = end;
            this.current = new float[6];

            this.k = 1.0f / velocity;
        }

        public override void Step(float delta)
        {
            base.Step(delta / k);
            for (int i = 0; i < 6; i++)
            {
                current[i] = start[i] + (end[i] - start[i]) * t;
            }
        }

        public override void Apply(Controller controller)
        {
            base.Apply(controller);
            controller.q = current;
        }
    }

    [Serializable]
    public class Position
    {
        public Vector3 pos;
        public Quat rot;

        public Position()
        {
            this.pos = new Vector3(0, 0, 0);
            this.rot = Quat.Identity;
        }

        public Position(Vector3 pos, Quat rot)
        {
            this.pos = pos;
            this.rot = rot;
        }

        public Position(Transform transform)
        {
            this.pos = transform.origin;
            this.rot = transform.basis.Quat().Normalized();
        }

        public Position Inverse()
        {
            return new Position(
                rot.Inverse().Xform(-pos),
                rot.Inverse()
            );
        }

        public static Position operator *(Position position0, Position position1)
        {
            return new Position(
                position0.pos + position0.rot.Xform(position1.pos),
                position0.rot * position1.rot
            );
        }
    }

    public static Vector3 Quat2Abc(Quat q)
    {
        q = q.Normalized();
        float a = Mathf.Atan2(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
        float b = Mathf.Asin(Mathf.Max(Mathf.Min(2.0f * (q.w * q.y - q.x * q.z), 1.0f), -1.0f));
        float c = Mathf.Atan2(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y));
        return new Vector3(c, b, a);
    }

    public static Quat Abc2Quat(Vector3 v)
    {
        return
            new Quat(new Vector3(0, 0, 1), v.z) *
            new Quat(new Vector3(0, 1, 0), v.y) *
            new Quat(new Vector3(1, 0, 0), v.x);
    }
}
