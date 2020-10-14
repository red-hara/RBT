using Godot;
using System;
using System.Reflection;
using System.Collections.Generic;
using NLua;

public class Executor : Node
{
    [Export]
    public NodePath robot;
    private Controller controller;

    [Export]
    public NodePath positionLabel;
    private Label _positionLabel;

    [Export]
    public NodePath errorLabel;
    private Label _errorLabel;

    [Export]
    public NodePath text;
    private TextEdit _text;

    private AppDomain appDomain = null;
    private Worker worker = null;

    private bool shallNotify = false;

    public override void _Ready()
    {
        _text = GetNode(text) as TextEdit;
        controller = (GetNode(robot) as Robot).controller;
        _positionLabel = GetNode(positionLabel) as Label;
        _errorLabel = GetNode(errorLabel) as Label;
    }

    public override void _Notification(int notificationCode)
    {
        if (notificationCode == MainLoop.NotificationWmQuitRequest)
        {
            Abort();
            GetTree().Quit();
        }
    }

    public override void _Process(float delta)
    {
        if (worker != null)
        {
            controller.tcp = worker.robot.currentTool;
            controller.origin = worker.robot.currentOrigin;
            if (worker.robot.currentCommand != null && !controller.finishedWithError)
            {
                controller.Move(worker.robot.currentCommand);
                worker.robot.SetCurrentCommand(null);
                shallNotify = true;
            }
            else if (!controller.isMoving && shallNotify)
            {
                shallNotify = false;
                worker.robot.doneCommand = true;
                worker.robot.currentPosition = controller.CurrentPosition(controller.q);
            }

            if (worker.error != null)
            {
                _errorLabel.Text = worker.error;
            }
            else if (controller.currentError != null)
            {
                _errorLabel.Text = controller.currentError;
            } else {
                _errorLabel.Text = "";
            }

            if (worker.doneEverything || controller.finishedWithError)
            {
                Abort();
            }
        }
        Controller.Position currentPosition = controller.CurrentPosition(controller.q);
        Vector3 cba = Controller.Quat2Abc(currentPosition.rot);
        Vector3 xyz = currentPosition.pos;
        _positionLabel.Text =
            "{X = " +  Round2(xyz.x) +
            ", Y = " + Round2(xyz.y) +
            ", Z = " + Round2(xyz.z) +
            ", A = " + Round2(Mathf.Rad2Deg(cba.z)) +
            ", B = " + Round2(Mathf.Rad2Deg(cba.y)) +
            ", C = " + Round2(Mathf.Rad2Deg(cba.x)) + "}";
    }

    public static float Round2(float a) {
        return Mathf.Round(a * 100) / 100.0f;
    } 

    public void Run()
    {
        if (appDomain == null)
        {
            controller.finishedWithError = false;

            AppDomainSetup ads = new AppDomainSetup();
            ads.ApplicationBase = AppDomain.CurrentDomain.BaseDirectory;

            string exeAssembly = Assembly.GetExecutingAssembly().FullName;
            string callingDomainName = System.Threading.Thread.GetDomain().FriendlyName;

            AppDomain ad2 = AppDomain.CreateDomain("AD #2", null, ads);
            worker =
                (Worker)ad2.CreateInstanceAndUnwrap(
                    exeAssembly,
                    typeof(Worker).FullName
                );
            worker.Start(_text.Text);
            appDomain = ad2;
        }
    }

    public void Abort()
    {
        if (appDomain != null)
        {
            AppDomain.Unload(appDomain);
            appDomain = null;
            worker = null;
        }
        controller.Stop();
    }

    [Serializable]
    private class Worker : MarshalByRefObject
    {

        public bool doneEverything = false;
        public Rbt robot = new Rbt();
        public String error = null;

        public void Start(String code)
        {
            System.Threading.Thread thread = new System.Threading.Thread(_Run);
            thread.Start(code);
        }

        private void _Run(object code)
        {
            Lua lua = new Lua();
            robot.currentLua = lua;
            lua["robot"] = robot;
            lua.NewTable("home");
            (lua["home"] as LuaTable)["A1"] = 0.0f;
            (lua["home"] as LuaTable)["A2"] = 0.0f;
            (lua["home"] as LuaTable)["A3"] = 0.0f;
            (lua["home"] as LuaTable)["A4"] = 0.0f;
            (lua["home"] as LuaTable)["A5"] = 90.0f;
            (lua["home"] as LuaTable)["A6"] = 0.0f;
            lua.DoString(
                @"
                function ptp(t, v)
                    v = v or 1
                    robot:ptp(t, v)
                end
                function lin(t, v)
                    v = v or 1
                    robot:lin(t, v)
                end
                function tool(t)
                    robot:tool(t)
                end
                function wait(t)
                    robot:wait(t)
                end
                function current()
                    return robot:current()
                end
                function origin(t)
                    robot:origin(t)
                end
                ");
            try
            {
                lua.DoString(code as string);
            }
            catch (Exception ex)
            {
                GD.Print(ex);
                error = ex.Message;
            }
            doneEverything = true;
        }
    }

    [Serializable]
    private class Rbt : MarshalByRefObject
    {
        public LuaTable cur;
        public Lua currentLua;
        public Controller.Position currentPosition;
        public bool doneCommand = false;
        public Controller.MotionCommand currentCommand = null;
        public Controller.Position currentTool = new Controller.Position();
        public Controller.Position currentOrigin = new Controller.Position();

        public void lin(LuaTable table, float velocity)
        {
            doneCommand = false;
            Controller.LinearCommand command = new Controller.LinearCommand();
            command.velocity = Mathf.Min(Mathf.Max(velocity, 0.001f), 2);
            if (table["X"] != null) { command.target.t[Controller.CartesianAxis.X] = LuaNumber2Float(table["X"]); }
            if (table["Y"] != null) { command.target.t[Controller.CartesianAxis.Y] = LuaNumber2Float(table["Y"]); }
            if (table["Z"] != null) { command.target.t[Controller.CartesianAxis.Z] = LuaNumber2Float(table["Z"]); }
            if (table["A"] != null) { command.target.t[Controller.CartesianAxis.A] = LuaNumber2Float(table["A"]); }
            if (table["B"] != null) { command.target.t[Controller.CartesianAxis.B] = LuaNumber2Float(table["B"]); }
            if (table["C"] != null) { command.target.t[Controller.CartesianAxis.C] = LuaNumber2Float(table["C"]); }
            currentCommand = command;

            while (!doneCommand)
            {
                System.Threading.Thread.Sleep(1);
            }

        }

        public void lin(LuaTable table)
        {
            lin(table, 1000);
        }

        public void ptp(LuaTable table, float velocity)
        {
            bool isJnt = false;
            bool isPtp = false;
            doneCommand = false;
            Controller.JointCommand jnt = new Controller.JointCommand();
            jnt.velocity = Mathf.Min(Mathf.Max(velocity, 0.01f), 1.0f);
            if (table["A1"] != null) { jnt.target.q[Controller.Axis.A1] = LuaNumber2Float(table["A1"]); isJnt = true; }
            if (table["A2"] != null) { jnt.target.q[Controller.Axis.A2] = LuaNumber2Float(table["A2"]); isJnt = true; }
            if (table["A3"] != null) { jnt.target.q[Controller.Axis.A3] = LuaNumber2Float(table["A3"]); isJnt = true; }
            if (table["A4"] != null) { jnt.target.q[Controller.Axis.A4] = LuaNumber2Float(table["A4"]); isJnt = true; }
            if (table["A5"] != null) { jnt.target.q[Controller.Axis.A5] = LuaNumber2Float(table["A5"]); isJnt = true; }
            if (table["A6"] != null) { jnt.target.q[Controller.Axis.A6] = LuaNumber2Float(table["A6"]); isJnt = true; }

            Controller.PtpCommand ptp = new Controller.PtpCommand();
            if (table["X"] != null) { ptp.target.t[Controller.CartesianAxis.X] = LuaNumber2Float(table["X"]); isPtp = true; }
            if (table["Y"] != null) { ptp.target.t[Controller.CartesianAxis.Y] = LuaNumber2Float(table["Y"]); isPtp = true; }
            if (table["Z"] != null) { ptp.target.t[Controller.CartesianAxis.Z] = LuaNumber2Float(table["Z"]); isPtp = true; }
            if (table["A"] != null) { ptp.target.t[Controller.CartesianAxis.A] = LuaNumber2Float(table["A"]); isPtp = true; }
            if (table["B"] != null) { ptp.target.t[Controller.CartesianAxis.B] = LuaNumber2Float(table["B"]); isPtp = true; }
            if (table["C"] != null) { ptp.target.t[Controller.CartesianAxis.C] = LuaNumber2Float(table["C"]); isPtp = true; }
            if (table["I"] != null) { ptp.target.i = LuaNumber2Int(table["I"]); isPtp = true; }

            if (isJnt && isPtp)
            {
                throw new ArgumentException("Both ptp and jnt arguments.");
            }
            if (isJnt)
            {
                currentCommand = jnt;
            }
            else if (isPtp)
            {
                currentCommand = ptp;
            }

            while (!doneCommand)
            {
                System.Threading.Thread.Sleep(1);
            }
        }

        public void ptp(LuaTable table)
        {
            ptp(table, 1.0f);
        }

        public void tool(LuaTable table)
        {
            currentTool = LuaTable2Position(table);
            System.Threading.Thread.Sleep(1);
        }

        public void origin(LuaTable table)
        {
            currentOrigin = LuaTable2Position(table);
            System.Threading.Thread.Sleep(1);
        }

        public LuaTable current()
        {
            currentLua.NewTable("robot.cur");
            LuaTable result = currentLua["robot.cur"] as LuaTable;
            Vector3 cba = Controller.Quat2Abc(currentPosition.rot);
            Vector3 xyz = currentPosition.pos;
            result["X"] = xyz.x;
            result["Y"] = xyz.y;
            result["Z"] = xyz.z;
            result["A"] = Mathf.Rad2Deg(cba.z);
            result["B"] = Mathf.Rad2Deg(cba.y);
            result["C"] = Mathf.Rad2Deg(cba.x);
            return result;
        }

        public void SetCurrentCommand(Controller.MotionCommand command)
        {
            currentCommand = command;
        }

        public void wait(float seconds) {
            System.Threading.Thread.Sleep((int)(seconds * 1000));
        }

        private static float LuaNumber2Float(object obj)
        {
            return Convert.ToSingle(obj);
        }

        private static int LuaNumber2Int(object obj)
        {
            return Convert.ToInt32(obj);
        }

        private Controller.Position LuaTable2Position(LuaTable table)
        {
            Controller.CartesianTarget position = new Controller.CartesianTarget();
            position.t[Controller.CartesianAxis.X] = LuaNumber2Float(table["X"]);
            position.t[Controller.CartesianAxis.Y] = LuaNumber2Float(table["Y"]);
            position.t[Controller.CartesianAxis.Z] = LuaNumber2Float(table["Z"]);
            position.t[Controller.CartesianAxis.A] = LuaNumber2Float(table["A"]);
            position.t[Controller.CartesianAxis.B] = LuaNumber2Float(table["B"]);
            position.t[Controller.CartesianAxis.C] = LuaNumber2Float(table["C"]);
            return position.AsPosition(null);
        }
    }
}
