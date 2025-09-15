// DragonSwarm Satellite controller
// Listens for host messages and maintains formation slot

IMyBroadcastListener _listener;
IMyShipController _controller;
readonly System.Collections.Generic.List<IMyThrust> _thrusters = new System.Collections.Generic.List<IMyThrust>();
int _slot = -1;
long _hostId = 0;
VRageMath.Vector3D _hostPos;

public Program()
{
    _listener = IGC.RegisterBroadcastListener(SwarmIGC.CHANNEL);
    _listener.SetMessageCallback("IGC");
    GridTerminalSystem.GetBlocksOfType(_thrusters);
    var controllers = new System.Collections.Generic.List<IMyShipController>();
    GridTerminalSystem.GetBlocksOfType(controllers);
    if (controllers.Count > 0) _controller = controllers[0];
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    Hello();
}

void Hello()
{
    SwarmIGC.Send(IGC, new SwarmIGC.Message { Sender = Me.CubeGrid.EntityId, Command = "HELLO" });
}

public void Main(string argument, UpdateType updateSource)
{
    if ((updateSource & UpdateType.IGC) != 0)
        ProcessMessages();
    if ((updateSource & UpdateType.Update10) != 0)
        MaintainFormation();
}

void ProcessMessages()
{
    while (_listener.HasPendingMessage)
    {
        MyIGCMessage raw = _listener.AcceptMessage();
        SwarmIGC.Message msg = new SwarmIGC.Message();
        if (!SwarmIGC.TryParse(raw, ref msg))
            continue;
        if (msg.Command == "ASSIGN" && msg.Sender == _hostId)
        {
            int.TryParse(msg.Data, out _slot);
        }
        else if (msg.Command == "HOST")
        {
            _hostId = msg.Sender;
            string[] parts = msg.Data.Split(',');
            if (parts.Length == 3)
            {
                double x, y, z;
                if (double.TryParse(parts[0], out x) && double.TryParse(parts[1], out y) && double.TryParse(parts[2], out z))
                    _hostPos = new VRageMath.Vector3D(x, y, z);
            }
        }
        else if (msg.Command == "HELLO")
        {
            // ignore other satellites
        }
    }
}

void MaintainFormation()
{
    if (_slot < 0 || _controller == null) return;
    VRageMath.Vector3D slot = SwarmCore.SlotPoint(_slot, 24, 100.0); // simple fixed formation
    VRageMath.Vector3D target = _hostPos + slot;
    VRageMath.Vector3D pos = Me.CubeGrid.WorldMatrix.Translation;
    VRageMath.Vector3D vel = _controller.GetShipVelocities().LinearVelocity;
    VRageMath.Vector3D toTarget = target - pos;
    VRageMath.Vector3D desiredVel = toTarget * 0.5; // proportional gain
    VRageMath.Vector3D dv = desiredVel - vel;
    double mass = _controller.CalculateShipMass().PhysicalMass;
    foreach (IMyThrust t in _thrusters)
    {
        VRageMath.Vector3D dir = t.WorldMatrix.Backward; // thrust direction
        double thrust = VRageMath.Vector3D.Dot(dir, dv) * mass;
        t.ThrustOverride = thrust > 0 ? (float)thrust : 0f;
    }
}
