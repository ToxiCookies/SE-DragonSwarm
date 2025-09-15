// DragonSwarm Host controller
// Handles satellite registration and command broadcast

readonly System.Collections.Generic.Dictionary<long, int> _satellites = new System.Collections.Generic.Dictionary<long, int>();
IMyBroadcastListener _listener;
int _nextIndex = 0;
SwarmCore.PID _pid = new SwarmCore.PID(0.6, 1.4); // example PID for slot radius control

public Program()
{
    _listener = IGC.RegisterBroadcastListener(SwarmIGC.CHANNEL);
    _listener.SetMessageCallback("IGC");
    Runtime.UpdateFrequency = UpdateFrequency.Update10; // 6 ticks per second
}

public void Main(string argument, UpdateType updateSource)
{
    if ((updateSource & (UpdateType.IGC | UpdateType.Update10)) != 0)
    {
        ProcessMessages();
        BroadcastStatus();
    }
    if (!string.IsNullOrEmpty(argument))
        HandleCommand(argument);
}

void ProcessMessages()
{
    while (_listener.HasPendingMessage)
    {
        MyIGCMessage raw = _listener.AcceptMessage();
        SwarmIGC.Message msg = new SwarmIGC.Message();
        if (!SwarmIGC.TryParse(raw, ref msg))
            continue;
        if (msg.Command == "HELLO")
            AssignSatellite(msg.Sender);
    }
}

void AssignSatellite(long entityId)
{
    if (_satellites.ContainsKey(entityId))
        return;
    int slot = _nextIndex++;
    _satellites[entityId] = slot;
    SwarmIGC.Send(IGC, new SwarmIGC.Message { Sender = Me.CubeGrid.EntityId, Command = "ASSIGN", Data = slot.ToString() });
}

void BroadcastStatus()
{
    VRageMath.Vector3D pos = Me.CubeGrid.WorldMatrix.Translation;
    string data = pos.X.ToString("0.##") + "," + pos.Y.ToString("0.##") + "," + pos.Z.ToString("0.##");
    SwarmIGC.Send(IGC, new SwarmIGC.Message { Sender = Me.CubeGrid.EntityId, Command = "HOST", Data = data });
}

void HandleCommand(string arg)
{
    // Broadcast command to all satellites
    SwarmIGC.Send(IGC, new SwarmIGC.Message { Sender = Me.CubeGrid.EntityId, Command = arg.ToUpperInvariant() });
}
