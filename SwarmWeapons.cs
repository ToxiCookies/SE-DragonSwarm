// DragonSwarm Weapons controller
// Listens for FIRE/CEASE commands via IGC and toggles weapons

readonly System.Collections.Generic.List<IMyUserControllableGun> _guns = new System.Collections.Generic.List<IMyUserControllableGun>();
IMyBroadcastListener _listener;
bool _fire = false;
bool _lastFire = false;

public Program()
{
    GridTerminalSystem.GetBlocksOfType(_guns);
    _listener = IGC.RegisterBroadcastListener(SwarmIGC.CHANNEL);
    _listener.SetMessageCallback("IGC");
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
}

public void Main(string argument, UpdateType updateSource)
{
    if ((updateSource & (UpdateType.IGC | UpdateType.Update10)) != 0)
        ProcessMessages();
    if (!string.IsNullOrEmpty(argument))
        HandleCommand(argument);
    if (_fire != _lastFire)
        ToggleGuns(_fire);
}

void ProcessMessages()
{
    while (_listener.HasPendingMessage)
    {
        MyIGCMessage raw = _listener.AcceptMessage();
        SwarmIGC.Message msg = new SwarmIGC.Message();
        if (!SwarmIGC.TryParse(raw, ref msg))
            continue;
        HandleCommand(msg.Command);
    }
}

void HandleCommand(string cmd)
{
    string c = cmd.ToUpperInvariant();
    if (c == "FIRE")
        _fire = true;
    else if (c == "CEASE")
        _fire = false;
}

void ToggleGuns(bool on)
{
    foreach (var gun in _guns)
        gun.ApplyAction(on ? "Shoot_On" : "Shoot_Off");
    _lastFire = on;
}

