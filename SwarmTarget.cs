// DragonSwarm Targeting module
// Shares enemy contact data and provides intercept predictions

IMyBroadcastListener _listener;
IMyCameraBlock _lidar;

long _targetId;
VRageMath.Vector3D _targetPos;
VRageMath.Vector3D _targetVel;
int _lostTicks;
const double LIDAR_RANGE = 4000.0; // meters
const int CLEAR_TICKS = 5;          // ticks before forgetting target

public Program()
{
    _listener = IGC.RegisterBroadcastListener(SwarmIGC.CHANNEL);
    _listener.SetMessageCallback("IGC");

    var cams = new System.Collections.Generic.List<IMyCameraBlock>();
    GridTerminalSystem.GetBlocksOfType(cams);
    if (cams.Count > 0)
    {
        _lidar = cams[0];
        _lidar.EnableRaycast = true;
    }

    Runtime.UpdateFrequency = UpdateFrequency.Update10;
}

public void Main(string argument, UpdateType updateSource)
{
    if ((updateSource & UpdateType.IGC) != 0)
        ProcessMessages();
    if ((updateSource & UpdateType.Update10) != 0)
        UpdateTarget();
}

void ProcessMessages()
{
    while (_listener.HasPendingMessage)
    {
        MyIGCMessage raw = _listener.AcceptMessage();
        SwarmIGC.Message msg = new SwarmIGC.Message();
        if (!SwarmIGC.TryParse(raw, ref msg))
            continue;
        if (msg.Command == "TARGET")
        {
            string[] parts = msg.Data.Split(';');
            if (parts.Length == 3)
            {
                long id;
                if (long.TryParse(parts[0], out id))
                {
                    _targetId = id;
                    _targetPos = ParseVector(parts[1]);
                    _targetVel = ParseVector(parts[2]);
                    _lostTicks = 0;
                }
            }
        }
        else if (msg.Command == "CLEAR")
        {
            if (_targetId != 0 && (msg.Data == _targetId.ToString() || msg.Data.Length == 0))
                ClearTarget();
        }
    }
}

void UpdateTarget()
{
    bool detected = false;
    if (_lidar != null && _lidar.CanScan(LIDAR_RANGE))
    {
        MyDetectedEntityInfo info = _lidar.Raycast(LIDAR_RANGE);
        if (!info.IsEmpty() && info.Relationship == MyRelationsBetweenPlayerAndBlock.Enemies)
        {
            _targetId = info.EntityId;
            _targetPos = info.Position;
            _targetVel = info.Velocity;
            _lostTicks = 0;
            detected = true;
        }
    }

    if (detected)
    {
        BroadcastTarget();
    }
    else if (_targetId != 0)
    {
        _lostTicks++;
        if (_lostTicks > CLEAR_TICKS)
            ClearTarget();
    }
}

void BroadcastTarget()
{
    string data = _targetId.ToString() + ";" + FormatVector(_targetPos) + ";" + FormatVector(_targetVel);
    SwarmIGC.Send(IGC, new SwarmIGC.Message { Sender = Me.CubeGrid.EntityId, Command = "TARGET", Data = data });
}

void ClearTarget()
{
    SwarmIGC.Send(IGC, new SwarmIGC.Message { Sender = Me.CubeGrid.EntityId, Command = "CLEAR", Data = _targetId.ToString() });
    _targetId = 0;
    _targetPos = new VRageMath.Vector3D();
    _targetVel = new VRageMath.Vector3D();
    _lostTicks = 0;
}

// Obtain predicted intercept point for a projectile
VRageMath.Vector3D GetIntercept(VRageMath.Vector3D shooterPos, VRageMath.Vector3D shooterVel, double projectileSpeed)
{
    if (_targetId == 0)
        return shooterPos;
    return SwarmCore.PredictIntercept(shooterPos, shooterVel, _targetPos, _targetVel, projectileSpeed);
}

static VRageMath.Vector3D ParseVector(string s)
{
    string[] a = s.Split(',');
    double x = 0, y = 0, z = 0;
    if (a.Length > 0) double.TryParse(a[0], out x);
    if (a.Length > 1) double.TryParse(a[1], out y);
    if (a.Length > 2) double.TryParse(a[2], out z);
    return new VRageMath.Vector3D(x, y, z);
}

static string FormatVector(VRageMath.Vector3D v)
{
    return v.X.ToString("0.##") + "," + v.Y.ToString("0.##") + "," + v.Z.ToString("0.##");
}
