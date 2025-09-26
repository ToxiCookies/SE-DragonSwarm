// DragonSwarm Host controller
// Handles satellite registration, command broadcast and data consolidation
//
// Host Commands:
//   HOUSEPARTY  - satellites in the closest third converge defensively (cancel with CURFEW)
//   CURFEW      - restore normal formation after HOUSEPARTY
//   DOCK        - attempt to dock all swarm members within range
//   UNDOCK      - release docked members back into formation
//   FIRE <#><*> - fire missile group # with mode A/B/C (0 fires all)
//                 A: host target  B: nearest enemy  C: largest grid
//   CEASEFIRE   - disable offensive and defensive actions
//   ARM         - enable offensive and defensive actions
//   INFO <id>   - show detailed data for a member on screen 5

readonly System.Collections.Generic.Dictionary<long, int> _satellites = new System.Collections.Generic.Dictionary<long, int>();
readonly System.Collections.Generic.Dictionary<long, VRageMath.Vector3D> _contacts = new System.Collections.Generic.Dictionary<long, VRageMath.Vector3D>();
readonly System.Collections.Generic.Dictionary<long, SatInfo> _satInfo = new System.Collections.Generic.Dictionary<long, SatInfo>();
readonly System.Collections.Generic.Dictionary<long, MissileInfo> _missiles = new System.Collections.Generic.Dictionary<long, MissileInfo>();
IMyBroadcastListener _listener;
IMyTextSurface[] _screens = new IMyTextSurface[5];
long _detailId = 0;
int _nextIndex = 0;
SwarmCore.PID _pid = new SwarmCore.PID(0.6, 1.4); // example PID for slot radius control

struct SatInfo { public double Shield; public double Energy; public double Health; }
struct MissileInfo { public int Group; public char Mode; }

public Program()
{
    _listener = IGC.RegisterBroadcastListener(SwarmIGC.CHANNEL);
    _listener.SetMessageCallback("IGC");
    for (int i = 0; i < 5 && i < Me.SurfaceCount; i++)
    {
        _screens[i] = Me.GetSurface(i);
        _screens[i].ContentType = VRage.Game.GUI.TextPanel.ContentType.TEXT_AND_IMAGE;
    }
    Runtime.UpdateFrequency = UpdateFrequency.Update10; // 6 ticks per second
}

public void Main(string argument, UpdateType updateSource)
{
    if ((updateSource & (UpdateType.IGC | UpdateType.Update10)) != 0)
    {
        ProcessMessages();
        BroadcastStatus();
        DrawDisplays();
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
        else if (msg.Command == "CONTACT")
            ParseContact(msg);
        else if (msg.Command == "STAT")
            ParseStat(msg);
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

void ParseContact(SwarmIGC.Message msg)
{
    string[] parts = msg.Data.Split(',');
    if (parts.Length == 3)
    {
        double x, y, z;
        if (double.TryParse(parts[0], out x) && double.TryParse(parts[1], out y) && double.TryParse(parts[2], out z))
            _contacts[msg.Sender] = new VRageMath.Vector3D(x, y, z);
    }
}

void ParseStat(SwarmIGC.Message msg)
{
    string[] parts = msg.Data.Split('|');
    if (parts.Length == 0) return;
    if (parts[0] == "M" && parts.Length >= 3)
    {
        int g; char m = 'A';
        int.TryParse(parts[1], out g);
        if (parts[2].Length > 0) m = parts[2][0];
        MissileInfo info = new MissileInfo();
        info.Group = g; info.Mode = m;
        _missiles[msg.Sender] = info;
    }
    else if (parts[0] == "S" && parts.Length >= 4)
    {
        double sh = 0, en = 0, hp = 0;
        double.TryParse(parts[1], out sh);
        double.TryParse(parts[2], out en);
        double.TryParse(parts[3], out hp);
        SatInfo info = new SatInfo();
        info.Shield = sh; info.Energy = en; info.Health = hp;
        _satInfo[msg.Sender] = info;
    }
}

void DrawDisplays()
{
    DrawSlots();
    DrawRadar();
    DrawMissiles();
    DrawSatellites();
    DrawDetail();
}

void DrawSlots()
{
    if (_screens[0] == null) return;
    System.Text.StringBuilder sb = new System.Text.StringBuilder();
    sb.Append("Shell Slots\n");
    for (int i = 0; i < 24; i++)
    {
        bool used = false;
        foreach (var kv in _satellites)
            if (kv.Value == i) { used = true; break; }
        sb.Append(i.ToString("00")).Append(": ").Append(used ? "X" : "-").Append('\n');
    }
    _screens[0].WriteText(sb.ToString());
}

void DrawRadar()
{
    if (_screens[1] == null) return;
    System.Text.StringBuilder sb = new System.Text.StringBuilder();
    sb.Append("Radar\n");
    VRageMath.Vector3D hostPos = Me.CubeGrid.WorldMatrix.Translation;
    foreach (var kv in _contacts)
    {
        VRageMath.Vector3D rel = kv.Value - hostPos;
        double dist = rel.Length();
        double az = System.Math.Atan2(rel.X, rel.Z) * 57.2957795;
        double el = dist > 0 ? System.Math.Asin(rel.Y / dist) * 57.2957795 : 0.0;
        sb.Append(kv.Key).Append(": ")
          .Append(dist.ToString("0")).Append("m ")
          .Append(az.ToString("0")).Append("°/")
          .Append(el.ToString("0")).Append("°\n");
    }
    _screens[1].WriteText(sb.ToString());
}

void DrawMissiles()
{
    if (_screens[2] == null) return;
    System.Text.StringBuilder sb = new System.Text.StringBuilder();
    sb.Append("Missiles\n");
    foreach (var kv in _missiles)
    {
        sb.Append(kv.Key).Append(": G").Append(kv.Value.Group)
          .Append(" M").Append(kv.Value.Mode).Append('\n');
    }
    _screens[2].WriteText(sb.ToString());
}

void DrawSatellites()
{
    if (_screens[3] == null) return;
    System.Text.StringBuilder sb = new System.Text.StringBuilder();
    sb.Append("Satellites\n");
    foreach (var kv in _satInfo)
    {
        sb.Append(kv.Key).Append(": Sh ").Append(kv.Value.Shield.ToString("0"))
          .Append(" En ").Append(kv.Value.Energy.ToString("0"))
          .Append(" Hp ").Append(kv.Value.Health.ToString("0")).Append('\n');
    }
    _screens[3].WriteText(sb.ToString());
}

void DrawDetail()
{
    if (_screens[4] == null) return;
    System.Text.StringBuilder sb = new System.Text.StringBuilder();
    if (_detailId != 0)
    {
        sb.Append("Detail ").Append(_detailId).Append('\n');
        MissileInfo mi; SatInfo si;
        if (_missiles.TryGetValue(_detailId, out mi))
        {
            sb.Append("Missile G").Append(mi.Group).Append(" M").Append(mi.Mode);
        }
        else if (_satInfo.TryGetValue(_detailId, out si))
        {
            sb.Append("Shield ").Append(si.Shield.ToString("0"))
              .Append(" Energy ").Append(si.Energy.ToString("0"))
              .Append(" Health ").Append(si.Health.ToString("0"));
        }
        else
        {
            sb.Append("No data");
        }
    }
    else sb.Append("No selection");
    _screens[4].WriteText(sb.ToString());
}

void HandleCommand(string arg)
{
    string cmd = arg.Trim();
    if (string.IsNullOrEmpty(cmd)) return;
    string upper = cmd.ToUpperInvariant();
    if (upper.StartsWith("FIRE"))
    {
        string[] p = cmd.Split(' ');
        string data = p.Length > 1 ? p[1].ToUpperInvariant() : "0A";
        Broadcast("FIRE", data);
    }
    else if (upper == "HOUSEPARTY")
        Broadcast("HOUSEPARTY");
    else if (upper == "CURFEW")
        Broadcast("CURFEW");
    else if (upper == "DOCK")
        Broadcast("DOCK");
    else if (upper == "UNDOCK")
        Broadcast("UNDOCK");
    else if (upper == "CEASEFIRE")
        Broadcast("CEASEFIRE");
    else if (upper == "ARM")
        Broadcast("ARM");
    else if (upper.StartsWith("INFO"))
    {
        string[] p = cmd.Split(' ');
        if (p.Length > 1)
        {
            long id;
            if (long.TryParse(p[1], out id))
                _detailId = id;
        }
    }
    else
        Broadcast(upper);
}

void Broadcast(string command, string data = null)
{
    SwarmIGC.Send(IGC, new SwarmIGC.Message { Sender = Me.CubeGrid.EntityId, Command = command, Data = data });
}

