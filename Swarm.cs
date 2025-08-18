// DragonSwarm (PB-safe) — Host/Satellite swarm with concentric Fibonacci spheres
// - Works fully inside the in-game Programmable Block (no usings)
// - Stable cadence, low allocations, conservative control
// - Fixes: correct host basis (Right = Forward x Up), robust thruster axis grouping,
//          grid-frame thrust mapping, arrival hold, accel clamp, relative speed cap,
//          optional orientation alignment to Host when holding

#region Fields & Config

const double PHI = 1.6180339887498948482;                  // golden ratio
const double GOLDEN_ANGLE = 2.0 * Math.PI * (1.0 - 1.0/PHI);// ~2.399963

readonly System.Globalization.CultureInfo CI = System.Globalization.CultureInfo.InvariantCulture;

MyIni _ini = new MyIni();
System.Text.StringBuilder _sb = new System.Text.StringBuilder(256);
System.Text.StringBuilder _echo = new System.Text.StringBuilder(512);

// configuration
Role _role = Role.Host;
string _formationGroup = "SwarmAlpha";
int _shellCount = 3;
int _pointsPerShell = 24;
double _shellSpacing = 250.0;
double _baseRadius = 1000.0;

double _maxSpeed = 50.0;   // m/s cap (relative to host)
double _kp = 0.6;          // position P gain (to velocity)
double _ki = 0.0;          // reserved
double _kd = 1.4;          // velocity gain (to acceleration)
double _gyroKp = 3.0;      // pointing gain
double _gyroKd = 1.0;      // damping gain
double _arrival = 10.0;     // meters: zero-thrust hold inside this

double _minSep = 18.0;     // meters: separation sensing
double _sepGain = 1.1;
double _hostBuffer = 22.0; // buffer distance beyond host radius
double _hostRadius = 0.0;  // meters: host grid radius for push-out zone
bool _useSensors = true;   // sensors must include "[Swarm Sensor]" in name
bool _sensorFallback = false; // true when sensors missing

// alignment options (new)
bool _alignToHost = true;           // align orientation to host when holding
double _alignKp   = 3.0;            // angular P gain for alignment
double _alignDeadzoneRad = 0.2; // ~2 deg
FaceSide _parkingFace = FaceSide.MatchHost; // which face points away when parked

string _controllerName = string.Empty;
string _refForward = string.Empty; // reserved
string _refUp = string.Empty;      // reserved

int _index = -1; // satellite index (global)
bool _debug = false;
bool _kamikaze = false; // dive into target and detonate
bool _weaponsEnabled = true; // allow weapon firing
string _weaponSubsystem = "Any"; // WeaponCore subsystem targeting
double _shieldDisableRange = 50000.0; // meters: shields off beyond this enemy distance

// cached blocks
IMyShipController _controller;
readonly System.Collections.Generic.List<IMyGyro> _gyros = new System.Collections.Generic.List<IMyGyro>(16);
readonly System.Collections.Generic.List<IMyThrust> _thrusters = new System.Collections.Generic.List<IMyThrust>(64);
readonly System.Collections.Generic.List<IMySensorBlock> _sensors = new System.Collections.Generic.List<IMySensorBlock>(8);
readonly System.Collections.Generic.List<IMyWarhead> _warheads = new System.Collections.Generic.List<IMyWarhead>(8);
readonly System.Collections.Generic.List<IMyJumpDrive> _jumpDrives = new System.Collections.Generic.List<IMyJumpDrive>(8);
readonly System.Collections.Generic.List<IMyTerminalBlock> _weapons = new System.Collections.Generic.List<IMyTerminalBlock>(32);
readonly System.Collections.Generic.List<IMyTerminalBlock> _lowPowerWeapons = new System.Collections.Generic.List<IMyTerminalBlock>(32);
readonly System.Collections.Generic.List<IMyFunctionalBlock> _shields = new System.Collections.Generic.List<IMyFunctionalBlock>(4);
IMyTerminalBlock _trackingTurret;
Vector3D _jumpTarget;
double _jumpDelay = -1.0; // seconds until executing a received jump

IMyRadioAntenna _antenna;
string _antennaHud = string.Empty;
bool _kamikazeOnEmpty = false;
bool _resupplySignaled = false;
readonly System.Collections.Generic.List<MyInventoryItem> _ammoTmp = new System.Collections.Generic.List<MyInventoryItem>(1);


readonly System.Collections.Generic.HashSet<long> _friendGrids = new System.Collections.Generic.HashSet<long>();

ThrusterAxis _axisX = new ThrusterAxis(); // +Right / -Right
ThrusterAxis _axisY = new ThrusterAxis(); // +Up / -Up
ThrusterAxis _axisZ = new ThrusterAxis(); // +Forward / -Forward

IMyBroadcastListener _hostListener;   // for satellites
IMyBroadcastListener _statusListener; // for host
IMyBroadcastListener _cmdListener;    // command listener for satellites
IMyBroadcastListener _friendListener; // friend grid IDs

string _hostTag;
string _statusTag;
string _cmdTag;
string _friendTag;

// runtime
int _tick = 0;
int _totalPoints = 0;
int _shellOfIndex = 0;
int _pointInShell = 0;

// host telemetry data
long _hostId;
Vector3D _hostPos;
Vector3D _hostVel;
MatrixD _hostMatrix = MatrixD.Identity;
int _hostTick;

double _timeSinceTelemetry = 1e6; // seconds
double _dt = 1.0/6.0; // last timestep in seconds

// mass cache
double _shipMass = 0;

enum Role { Host, Satellite }

enum FaceSide { MatchHost, Forward, Backward, Up, Down, Left, Right }

class ThrusterAxis {
    public readonly System.Collections.Generic.List<IMyThrust> Pos = new System.Collections.Generic.List<IMyThrust>(16);
    public readonly System.Collections.Generic.List<IMyThrust> Neg = new System.Collections.Generic.List<IMyThrust>(16);
    public double MaxPos;
    public double MaxNeg;
    public void Reset() { Pos.Clear(); Neg.Clear(); MaxPos = 0; MaxNeg = 0; }
}

#endregion

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10; // ~6 Hz main loop
    LoadState();
    ParseIni();
    DiscoverBlocks();
    SetupIGC();
}

public void Save()
{
    SaveState();
    ClearOverrides();
}

#region State

void LoadState()
{
    if (string.IsNullOrWhiteSpace(Storage)) return;
    var ini = new MyIni();
    if (ini.TryParse(Storage))
    {
        _index = ini.Get("State","Index").ToInt32(_index);
        _debug = ini.Get("State","Debug").ToBoolean(_debug);
    }
}

void SaveState()
{
    var ini = new MyIni();
    ini.Set("State","Index",_index);
    ini.Set("State","Debug",_debug);
    Storage = ini.ToString();
}

#endregion

#region Config

void ParseIni()
{
    if (!_ini.TryParse(Me.CustomData)) _ini.Clear();

    string roleStr = _ini.Get("Role","Mode").ToString(_role == Role.Satellite ? "Satellite" : "Host");
    _role = roleStr == "Satellite" ? Role.Satellite : Role.Host;

    _formationGroup = _ini.Get("IDs","FormationGroup").ToString(_formationGroup);
    string idxStr = _ini.Get("IDs","Index").ToString("auto");

    _shellCount     = Math.Max(1, _ini.Get("Formation","ShellCount").ToInt32(_shellCount));
    _pointsPerShell = Math.Max(1, _ini.Get("Formation","PointsPerShell").ToInt32(_pointsPerShell));
    _shellSpacing   = Math.Max(1.0, _ini.Get("Formation","ShellSpacing").ToDouble(_shellSpacing));
    _baseRadius     = Math.Max(1.0, _ini.Get("Formation","BaseRadius").ToDouble(_baseRadius));

    _maxSpeed = Math.Max(1.0, _ini.Get("Control","MaxSpeed").ToDouble(_maxSpeed));
    _kp       = _ini.Get("Control","PositionKp").ToDouble(_kp);
    _ki       = _ini.Get("Control","PositionKi").ToDouble(_ki);
    _kd       = _ini.Get("Control","PositionKd").ToDouble(_kd);
    _gyroKp   = _ini.Get("Control","GyroKp").ToDouble(_gyroKp);
    _gyroKd   = _ini.Get("Control","GyroKd").ToDouble(_gyroKd);
    _arrival  = Math.Max(0.1, _ini.Get("Control","ArrivalTolerance").ToDouble(_arrival));

    _minSep     = Math.Max(0.1, _ini.Get("Avoidance","MinSeparation").ToDouble(_minSep));
    _sepGain    = _ini.Get("Avoidance","SeparationGain").ToDouble(_sepGain);
    _hostBuffer = _ini.Get("Avoidance","HostBuffer").ToDouble(_hostBuffer);
    _hostRadius = _ini.Get("Avoidance","HostRadius").ToDouble(_hostRadius);
    _useSensors = _ini.Get("Avoidance","UseSensors").ToBoolean(_useSensors);

    _controllerName = _ini.Get("Blocks","MainControllerName").ToString(_controllerName);
    _refForward     = _ini.Get("Blocks","ReferenceForward").ToString(_refForward);
    _refUp          = _ini.Get("Blocks","ReferenceUp").ToString(_refUp);

    _weaponSubsystem = _ini.Get("Weapons","TargetSubsystem").ToString(_weaponSubsystem);

    // alignment options
    _alignToHost = _ini.Get("Control","AlignToHost").ToBoolean(_alignToHost);
    _alignKp     = _ini.Get("Control","AlignKp").ToDouble(_alignKp);
    double alignDeadzoneDeg = _ini.Get("Control","AlignDeadzoneDeg").ToDouble(_alignDeadzoneRad * 180.0 / Math.PI);
    _alignDeadzoneRad = alignDeadzoneDeg * Math.PI / 180.0;

    string faceStr = _ini.Get("Control","ParkingFace").ToString("MatchHost");
    switch(faceStr)
    {
        case "Forward": _parkingFace = FaceSide.Forward; break;
        case "Backward": _parkingFace = FaceSide.Backward; break;
        case "Up": _parkingFace = FaceSide.Up; break;
        case "Down": _parkingFace = FaceSide.Down; break;
        case "Left": _parkingFace = FaceSide.Left; break;
        case "Right": _parkingFace = FaceSide.Right; break;
        default: _parkingFace = FaceSide.MatchHost; break;
    }

    // total points
    _totalPoints = 0;
    for (int s=0; s<_shellCount; s++)
        _totalPoints += _pointsPerShell * (s+1);
    if (_totalPoints <= 0) _totalPoints = 1;

    // choose index
    if (string.Equals(idxStr, "auto", System.StringComparison.OrdinalIgnoreCase))
    {
        long gid = Me.CubeGrid.EntityId;
        _index = (int)Math.Abs(gid % _totalPoints);
    }
    else
    {
        int idx;
        if (int.TryParse(idxStr, out idx))
            _index = Math.Max(0, Math.Min(_totalPoints-1, idx));
        else
            _index = 0;
    }

    MapIndexToShellPoint(_index);
}

void MapIndexToShellPoint(int idx)
{
    int rem = idx;
    _shellOfIndex = 0;
    _pointInShell = 0;
    for (int s=0; s<_shellCount; s++)
    {
        int ns = _pointsPerShell*(s+1);
        if (rem < ns) { _shellOfIndex = s; _pointInShell = rem; return; }
        rem -= ns;
    }
    _shellOfIndex = _shellCount - 1;
    _pointInShell = 0;
}

#endregion

#region Block Discovery

void DiscoverBlocks()
{
    _controller = null;
    _gyros.Clear();
    _thrusters.Clear();
    _sensors.Clear();
    _warheads.Clear();
    _weapons.Clear();
    _lowPowerWeapons.Clear();
    _shields.Clear();
    _trackingTurret = null;
    _jumpDrives.Clear();
    _axisX.Reset(); _axisY.Reset(); _axisZ.Reset();

    _friendGrids.Clear();
    _friendGrids.Add(Me.CubeGrid.EntityId);

    var tmp = new System.Collections.Generic.List<IMyTerminalBlock>(128);
    GridTerminalSystem.GetBlocks(tmp);
    for (int i=0; i<tmp.Count; i++)
    {
        var b = tmp[i];
        if (b.CubeGrid != Me.CubeGrid) continue;

        var shield = b as IMyFunctionalBlock;
        if (shield != null)
        {
            string disp = shield.DefinitionDisplayNameText;
            if (!string.IsNullOrEmpty(disp) &&
                disp.IndexOf("Shield", System.StringComparison.OrdinalIgnoreCase) >= 0)
            {
                _shields.Add(shield);
                continue;
            }
        }

        var sc = b as IMyShipController;
        if (sc != null)
        {
            if (_controller == null)
            {
                if (string.IsNullOrEmpty(_controllerName) || sc.CustomName == _controllerName)
                    _controller = sc;
            }
            continue;
        }

        var g = b as IMyGyro;
        if (g != null) { _gyros.Add(g); continue; }

        var t = b as IMyThrust;
        if (t != null) { _thrusters.Add(t); continue; }

        var jd = b as IMyJumpDrive;
        if (jd != null) { _jumpDrives.Add(jd); continue; }

        var w = b as IMyWarhead;
        if (w != null) { _warheads.Add(w); continue; }

        var ant = b as IMyRadioAntenna;
        if (ant != null)
        {
            if (_antenna == null) { _antenna = ant; _antennaHud = ant.HudText; }
            continue;
        }

        var gun = b as IMyUserControllableGun;
        if (gun != null)
        {
            _weapons.Add(gun);
            CategorizeWeapon(gun);
            if (_trackingTurret == null)
            {
                var lt = gun as IMyLargeTurretBase;
                if (lt != null) _trackingTurret = lt;
            }
            continue;
        }

        if (b.GetActionWithName("Shoot_On") != null && b.GetActionWithName("Shoot_Off") != null)
        {
            _weapons.Add(b);
            CategorizeWeapon(b);
            if (_trackingTurret == null && b.GetProperty("WC_TargetLock") != null)
                _trackingTurret = b;
            continue;
        }

        var s = b as IMySensorBlock;
        if (s != null && s.CustomName.IndexOf("[Swarm Sensor]", System.StringComparison.OrdinalIgnoreCase) >= 0)
            _sensors.Add(s);
    }

    // fallback controller if named one not found
    if (_controller == null)
    {
        for (int i=0; i<tmp.Count; i++)
        {
            var sc = tmp[i] as IMyShipController;
            if (sc != null) { _controller = sc; break; }
        }
    }

    // categorize thrusters by actual acceleration axis in GRID local frame (robust)
    MatrixD gridWM = ( _controller != null ? _controller.CubeGrid.WorldMatrix : Me.CubeGrid.WorldMatrix );
    MatrixD invGrid = MatrixD.Transpose(gridWM);

    for (int i = 0; i < _thrusters.Count; i++)
    {
        var t = _thrusters[i];

        // Actual acceleration is opposite of the block's forward
        Vector3D accWorld = -t.WorldMatrix.Forward;
        Vector3D accLocal = Vector3D.TransformNormal(accWorld, invGrid);

        // Choose dominant component to classify
        double ax = Math.Abs(accLocal.X), ay = Math.Abs(accLocal.Y), az = Math.Abs(accLocal.Z);

        if (ax >= ay && ax >= az)
        {
            if (accLocal.X >= 0) { _axisX.Pos.Add(t); _axisX.MaxPos += t.MaxEffectiveThrust; }
            else                 { _axisX.Neg.Add(t); _axisX.MaxNeg += t.MaxEffectiveThrust; }
        }
        else if (ay >= ax && ay >= az)
        {
            if (accLocal.Y >= 0) { _axisY.Pos.Add(t); _axisY.MaxPos += t.MaxEffectiveThrust; }
            else                 { _axisY.Neg.Add(t); _axisY.MaxNeg += t.MaxEffectiveThrust; }
        }
        else
        {
            if (accLocal.Z >= 0) { _axisZ.Pos.Add(t); _axisZ.MaxPos += t.MaxEffectiveThrust; }
            else                 { _axisZ.Neg.Add(t); _axisZ.MaxNeg += t.MaxEffectiveThrust; }
        }
    }

    ApplyWeaponTargeting();

    if (_useSensors)
    {
        _sensorFallback = (_sensors.Count == 0);
        if (_sensorFallback)
        {
            _useSensors = false;
            _hostBuffer = Math.Max(_hostBuffer, _minSep * 2.0);
        }
    }
    else
    {
        _sensorFallback = false;
    }
}

void ApplyWeaponTargeting()
{
    for (int i=0; i<_weapons.Count; i++)
    {
        var w = _weapons[i];
        if (w.GetProperty("WC_TargetSubSystem") != null)
            w.SetValue<string>("WC_TargetSubSystem", _weaponSubsystem);
    }
}

#endregion

#region IGC

void SetupIGC()
{
    _hostTag   = _formationGroup + ".HOST.TELEMETRY";
    _statusTag = _formationGroup + ".SAT.STATUS";
    _cmdTag    = _formationGroup + ".CMD";
    _friendTag = _formationGroup + ".FRIEND";

    _hostListener   = null;
    _statusListener = null;
    _cmdListener    = null;
    _friendListener = null;

    if (_role == Role.Satellite)
    {
        _hostListener = IGC.RegisterBroadcastListener(_hostTag);
        _hostListener.SetMessageCallback(_hostTag); // optional
        _cmdListener = IGC.RegisterBroadcastListener(_cmdTag);
        _cmdListener.SetMessageCallback(_cmdTag);
    }
    else
    {
        _statusListener = IGC.RegisterBroadcastListener(_statusTag);
        _statusListener.SetMessageCallback(_statusTag);
    }

    _friendListener = IGC.RegisterBroadcastListener(_friendTag);
    _friendListener.SetMessageCallback(_friendTag);
}

#endregion

public void Main(string argument, UpdateType updateSource)
{
    if ((updateSource & UpdateType.Terminal) != 0 || (updateSource & UpdateType.Trigger) != 0)
    {
        if (argument == "reload")
        {
            ParseIni();
            DiscoverBlocks();
            SetupIGC();
        }
        else if (argument.StartsWith("debug", System.StringComparison.OrdinalIgnoreCase))
        {
            _debug = argument.EndsWith("on", System.StringComparison.OrdinalIgnoreCase);
        }
        else if (argument.StartsWith("index", System.StringComparison.OrdinalIgnoreCase))
        {
            var parts = argument.Split(new[]{' '}, 2);
            if (parts.Length > 1)
            {
                int idx;
                if (int.TryParse(parts[1], out idx))
                {
                    _index = Math.Max(0, Math.Min(_totalPoints-1, idx));
                    MapIndexToShellPoint(_index);
                }
            }
        }
        else if (argument == "boom" && _role == Role.Host)
        {
            IGC.SendBroadcastMessage(_cmdTag, "CMD|DETONATE|", TransmissionDistance.TransmissionDistanceMax);
        }
        else if (argument == "kamikaze" && _role == Role.Host)
        {
            IGC.SendBroadcastMessage(_cmdTag, "CMD|KAMIKAZE|", TransmissionDistance.TransmissionDistanceMax);
        }
        else if (argument == "kamikazeempty" && _role == Role.Host)
        {
            IGC.SendBroadcastMessage(_cmdTag, "CMD|KAMEMPTY|1|", TransmissionDistance.TransmissionDistanceMax);
        }
        else if (argument == "resupply" && _role == Role.Host)
        {
            IGC.SendBroadcastMessage(_cmdTag, "CMD|KAMEMPTY|0|", TransmissionDistance.TransmissionDistanceMax);
        }
        else if (argument == "ceasefire" && _role == Role.Host)
        {
            IGC.SendBroadcastMessage(_cmdTag, "CMD|CEASEFIRE|", TransmissionDistance.TransmissionDistanceMax);
        }
        else if (argument == "rearm" && _role == Role.Host)
        {
            IGC.SendBroadcastMessage(_cmdTag, "CMD|REARM|", TransmissionDistance.TransmissionDistanceMax);
        }
        else if (argument == "jump" && _role == Role.Host)
        {
            if (_jumpDrives.Count > 0)
            {
                Vector3D tgt = _jumpDrives[0].GetValue<Vector3D>("JumpTarget");
                _sb.Clear();
                _sb.Append("CMD|JUMP|");
                AppendVector(tgt);
                IGC.SendBroadcastMessage(_cmdTag, _sb.ToString(), TransmissionDistance.TransmissionDistanceMax);
                _jumpDrives[0].ApplyAction("Jump");
            }
        }
    }

    _dt = Runtime.TimeSinceLastRun.TotalSeconds;
    if (_dt <= 0) _dt = 1.0/6.0; // defensive on first tick (~Update10)
    _timeSinceTelemetry += _dt;
    _tick++;

    if (_role == Role.Satellite &&
        _timeSinceTelemetry > 43200.0 && _timeSinceTelemetry < 100000.0)
    {
        bool hasJump = false;
        for (int i=0; i<_jumpDrives.Count; i++)
        {
            var jd = _jumpDrives[i];
            if (jd != null && jd.IsWorking) { hasJump = true; break; }
        }
        if (!hasJump)
        {
            for (int i=0; i<_warheads.Count; i++)
            {
                var w = _warheads[i];
                if (w != null)
                {
                    w.IsArmed = true;
                    w.Detonate();
                }
            }
        }
    }

    if (_friendListener != null)
    {
        while (_friendListener.HasPendingMessage)
        {
            var msg = _friendListener.AcceptMessage();
            var s = msg.Data as string;
            if (s != null)
            {
                long id;
                if (long.TryParse(s, out id))
                    _friendGrids.Add(id);
            }
        }
    }

    if (_role == Role.Host) HostStep();
    else SatStep();

    WeaponStep();

    EchoStatus();
}

#region Weapons

void CategorizeWeapon(IMyTerminalBlock w)
{
    string disp = w.DefinitionDisplayNameText;
    if (!string.IsNullOrEmpty(disp))
    {
        if (disp.IndexOf("Gatling", System.StringComparison.OrdinalIgnoreCase) >= 0 ||
            disp.IndexOf("Autocannon", System.StringComparison.OrdinalIgnoreCase) >= 0 ||
            disp.IndexOf("Machine Gun", System.StringComparison.OrdinalIgnoreCase) >= 0)
            _lowPowerWeapons.Add(w);
    }
}

void WeaponStep()
{
    if (_role == Role.Satellite) MonitorAmmo();
    if (!_weaponsEnabled)
    {
        CeaseFire();
        UpdateShields(false);
        return;
    }
    if (_trackingTurret == null || _weapons.Count == 0)
    {
        UpdateShields(false);
        return;
    }

    long targetId; Vector3D targetPos; double targetDist; bool targetSmall;
    bool hasTarget = TryGetWeaponTarget(out targetId, out targetPos, out targetDist, out targetSmall);
    bool enemyNearby = hasTarget && targetDist <= _shieldDisableRange;

    UpdateShields(enemyNearby);
    if (enemyNearby && targetDist <= 12000.0)
        FireWeapons(targetId, targetPos, targetSmall);
    else
        CeaseFire();
}

bool TryGetWeaponTarget(out long id, out Vector3D pos, out double dist)
{
    bool small;
    return TryGetWeaponTarget(out id, out pos, out dist, out small);
}

bool TryGetWeaponTarget(out long id, out Vector3D pos, out double dist, out bool smallGrid)
{
    id = 0; pos = Vector3D.Zero; dist = double.MaxValue; smallGrid = false;
    if (_trackingTurret == null) return false;

    bool hasTarget = false;
    long targetId = 0; Vector3D targetPos = Vector3D.Zero; double targetDist = double.MaxValue; bool targetSmall = false;

    var vt = _trackingTurret as IMyLargeTurretBase;
    if (vt != null)
    {
        var info = vt.GetTargetedEntity();
        if (!info.IsEmpty())
        {
            hasTarget = true;
            targetId = info.EntityId;
            targetPos = info.Position;
            targetDist = Vector3D.Distance(targetPos, vt.GetPosition());
            targetSmall = (info.Type == MyDetectedEntityType.SmallGrid);
        }
    }
    else if (_trackingTurret.GetProperty("WC_TargetLock") != null)
    {
        targetId = _trackingTurret.GetValue<long>("WC_TargetLock");
        if (targetId != 0)
        {
            hasTarget = true;
            if (_trackingTurret.GetProperty("WC_TargetPosition") != null)
            {
                targetPos = _trackingTurret.GetValue<Vector3D>("WC_TargetPosition");
                targetDist = Vector3D.Distance(targetPos, _trackingTurret.GetPosition());
            }
        }
    }


    if (!hasTarget || targetId == 0 || _friendGrids.Contains(targetId)) return false;
    id = targetId; pos = targetPos; dist = targetDist; smallGrid = targetSmall;
    return true;

}

void FireWeapons(long id, Vector3D tpos, bool smallGrid)
{
    const double ALIGN_COS = 0.98; // ~11 deg
    for (int i=0; i<_weapons.Count; i++)
    {
        var w = _weapons[i];
        bool isLow = _lowPowerWeapons.Contains(w);
        if (smallGrid && !isLow)
        {
            w.ApplyAction("Shoot_Off");
            if (w.GetProperty("WC_TargetLock") != null)
                w.SetValue<long>("WC_TargetLock", 0L);
            continue;
        }
        bool canShoot = true;
        var turret = w as IMyLargeTurretBase;
        if (turret == null)
        {
            Vector3D toTarget = tpos - w.WorldMatrix.Translation;
            if (toTarget.LengthSquared() > 1e-3)
            {
                double dot = Vector3D.Dot(w.WorldMatrix.Forward, Vector3D.Normalize(toTarget));
                if (dot < ALIGN_COS) canShoot = false;
            }
        }
        if (canShoot)
            w.ApplyAction("Shoot_On");
        else
            w.ApplyAction("Shoot_Off");
        if (w.GetProperty("WC_TargetLock") != null)
            w.SetValue<long>("WC_TargetLock", id);
    }
}

void CeaseFire()
{
    for (int i=0; i<_weapons.Count; i++)
    {
        var w = _weapons[i];
        w.ApplyAction("Shoot_Off");
        if (w.GetProperty("WC_TargetLock") != null)
            w.SetValue<long>("WC_TargetLock", 0L);
    }
}

void UpdateShields(bool enemyNearby)
{
    for (int i=0; i<_shields.Count; i++)
    {
        var s = _shields[i];
        if (enemyNearby)
        {
            if (!s.Enabled) s.Enabled = true;
        }
        else
        {
            if (s.Enabled) s.Enabled = false;
        }
    }
}

void MonitorAmmo()
{
    if (_weapons.Count == 0) return;
    bool hasAmmo = false;
    for (int i=0; i<_weapons.Count; i++)
    {
        var gun = _weapons[i];
        if (gun == null || gun.InventoryCount == 0) continue;
        var inv = gun.GetInventory(0);
        _ammoTmp.Clear();
        inv.GetItems(_ammoTmp);
        if (_ammoTmp.Count > 0) { hasAmmo = true; break; }
    }
    if (!hasAmmo)
    {
        if (_kamikazeOnEmpty)
        {
            if (!_kamikaze)
            {
                _kamikaze = true;
                for (int i=0; i<_warheads.Count; i++)
                {
                    var w = _warheads[i];
                    if (w != null) w.IsArmed = true;
                }
            }
        }
        else if (_antenna != null && !_resupplySignaled)
        {
            _antenna.HudText = "Needs Resupply";
            _resupplySignaled = true;
        }
    }
    else
    {
        if (_resupplySignaled && _antenna != null)
        {
            _antenna.HudText = _antennaHud;
            _resupplySignaled = false;
        }
    }
}

#endregion

#region Host

void HostStep()
{
    if (_controller == null) return;

    // 2 Hz telemetry (every 3rd tick of Update10 ~6 Hz)
    if ((_tick % 3) == 0) SendTelemetry();

    // Optional: consume status messages
    if (_statusListener != null)
    {
        while (_statusListener.HasPendingMessage)
        {
            var msg = _statusListener.AcceptMessage();
            // (ignored for now)
        }
    }
}

void SendTelemetry()
{
    Vector3D pos = _controller.GetPosition();
    Vector3D vel = _controller.GetShipVelocities().LinearVelocity;
    MatrixD wm  = _controller.WorldMatrix;

    // payload: hostId|tick|pos|vel|fwd|up| (all doubles invariant)
    _sb.Clear();
    _sb.Append(_controller.CubeGrid.EntityId); _sb.Append('|');
    _sb.Append(_tick); _sb.Append('|');
    AppendVector(pos); AppendVector(vel); AppendVector(wm.Forward); AppendVector(wm.Up);

    // Use max distance so different grids receive it over antenna
    IGC.SendBroadcastMessage(_hostTag, _sb.ToString(), TransmissionDistance.TransmissionDistanceMax);
}

void AppendVector(Vector3D v)
{
    _sb.Append(v.X.ToString("R", CI)); _sb.Append('|');
    _sb.Append(v.Y.ToString("R", CI)); _sb.Append('|');
    _sb.Append(v.Z.ToString("R", CI)); _sb.Append('|');
}

#endregion

#region Satellite

void SatStep()
{
    // Poll host telemetry
    if (_hostListener != null)
    {
        while (_hostListener.HasPendingMessage)
        {
            var msg = _hostListener.AcceptMessage();
            var s = msg.Data as string;
            if (s != null && msg.Tag == _hostTag)
            {
                if (ParseTelemetry(s))
                    _timeSinceTelemetry = 0.0;
            }
        }
    }

    // Poll command messages
    if (_cmdListener != null)
    {
        while (_cmdListener.HasPendingMessage)
        {
            var msg = _cmdListener.AcceptMessage();
            var s = msg.Data as string;
            if (s != null && msg.Tag == _cmdTag && s.StartsWith("CMD|", System.StringComparison.Ordinal))
            {
                var parts = s.Split('|');
                if (parts.Length > 1)
                {
                    if (parts[1] == "DETONATE")
                    {
                        for (int i=0; i<_warheads.Count; i++)
                        {
                            var w = _warheads[i];
                            if (w != null)
                            {
                                w.IsArmed = true;
                                w.Detonate();
                            }
                        }
                    }
                    else if (parts[1] == "KAMIKAZE")
                    {
                        _kamikaze = true;
                        for (int i=0; i<_warheads.Count; i++)
                        {
                            var w = _warheads[i];
                            if (w != null) w.IsArmed = true;
                        }
                    }
                    else if (parts[1] == "KAMEMPTY")
                    {
                        bool enable = parts.Length > 2 && parts[2] == "1";
                        _kamikazeOnEmpty = enable;
                        if (!enable && _antenna != null)
                        {
                            _antenna.HudText = _antennaHud;
                            _resupplySignaled = false;
                        }
                    }
                    else if (parts[1] == "CEASEFIRE")
                    {
                        _weaponsEnabled = false;
                        CeaseFire();
                    }
                    else if (parts[1] == "REARM")
                    {
                        _weaponsEnabled = true;
                    }
                    else if (parts[1] == "JUMP" && parts.Length >= 5)
                    {
                        int idx = 2;
                        Vector3D tgt;
                        if (TryReadVec(parts, ref idx, out tgt))
                        {
                            _jumpTarget = tgt;
                            _jumpDelay = 1.0; // small delay before jumping
                        }
                    }
                }
            }
        }
    }

    if (_jumpDelay >= 0.0)
    {
        _jumpDelay -= _dt;
        if (_jumpDelay <= 0.0)
        {
            for (int i=0; i<_jumpDrives.Count; i++)
            {
                var jd = _jumpDrives[i];
                if (jd != null)
                {
                    jd.SetValue<Vector3D>("JumpTarget", _jumpTarget);
                    jd.ApplyAction("Jump");
                }
            }
            _jumpDelay = -1.0;
        }
    }

    // Telemetry lost: after 10s, disarm
    if (_timeSinceTelemetry > 10.0)
    {
        ClearOverrides();
        return;
    }

    // Control step ~6 Hz (every Update10 tick)
    ControlStep();

    // Status ~0.5 Hz (every 12th Update10 tick)
    if ((_tick % 12) == 0) SendStatus();
}

bool ParseTelemetry(string s)
{
    // Expect 14 fields: hostId|tick|pos(3)|vel(3)|fwd(3)|up(3)| (trailing '|')
    var parts = s.Split('|'); // 2 Hz → acceptable alloc
    if (parts.Length < 14) return false;

    int idx = 0;
    long hostId; int hostTick;
    if (!long.TryParse(parts[idx++], System.Globalization.NumberStyles.Integer, CI, out hostId)) return false;
    if (!int.TryParse(parts[idx++], System.Globalization.NumberStyles.Integer, CI, out hostTick)) return false;

    Vector3D pos, vel, fwd, up;
    if (!TryReadVec(parts, ref idx, out pos)) return false;
    if (!TryReadVec(parts, ref idx, out vel)) return false;
    if (!TryReadVec(parts, ref idx, out fwd)) return false;
    if (!TryReadVec(parts, ref idx, out up)) return false;

    // Build orthonormal basis (correct cross order: Right = Forward x Up)
    Vector3D r = Vector3D.Cross(fwd, up);
    if (r.LengthSquared() < 1e-8) r = Vector3D.CalculatePerpendicularVector(fwd);
    r = Vector3D.Normalize(r);
    up = Vector3D.Normalize(Vector3D.Cross(r, fwd));
    fwd = Vector3D.Normalize(fwd);

    _hostId = hostId;
    _friendGrids.Add(hostId);
    _hostTick = hostTick;
    _hostPos = pos;
    _hostVel = vel;
    _hostMatrix = MatrixD.Identity;
    _hostMatrix.Translation = pos;
    _hostMatrix.Right   = r;
    _hostMatrix.Up      = up;
    _hostMatrix.Forward = fwd;

    return true;
}

bool TryReadVec(string[] p, ref int idx, out Vector3D v)
{
    double x,y,z;
    if (idx+2 >= p.Length) { v = Vector3D.Zero; return false; }
    if (!double.TryParse(p[idx++], System.Globalization.NumberStyles.Float, CI, out x)) { v = Vector3D.Zero; return false; }
    if (!double.TryParse(p[idx++], System.Globalization.NumberStyles.Float, CI, out y)) { v = Vector3D.Zero; return false; }
    if (!double.TryParse(p[idx++], System.Globalization.NumberStyles.Float, CI, out z)) { v = Vector3D.Zero; return false; }
    v = new Vector3D(x,y,z); return true;
}

void ControlStep()
{
    if (_controller == null) return;

    if (_kamikaze)
    {
        KamikazeStep();
        return;
    }

    _shipMass = _controller.CalculateShipMass().PhysicalMass;
    Vector3D myPos = _controller.GetPosition();
    Vector3D vel   = _controller.GetShipVelocities().LinearVelocity;

    // Host-relative target and error
    Vector3D target = ComputeTarget();
    Vector3D error  = target - myPos;

    // Relative velocity (to host)
    Vector3D relVel = vel - _hostVel;

    // ARRIVAL HOLD: close & slow → zero thrust and orient per config
    if (error.LengthSquared() <= (_arrival * _arrival) && relVel.LengthSquared() < 0.25) // ~0.5 m/s
    {
        ZeroThrust();
        long tid; Vector3D tpos; double tdist;
        if (TryGetWeaponTarget(out tid, out tpos, out tdist))
        {
            Vector3D toTarget = tpos - myPos;
            if (toTarget.LengthSquared() > 1e-6) ApplyGyros(toTarget);
        }
        else
        {
            Vector3D fromHost = myPos - _hostPos;
            switch (_parkingFace)
            {
                case FaceSide.Forward:
                case FaceSide.Backward:
                case FaceSide.Up:
                case FaceSide.Down:
                case FaceSide.Left:
                case FaceSide.Right:
                    ApplyGyrosFaceAway(_parkingFace, fromHost);
                    break;
                default:
                    if (_alignToHost) ApplyGyrosAlignToHost();
                    else              ApplyGyrosFaceAway(FaceSide.Backward, fromHost); // face host
                    break;
            }
        }
        return;
    }

    // PD (host-relative): accelCmd = Kd * (Kp*error - relVel)
    Vector3D accelCmd = (_kp * error - relVel) * _kd;

    // Gravity compensation (planetary). In space this is zero.
    Vector3D g = _controller.GetNaturalGravity();
    if (g.LengthSquared() > 1e-6) accelCmd -= g;

    // Host radius push-out: repulse when entering host radius + buffer
    Vector3D toHost = myPos - _hostPos;
    double distHost = toHost.Length();
    double innerLimit = _hostRadius + _hostBuffer;
    if (distHost > 1e-3 && distHost < innerLimit)
    {
        double t = (innerLimit - distHost) / System.Math.Max(_hostBuffer, 1e-3); // 0..1
        double sepAccel = _sepGain * t * t; // stronger close to host
        accelCmd += (toHost / distHost) * sepAccel;
    }

    // Sensor-based separation (optional)
    if (_useSensors && _sensors.Count > 0)
    {
        for (int i=0; i<_sensors.Count; i++)
        {
            var s = _sensors[i];
            if (!s.IsWorking || !s.IsActive) continue;
            var info = s.LastDetectedEntity;
            if (info.Type == MyDetectedEntityType.None) continue;

            Vector3D p = info.Position; // world
            Vector3D sep = myPos - p;
            double d = sep.Length();
            if (d > 1e-3 && d < _minSep)
                accelCmd += sep * (_sepGain / d);
        }
    }

    // Extra damping if moving away from the target (helps kill runaways)
    if (Vector3D.Dot(error, relVel) < 0) accelCmd -= relVel * 0.5;

    // Hard relative speed cap: if above MaxSpeed, strip acceleration component that increases it
    double relSpeed = relVel.Length();
    if (relSpeed > _maxSpeed)
    {
        Vector3D dir = relVel / System.Math.Max(relSpeed, 1e-3);
        double along = Vector3D.Dot(accelCmd, dir);
        if (along > 0) accelCmd -= dir * along; // keep braking/sideways only
    }

    // Clamp acceleration to a safe fraction of available thrust
    double sumThrust =
        _axisX.MaxPos + _axisX.MaxNeg +
        _axisY.MaxPos + _axisY.MaxNeg +
        _axisZ.MaxPos + _axisZ.MaxNeg;

    double maxAccel = (sumThrust > 0 && _shipMass > 1e-3) ? (sumThrust / _shipMass) * 0.6 : 5.0;
    double aMag = accelCmd.Length();
    if (aMag > maxAccel) accelCmd *= (maxAccel / aMag);

    // Map world accel -> GRID local axes (use CubeGrid frame to match thruster categorization)
    MatrixD grid = Me.CubeGrid.WorldMatrix;
    Vector3D localAccel = Vector3D.TransformNormal(accelCmd, MatrixD.Transpose(grid));

    ApplyThrust(_axisX, localAccel.X);
    ApplyThrust(_axisY, localAccel.Y);
    ApplyThrust(_axisZ, localAccel.Z);

    // Gyro steering: face enemy target if available, otherwise accel/host
    Vector3D steer = (accelCmd.LengthSquared() > 1.0) ? accelCmd : (_hostPos - myPos);
    long tid; Vector3D tpos; double tdist;
    if (TryGetWeaponTarget(out tid, out tpos, out tdist))
    {
        Vector3D toTarget = tpos - myPos;
        if (toTarget.LengthSquared() > 1e-6) steer = toTarget;
    }
    ApplyGyros(steer);
}

bool TryGetKamikazeTarget(out Vector3D pos)
{
    long tid; double dist;
    if (TryGetWeaponTarget(out tid, out pos, out dist)) return true;
    pos = Vector3D.Zero;
    return false;
}

void KamikazeStep()
{
    Vector3D targetPos;
    if (!TryGetKamikazeTarget(out targetPos)) return;
    Vector3D myPos = _controller.GetPosition();
    Vector3D toTarget = targetPos - myPos;
    double dist = toTarget.Length();
    if (dist > 1e-3)
    {
        Vector3D dir = toTarget / dist;
        MatrixD grid = Me.CubeGrid.WorldMatrix;
        Vector3D local = Vector3D.TransformNormal(dir, MatrixD.Transpose(grid));
        FullThrustAxis(_axisX, local.X);
        FullThrustAxis(_axisY, local.Y);
        FullThrustAxis(_axisZ, local.Z);
        ApplyGyros(toTarget);
    }

    if (dist < 25.0)
    {
        for (int i=0; i<_warheads.Count; i++)
        {
            var w = _warheads[i];
            if (w != null)
            {
                w.IsArmed = true;
                w.Detonate();
            }
        }
    }
}

Vector3D ComputeTarget()
{
    int s = _shellOfIndex;
    int n = _pointsPerShell * (s+1);
    int j = _pointInShell;

    double y = 1.0 - 2.0 * (j + 0.5) / System.Math.Max(1, n);
    double r = System.Math.Sqrt(System.Math.Max(0.0, 1.0 - y*y));
    double theta = j * GOLDEN_ANGLE;
    double x = r * System.Math.Cos(theta);
    double z = r * System.Math.Sin(theta);

    Vector3D unit = new Vector3D(x, y, z);
    double radius = _baseRadius + s * _shellSpacing;

    // Rotate with host frame
    return _hostPos
         + radius * ( _hostMatrix.Right   * unit.X
                    + _hostMatrix.Up      * unit.Y
                    + _hostMatrix.Forward * unit.Z );
}

void ApplyThrust(ThrusterAxis axis, double accel)
{
    if (axis.Pos.Count == 0 && axis.Neg.Count == 0) return;

    if (accel >= 0)
    {
        double force = accel * _shipMass;
        double pct = (axis.MaxPos > 1e-3) ? (force / axis.MaxPos) : 0.0;
        if (pct < 0.02) pct = 0.0; // deadzone
        if (pct > 1) pct = 1;
        for (int i=0; i<axis.Pos.Count; i++) axis.Pos[i].ThrustOverridePercentage = (float)pct;
        for (int i=0; i<axis.Neg.Count; i++) axis.Neg[i].ThrustOverridePercentage = 0f;
    }
    else
    {
        double force = -accel * _shipMass;
        double pct = (axis.MaxNeg > 1e-3) ? (force / axis.MaxNeg) : 0.0;
        if (pct < 0.02) pct = 0.0; // deadzone
        if (pct > 1) pct = 1;
        for (int i=0; i<axis.Neg.Count; i++) axis.Neg[i].ThrustOverridePercentage = (float)pct;
        for (int i=0; i<axis.Pos.Count; i++) axis.Pos[i].ThrustOverridePercentage = 0f;
    }
}

void FullThrustAxis(ThrusterAxis axis, double dir)
{
    if (axis.Pos.Count == 0 && axis.Neg.Count == 0) return;

    if (dir > 0.01)
    {
        for (int i=0; i<axis.Pos.Count; i++) axis.Pos[i].ThrustOverridePercentage = 1f;
        for (int i=0; i<axis.Neg.Count; i++) axis.Neg[i].ThrustOverridePercentage = 0f;
    }
    else if (dir < -0.01)
    {
        for (int i=0; i<axis.Neg.Count; i++) axis.Neg[i].ThrustOverridePercentage = 1f;
        for (int i=0; i<axis.Pos.Count; i++) axis.Pos[i].ThrustOverridePercentage = 0f;
    }
    else
    {
        for (int i=0; i<axis.Pos.Count; i++) axis.Pos[i].ThrustOverridePercentage = 0f;
        for (int i=0; i<axis.Neg.Count; i++) axis.Neg[i].ThrustOverridePercentage = 0f;
    }
}

void ApplyGyros(Vector3D desiredWorld)
{
    if (_controller == null || desiredWorld.LengthSquared() < 1e-8) return;

    Vector3D fwd = _controller.WorldMatrix.Forward;
    Vector3D targetDir = Vector3D.Normalize(desiredWorld);

    Vector3D axis = fwd.Cross(targetDir);
    double sinAngle = axis.Length();
    if (sinAngle < 1e-8) axis = Vector3D.Zero;
    else axis /= sinAngle;

    // Small-angle approx: rotation vector magnitude ~= angle
    double angle = System.Math.Asin(System.Math.Min(1.0, System.Math.Max(-1.0, sinAngle)));
    Vector3D rotVec = axis * (angle * _gyroKp);

    // To ship local and apply damping
    MatrixD invShip = MatrixD.Transpose(_controller.WorldMatrix);
    Vector3D localRot = Vector3D.TransformNormal(rotVec, invShip);
    Vector3D angVel = _controller.GetShipVelocities().AngularVelocity;
    Vector3D localAng = Vector3D.TransformNormal(angVel, invShip);
    localRot -= localAng * _gyroKd;

    // Modest clamp
    if (localRot.X > 2) localRot.X = 2; if (localRot.X < -2) localRot.X = -2;
    if (localRot.Y > 2) localRot.Y = 2; if (localRot.Y < -2) localRot.Y = -2;
    if (localRot.Z > 2) localRot.Z = 2; if (localRot.Z < -2) localRot.Z = -2;

    for (int i=0; i<_gyros.Count; i++)
    {
        var g = _gyros[i];
        MatrixD inv = MatrixD.Transpose(g.WorldMatrix);
        Vector3D gyroVec = Vector3D.TransformNormal(localRot, inv);
        g.GyroOverride = true;
        g.Pitch = (float)gyroVec.X;
        g.Yaw   = (float)gyroVec.Y;
        g.Roll  = (float)gyroVec.Z;
    }
}
// Orient a chosen ship face away from the host direction.
// Returns true when within deadzone.
bool ApplyGyrosFaceAway(FaceSide side, Vector3D hostDir)
{
    if (_controller == null) return true;
    double lenSq = hostDir.LengthSquared();
    if (lenSq < 1e-8)
    {
        for (int i=0; i<_gyros.Count; i++)
        {
            var g = _gyros[i];
            g.GyroOverride = false;
            g.Pitch = g.Yaw = g.Roll = 0f;
        }
        return true;
    }
    Vector3D outDir = hostDir / System.Math.Sqrt(lenSq);
    Vector3D tF, tU, tR;
    if (side == FaceSide.Forward || side == FaceSide.Backward)
    {
        tF = (side == FaceSide.Forward) ? outDir : -outDir;
        Vector3D upHint = _hostMatrix.Up;
        upHint -= tF * Vector3D.Dot(upHint, tF);
        if (upHint.LengthSquared() < 1e-6)
            upHint = _hostMatrix.Right - tF * Vector3D.Dot(_hostMatrix.Right, tF);
        tU = Vector3D.Normalize(upHint);
        tR = tF.Cross(tU);
    }
    else if (side == FaceSide.Up || side == FaceSide.Down)
    {
        tU = (side == FaceSide.Up) ? outDir : -outDir;
        Vector3D fHint = _hostMatrix.Forward;
        fHint -= tU * Vector3D.Dot(fHint, tU);
        if (fHint.LengthSquared() < 1e-6)
            fHint = _hostMatrix.Right - tU * Vector3D.Dot(_hostMatrix.Right, tU);
        tF = Vector3D.Normalize(fHint);
        tR = tF.Cross(tU);
    }
    else
    {
        tR = (side == FaceSide.Right) ? outDir : -outDir;
        Vector3D uHint = _hostMatrix.Up;
        uHint -= tR * Vector3D.Dot(uHint, tR);
        if (uHint.LengthSquared() < 1e-6)
            uHint = _hostMatrix.Forward - tR * Vector3D.Dot(_hostMatrix.Forward, tR);
        tU = Vector3D.Normalize(uHint);
        tF = tU.Cross(tR);
    }
    Vector3D sF = _controller.WorldMatrix.Forward;
    Vector3D sU = _controller.WorldMatrix.Up;
    Vector3D sR = _controller.WorldMatrix.Right;
    Vector3D err = sF.Cross(tF) + sU.Cross(tU) + sR.Cross(tR);
    double errMag = err.Length();
    if (errMag < _alignDeadzoneRad)
    {
        for (int i=0; i<_gyros.Count; i++)
        {
            var g = _gyros[i];
            g.GyroOverride = true;
            g.Pitch = g.Yaw = g.Roll = 0f;
        }
        return true;
    }
    MatrixD invShip = MatrixD.Transpose(_controller.WorldMatrix);
    Vector3D localRot = Vector3D.TransformNormal(err, invShip) * _alignKp;
    if (localRot.X > 2) localRot.X = 2; if (localRot.X < -2) localRot.X = -2;
    if (localRot.Y > 2) localRot.Y = 2; if (localRot.Y < -2) localRot.Y = -2;
    if (localRot.Z > 2) localRot.Z = 2; if (localRot.Z < -2) localRot.Z = -2;
    for (int i=0; i<_gyros.Count; i++)
    {
        var g = _gyros[i];
        MatrixD inv = MatrixD.Transpose(g.WorldMatrix);
        Vector3D gyroVec = Vector3D.TransformNormal(localRot, inv);
        g.GyroOverride = true;
        g.Pitch = (float)gyroVec.X;
        g.Yaw   = (float)gyroVec.Y;
        g.Roll  = (float)gyroVec.Z;
    }
    return false;
}


// Rotate the ship to match the host's orientation (Forward & Up).
// Returns true if within deadzone (i.e., aligned enough).
bool ApplyGyrosAlignToHost()
{
    if (_controller == null) return true;

    // Ship & target bases
    Vector3D sF = _controller.WorldMatrix.Forward;
    Vector3D sU = _controller.WorldMatrix.Up;
    Vector3D sR = _controller.WorldMatrix.Right;

    Vector3D tF = _hostMatrix.Forward;
    Vector3D tU = _hostMatrix.Up;
    Vector3D tR = _hostMatrix.Right;

    // Orientation error as rotation vector (sum of basis cross products)
    Vector3D err = sF.Cross(tF) + sU.Cross(tU) + sR.Cross(tR);

    double errMag = err.Length();
    if (errMag < _alignDeadzoneRad)
    {
        // Close enough: turn off overrides to avoid micro-spin
        for (int i=0; i<_gyros.Count; i++)
        {
            var g = _gyros[i];
            g.GyroOverride = false;
            g.Pitch = g.Yaw = g.Roll = 0f;
        }
        return true;
    }

    // Drive gyros with proportional rates in each gyro's local space
    MatrixD invShip = MatrixD.Transpose(_controller.WorldMatrix);
    Vector3D localRot = Vector3D.TransformNormal(err, invShip) * _alignKp;

    // Modest clamp
    if (localRot.X > 2) localRot.X = 2; if (localRot.X < -2) localRot.X = -2;
    if (localRot.Y > 2) localRot.Y = 2; if (localRot.Y < -2) localRot.Y = -2;
    if (localRot.Z > 2) localRot.Z = 2; if (localRot.Z < -2) localRot.Z = -2;

    for (int i=0; i<_gyros.Count; i++)
    {
        var g = _gyros[i];
        MatrixD inv = MatrixD.Transpose(g.WorldMatrix);
        Vector3D gyroVec = Vector3D.TransformNormal(localRot, inv);
        g.GyroOverride = true;
        g.Pitch = (float)gyroVec.X;
        g.Yaw   = (float)gyroVec.Y;
        g.Roll  = (float)gyroVec.Z;
    }
    return false;
}

void SendStatus()
{
    if (_controller == null) return;
    Vector3D pos = _controller.GetPosition();
    _sb.Clear();
    _sb.Append(Me.CubeGrid.EntityId); _sb.Append('|');
    _sb.Append(_index); _sb.Append('|');
    AppendVector(pos);
    IGC.SendBroadcastMessage(_statusTag, _sb.ToString(), TransmissionDistance.TransmissionDistanceMax);
}

#endregion

#region Utils

void ZeroThrust()
{
    for (int i=0; i<_axisX.Pos.Count; i++) _axisX.Pos[i].ThrustOverridePercentage = 0f;
    for (int i=0; i<_axisX.Neg.Count; i++) _axisX.Neg[i].ThrustOverridePercentage = 0f;
    for (int i=0; i<_axisY.Pos.Count; i++) _axisY.Pos[i].ThrustOverridePercentage = 0f;
    for (int i=0; i<_axisY.Neg.Count; i++) _axisY.Neg[i].ThrustOverridePercentage = 0f;
    for (int i=0; i<_axisZ.Pos.Count; i++) _axisZ.Pos[i].ThrustOverridePercentage = 0f;
    for (int i=0; i<_axisZ.Neg.Count; i++) _axisZ.Neg[i].ThrustOverridePercentage = 0f;
}

void ClearOverrides()
{
    for (int i=0; i<_thrusters.Count; i++)
        _thrusters[i].ThrustOverridePercentage = 0f;

    for (int i=0; i<_gyros.Count; i++)
    {
        var g = _gyros[i];
        g.GyroOverride = false;
        g.Pitch = g.Yaw = g.Roll = 0f;
    }
}

void EchoStatus()
{
    _echo.Clear();
    _echo.Append("Role: ").Append(_role.ToString()).Append('\n');

    if (_role == Role.Satellite)
    {
        _echo.Append("Idx ").Append(_index)
             .Append("  s").Append(_shellOfIndex)
             .Append(" j").Append(_pointInShell).Append('\n');
        _echo.Append("Telm ").Append(System.Math.Round(_timeSinceTelemetry,1).ToString(CI)).Append("s\n");
    }

    if (_sensorFallback)
        _echo.Append("WARN: no [Swarm Sensor]\n");

    if (_debug && _controller != null)
    {
        _echo.Append("Gyros ").Append(_gyros.Count).Append("  Thr ").Append(_thrusters.Count).Append('\n');
    }

    Echo(_echo.ToString());
}

#endregion
