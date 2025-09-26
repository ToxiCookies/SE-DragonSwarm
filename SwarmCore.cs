// Shared math and helper routines for DragonSwarm
// Lightweight utilities used by Host and Satellite scripts

static class SwarmCore
{
    const double PHI = 1.6180339887498948482; // golden ratio
    const double GOLDEN_ANGLE = 2.39996322972865332; // 2*pi*(1-1/phi)

    // Compute a Fibonacci sphere position for a given index
    public static VRageMath.Vector3D SlotPoint(int index, int total, double radius)
    {
        // Based on formula for even distribution on a sphere
        double i = (double)index + 0.5;
        double phi = System.Math.Acos(1.0 - 2.0 * i / (double)total);
        double theta = GOLDEN_ANGLE * i;
        double x = radius * System.Math.Sin(phi) * System.Math.Cos(theta);
        double y = radius * System.Math.Sin(phi) * System.Math.Sin(theta);
        double z = radius * System.Math.Cos(phi);
        return new VRageMath.Vector3D(x, y, z);
    }

    // Minimal PID controller for position -> velocity mapping
    public class PID
    {
        double _p, _d, _lastError;
        public PID(double p, double d)
        {
            _p = p; _d = d; _lastError = 0.0;
        }
        public double Step(double error, double dt)
        {
            double derivative = (error - _lastError) / dt;
            _lastError = error;
            return _p * error + _d * derivative;
        }
    }

    // Predict intercept point for a moving target given projectile speed
    public static VRageMath.Vector3D PredictIntercept(
        VRageMath.Vector3D shooterPos,
        VRageMath.Vector3D shooterVel,
        VRageMath.Vector3D targetPos,
        VRageMath.Vector3D targetVel,
        double projectileSpeed)
    {
        VRageMath.Vector3D relPos = targetPos - shooterPos;
        VRageMath.Vector3D relVel = targetVel - shooterVel;

        double a = relVel.LengthSquared() - projectileSpeed * projectileSpeed;
        double b = 2.0 * VRageMath.Vector3D.Dot(relVel, relPos);
        double c = relPos.LengthSquared();
        double t;

        if (System.Math.Abs(a) < 1e-6)
            t = -c / b;
        else
        {
            double det = b * b - 4.0 * a * c;
            if (det < 0)
                return targetPos;
            double sqrt = System.Math.Sqrt(det);
            double t1 = (-b + sqrt) / (2.0 * a);
            double t2 = (-b - sqrt) / (2.0 * a);
            t = System.Math.Max(t1, t2);
        }

        if (t < 0)
            t = 0;
        return targetPos + relVel * t;
    }

    // Shared helper for interfacing with DefenseShields blocks on a grid
    // Provides a lightweight automation wrapper that scripts can reuse
    public class DefenseShieldController
    {
        public struct ShieldInfo
        {
            public double Percent;
            public double Hitpoints;
            public double Heat;
            public double InputPower;
            public bool Online;
            public bool FortifyActive;
            public bool ModulateActive;
        }

        static readonly string[] ACTION_RAISE = new string[] { "ShieldOn", "ShieldUp", "RaiseShield", "ShieldRaise" };
        static readonly string[] ACTION_LOWER = new string[] { "ShieldOff", "ShieldDown", "LowerShield", "ShieldLower" };
        static readonly string[] ACTION_MOD_ON = new string[] { "ShieldModulate", "ShieldModulateOn", "Modulate", "ModulateOn" };
        static readonly string[] ACTION_MOD_OFF = new string[] { "ShieldDemodulate", "ShieldModulateOff", "ModulateOff", "Demodulate" };
        static readonly string[] ACTION_FORTIFY_ON = new string[] { "ShieldFortify", "ShieldFortifyOn", "Fortify", "FortifyOn" };
        static readonly string[] ACTION_FORTIFY_OFF = new string[] { "ShieldFortifyOff", "FortifyOff", "ShieldDefortify", "Defortify" };

        readonly System.Collections.Generic.List<IMyTerminalBlock> _candidates = new System.Collections.Generic.List<IMyTerminalBlock>();

        IMyTerminalBlock _shieldBlock;
        IMyTerminalBlock _controllerBlock;
        IMyTerminalBlock _modulatorBlock;

        ShieldInfo _info;
        bool _raiseState;
        bool _modulateState;
        bool _fortifyState;

        public string Tag = "[Shield]";
        public bool RequireSameGrid = true;

        public DefenseShieldController()
        {
            _info = new ShieldInfo();
        }

        public bool HasShield
        {
            get { return _shieldBlock != null || _controllerBlock != null || _modulatorBlock != null; }
        }

        public ShieldInfo Info
        {
            get { return _info; }
        }

        public void Reset()
        {
            _shieldBlock = null;
            _controllerBlock = null;
            _modulatorBlock = null;
            _info = new ShieldInfo();
            _raiseState = false;
            _modulateState = false;
            _fortifyState = false;
        }

        public bool Acquire(IMyGridTerminalSystem gts, IMyCubeGrid grid)
        {
            _shieldBlock = null;
            _controllerBlock = null;
            _modulatorBlock = null;
            _candidates.Clear();

            gts.GetBlocksOfType(_candidates, delegate (IMyTerminalBlock block)
            {
                if (block == null)
                    return false;
                if (RequireSameGrid && block.CubeGrid != grid)
                    return false;
                if (!MatchesDefenseShield(block))
                    return false;
                if (!string.IsNullOrEmpty(Tag) && block.CustomName.IndexOf(Tag, System.StringComparison.OrdinalIgnoreCase) < 0)
                    return false;
                return true;
            });

            foreach (IMyTerminalBlock block in _candidates)
            {
                string subtype = block.BlockDefinition != null ? block.BlockDefinition.SubtypeName : string.Empty;
                if (_controllerBlock == null && Contains(subtype, "Controller"))
                    _controllerBlock = block;
                else if (_modulatorBlock == null && Contains(subtype, "Modulator"))
                    _modulatorBlock = block;
                else if (_shieldBlock == null)
                    _shieldBlock = block;
            }

            if (_controllerBlock == null)
                _controllerBlock = _shieldBlock;

            return HasShield;
        }

        bool MatchesDefenseShield(IMyTerminalBlock block)
        {
            if (block.BlockDefinition == null)
                return block.CustomName.IndexOf("Shield", System.StringComparison.OrdinalIgnoreCase) >= 0;

            string typeId = block.BlockDefinition.TypeIdString;
            string subtype = block.BlockDefinition.SubtypeName;

            if (Contains(typeId, "DefenseShields"))
                return true;
            if (Contains(subtype, "DefenseShields"))
                return true;
            if (Contains(subtype, "Shield"))
                return true;
            return block.CustomName.IndexOf("Shield", System.StringComparison.OrdinalIgnoreCase) >= 0;
        }

        bool Contains(string text, string fragment)
        {
            if (string.IsNullOrEmpty(text) || string.IsNullOrEmpty(fragment))
                return false;
            return text.IndexOf(fragment, System.StringComparison.OrdinalIgnoreCase) >= 0;
        }

        IMyTerminalBlock PrimaryBlock
        {
            get
            {
                if (_controllerBlock != null) return _controllerBlock;
                if (_shieldBlock != null) return _shieldBlock;
                return _modulatorBlock;
            }
        }

        public ShieldInfo Update()
        {
            ShieldInfo info = new ShieldInfo();
            if (!HasShield)
            {
                _info = info;
                return info;
            }

            info.Percent = SampleFloat("ShieldPercent");
            info.Hitpoints = SampleFloat("ShieldHP");
            info.Heat = SampleFloat("ShieldHeat");
            info.InputPower = SampleFloat("ShieldInput");
            info.Online = SampleBool("ShieldOnline") || info.Percent > 0.01;
            info.FortifyActive = SampleBool("ShieldFortify") || _fortifyState;
            info.ModulateActive = SampleBool("ShieldModulate") || _modulateState;

            _info = info;
            return info;
        }

        public bool Maintain(bool raise, bool modulate, bool fortify)
        {
            bool ok = true;
            if (raise != _raiseState)
            {
                if (ApplyAction(raise ? ACTION_RAISE : ACTION_LOWER))
                    _raiseState = raise;
                else
                    ok = false;
            }
            if (modulate != _modulateState)
            {
                if (ApplyAction(modulate ? ACTION_MOD_ON : ACTION_MOD_OFF))
                    _modulateState = modulate;
                else
                    ok = false;
            }
            if (fortify != _fortifyState)
            {
                if (ApplyAction(fortify ? ACTION_FORTIFY_ON : ACTION_FORTIFY_OFF))
                    _fortifyState = fortify;
                else
                    ok = false;
            }
            return ok;
        }

        public bool SetRaised(bool raise)
        {
            if (!HasShield)
                return false;
            if (raise == _raiseState)
                return true;
            if (ApplyAction(raise ? ACTION_RAISE : ACTION_LOWER))
            {
                _raiseState = raise;
                return true;
            }
            return false;
        }

        public bool SetModulate(bool enable)
        {
            if (!HasShield)
                return false;
            if (enable == _modulateState)
                return true;
            if (ApplyAction(enable ? ACTION_MOD_ON : ACTION_MOD_OFF))
            {
                _modulateState = enable;
                return true;
            }
            return false;
        }

        public bool SetFortify(bool enable)
        {
            if (!HasShield)
                return false;
            if (enable == _fortifyState)
                return true;
            if (ApplyAction(enable ? ACTION_FORTIFY_ON : ACTION_FORTIFY_OFF))
            {
                _fortifyState = enable;
                return true;
            }
            return false;
        }

        float SampleFloat(string propertyId)
        {
            float value;
            if (TryReadFloat(PrimaryBlock, propertyId, out value))
                return value;
            if (_shieldBlock != null && _shieldBlock != PrimaryBlock && TryReadFloat(_shieldBlock, propertyId, out value))
                return value;
            if (_modulatorBlock != null && _modulatorBlock != PrimaryBlock && TryReadFloat(_modulatorBlock, propertyId, out value))
                return value;
            return 0f;
        }

        bool TryReadFloat(IMyTerminalBlock block, string propertyId, out float value)
        {
            value = 0f;
            if (block == null)
                return false;
            Sandbox.ModAPI.Ingame.ITerminalProperty property = block.GetProperty(propertyId);
            if (property == null)
                return false;
            var getter = property.AsFloat();
            if (getter == null)
                return false;
            value = getter.GetValue(block);
            return true;
        }

        bool SampleBool(string propertyId)
        {
            bool value;
            if (TryReadBool(PrimaryBlock, propertyId, out value))
                return value;
            if (_shieldBlock != null && _shieldBlock != PrimaryBlock && TryReadBool(_shieldBlock, propertyId, out value))
                return value;
            if (_modulatorBlock != null && _modulatorBlock != PrimaryBlock && TryReadBool(_modulatorBlock, propertyId, out value))
                return value;
            return false;
        }

        bool TryReadBool(IMyTerminalBlock block, string propertyId, out bool value)
        {
            value = false;
            if (block == null)
                return false;
            Sandbox.ModAPI.Ingame.ITerminalProperty property = block.GetProperty(propertyId);
            if (property == null)
                return false;
            var getter = property.AsBool();
            if (getter == null)
                return false;
            value = getter.GetValue(block);
            return true;
        }

        bool ApplyAction(string[] names)
        {
            if (names == null || names.Length == 0)
                return false;
            if (TryApplyAction(_controllerBlock, names))
                return true;
            if (TryApplyAction(_shieldBlock, names))
                return true;
            if (TryApplyAction(_modulatorBlock, names))
                return true;
            return false;
        }

        bool TryApplyAction(IMyTerminalBlock block, string[] names)
        {
            if (block == null)
                return false;
            foreach (string name in names)
            {
                if (string.IsNullOrEmpty(name))
                    continue;
                Sandbox.ModAPI.Ingame.ITerminalAction action = block.GetActionWithName(name);
                if (action != null)
                {
                    action.Apply(block);
                    return true;
                }
            }
            return false;
        }
    }
}
