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
}
