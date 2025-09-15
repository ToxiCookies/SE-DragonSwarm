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
}
