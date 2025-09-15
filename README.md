## DragonSwarm (Modular)

DragonSwarm has been refactored into lightweight programmable-block scripts that
communicate through a tiny custom inter-grid API.  The swarm now fits on a grid
with at most four programmable blocks:

1. **SwarmHost.cs** – runs on the host grid, assigns satellite indices and
   broadcasts commands and host telemetry.
2. **SwarmSatellite.cs** – runs on satellites or missiles.  Each instance
   listens for host telemetry, keeps to its assigned Fibonacci-sphere slot and
   executes host commands.
3. **(Optional) additional PBs** can extend behaviour (weapon logic, dedicated
   missile profiles) but the core system functions with only Host + Satellites.

Shared logic lives in `SwarmCore.cs` and `SwarmIGC.cs`.  These files contain the
formation math, a minimal PID controller and the custom API used for
inter-Programmable Block communication.  They are intended to be copied into
other scripts that participate in the swarm.

The new layout avoids heavy per-tick work: all scripts run at
`UpdateFrequency.Update10` (6 ticks per second) and cache block references at
startup.  Typical runtime is well below the 0.5 ms per tick limit.

### Usage

* Copy the desired script into a Programmable Block and recompile.
* Ensure all participating grids share the same IGC channel
  (`DRAGONSWARM` by default).
* Satellites automatically announce themselves; the host assigns orbit slots and
  continually broadcasts its position.
* Manual commands typed into the host PB argument field are rebroadcast to all
  satellites (e.g. `KAMIKAZE` or `FIRE`).

Custom data examples from the previous version remain compatible for basic
operation, though many advanced options have been trimmed for simplicity.
