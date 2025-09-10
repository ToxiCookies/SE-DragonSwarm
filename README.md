## DragonSwarm

An autonomous, host-and-swarm program running from a single script. Satellites fall into designated orbit slots, arranged in concentric shells. Satellites favor lighter weapons against small targets. Missiles behave like satellites until fired, then accelerate toward their targets with controlled thrust to avoid overshooting before detonation.
Drop the respective .ini files into the custom data field, and recompile.
Very customizable., just edit the .inis

Missiles can be assigned to firing groups using the `Group` value in a `[Missile]` section of their custom data.

Friendly grid IDs are shared over an IGC tag. By default this tag is `<FormationGroup>.FRIEND`, but it can be changed via the `FriendTag` setting in the `[IDs]` section of the custom data.
Satellites wait for resupply when out of ammunition and will only broadcast a resupply request if they lack any energy weapons. Set `KamikazeOnEmpty=true` in the `[Weapons]` section to force kamikaze runs instead.

When the [Defense Shields](https://steamcommunity.com/sharedfiles/filedetails/?id=1365616918) mod is installed the script directly manages the shield controller. Add a `[Defense]` section with `ShieldTriggerDistance` to satellite custom data to set the detection range. Shields rise whenever enemies are detected within this distance and drop after ten seconds with no contacts. The script shunts inactive sides to reinforce the threatened face and toggles fortification when close threats are detected. Missiles and kamikaze drones enable structural reinforcement shortly before impact to survive until detonation.

### Commands

Run these from the host Programmable Block terminal or via timer blocks.

* `boom` – broadcasts `CMD|DETONATE|` causing satellites to immediately detonate.
* `kamikaze` – broadcasts `CMD|KAMIKAZE|` ordering satellites to dive toward their current target and detonate when within ~25 m.
* `ceasefire` – broadcasts `CMD|CEASEFIRE|` disabling satellite weapons until rearmed.
* `rearm` – broadcasts `CMD|REARM|` allowing satellites to fire again.
* `jump` – broadcasts `CMD|JUMP|x|y|z|` with the host's jump target; satellites jump to this location shortly after.
* `fire <group>` – broadcasts `CMD|FIRE|<group>|` causing missiles in the given group to strike the largest enemy grid within range.
* `kamikazeempty` – broadcasts `CMD|KAMEMPTY|1|` so satellites ram the nearest hostile grid once out of ammo.
* `resupply` – broadcasts `CMD|KAMEMPTY|0|` restoring the default behaviour of signaling for resupply.

### Failsafe

Satellites without any jump drives will automatically arm and detonate their warheads if host telemetry has been absent for more than 12 hours (but less than roughly 28 hours).
