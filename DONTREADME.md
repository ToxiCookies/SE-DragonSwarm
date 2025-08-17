## DragonSwarm Commands

Run these from the host Programmable Block terminal or via timer blocks.

* `boom` – broadcasts `CMD|DETONATE|` causing satellites to immediately detonate.
* `kamikaze` – broadcasts `CMD|KAMIKAZE|` ordering satellites to dive toward their current target and detonate when within ~25 m.
* `ceasefire` – broadcasts `CMD|CEASEFIRE|` disabling satellite weapons until rearmed.
* `rearm` – broadcasts `CMD|REARM|` allowing satellites to fire again.
* `kamikazeempty` – broadcasts `CMD|KAMEMPTY|1|` so satellites ram the nearest hostile grid once out of ammo.
* `resupply` – broadcasts `CMD|KAMEMPTY|0|` restoring the default behaviour of signaling for resupply.
