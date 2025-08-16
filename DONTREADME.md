## DragonSwarm Commands

Run these from the host Programmable Block terminal or via timer blocks.

* `boom` – broadcasts `CMD|DETONATE|` causing satellites to immediately detonate.
* `kamikaze` – broadcasts `CMD|KAMIKAZE|` ordering satellites to dive toward the host and detonate when within ~25 m.
* `kamikazeempty on/off` – broadcasts `CMD|AMMO_KAMIKAZE|ON/OFF` to toggle kamikaze-on-empty mode. When enabled, satellites without ammo seek the nearest hostile grid and explode on impact; otherwise, they rename their antenna to request resupply.
