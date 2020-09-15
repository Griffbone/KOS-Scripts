clearscreen.
sas off.

declare function visViva {
	declare local parameter ha.
	declare local parameter hp.
	declare local parameter aporpe. 
	declare local parameter cb.
	
	local ap to ha + cb:radius.
	local pe to hp + cb:radius.
	local a to (ap+pe)/2.

	if aporpe = "ap" {
		local va to sqrt(cb:mu*(2/ap - 1/a)).
		return va. 
	}

	else if aporpe = "pe" {
		local vp to sqrt(cb:mu*(2/pe - 1/a)).
		return vp.
	}
	
}

declare parameter phi.

wait 3.
set apfinal to ship:apoapsis.
set pefinal to ship:periapsis.

// GET TRANSFER ORBIT GEOMETRY
set rad to ship:altitude + kerbin:radius.			// orbit radius 
set w to 360/ship:orbit:period. 					// ship mean motion
set Dt to phi/w. 									// time to close phase angle
set T to ship:orbit:period + Dt. 					// phasing orbit period.

set a to ((T^2*kerbin:mu)/(4*constant:pi^2))^(1/3).	// transfer orbit semimajor axis 
set ra to 2*a - (ship:periapsis + kerbin:radius). 	// transfer orbit apoapsis radius 
set ap to ra - kerbin:radius.						// transfer orbit apoapsis altitude

// GET ENGINE ISP
list engines in eng.
set isp to eng[0]:isp.
set ve to isp*constant:g0.

// GET BURN DELTA-V (SAME FOR BOTH BURNS)
set DV to visViva(ap, ship:periapsis, "pe", kerbin) - visViva(ship:apoapsis, ship:periapsis, "pe", kerbin).
set mf to ship:mass/(constant:E^(abs(DV)/ve)).		// final mass of the vehicle

// RAISE APOAPSIS
set mdot to ship:maxthrust/ve. 						// engine mass flow
set Tburn to (ship:mass-mf)/mdot. 					// burn time of the manuever
set tb to time:seconds + eta:periapsis - Tburn/2 - 20. 
kuniverse:timewarp:warpto(tb).
wait until time:seconds > tb.
lock steering to prograde. 
wait 20.
lock throttle to 1. 
wait Tburn.
lock throttle to 0.

// WAIT
wait 5.

// LOWER APOAPSIS
set mf to ship:mass/(constant:E^(abs(DV)/ve)).		// final mass of the vehicle
set Tburn to (ship:mass-mf)/mdot. 					// burn time of the manuever
set tb to time:seconds + eta:periapsis - Tburn/2 - 20. 
kuniverse:timewarp:warpto(tb).
wait until time:seconds > tb.
lock steering to retrograde. 
wait 20.
lock throttle to 1. 
wait Tburn.
lock throttle to 0.

wait 3.