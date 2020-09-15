clearscreen. 
sas off.

// FUNCTIONS
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




// SCRIPT PARAMETERS
declare parameter ap.
declare parameter pe.
declare parameter centralBod.




// GET ENGINE ISP
list engines in eng.
set isp to eng[0]:isp.
set ve to isp*constant:g0.




// ADJUST APOAPSIS OF THE ORBIT 
set DV to visViva(ap, ship:periapsis, "pe", centralBod) - visViva(ship:apoapsis, ship:periapsis, "pe", centralBod). // get burn delta-v
set mf to ship:mass/(constant:E^(abs(DV)/ve)).												// final mass of the vehicle
set mdot to (ship:mass-mf)/3.																// mass flow for a 3-second burn
set T to (mdot*ve)/ship:maxthrust.															// get throttle for a 3-second burn

set t0 to time:seconds + eta:periapsis - 20.
kuniverse:timewarp:warpto(t0).
wait until time:seconds >= t0.

if DV < 0 {
	lock steering to ship:retrograde.
}

else {
	lock steering to ship:prograde. 
}

wait 20. 

lock throttle to T. 
wait 3.
lock throttle to 0.
sas on.




// ADJUST PERIAPSIS OF THE ORBIT - perform 180 deg from first manuever (location of apoapsis/periapsis may have flipped)
wait 5. 
sas off.

set tta to ship:orbit:trueanomaly + 180.

set apo to 180-tta. 
if apo < 0 {
	set apo to 360 + apo.
}

set peri to tta. 

// burn at apoapsis
if peri < apo {
	print "burning at apoapsis".
	set Dv to visViva(ship:apoapsis, pe, "ap", centralBod) - visViva(ship:apoapsis, ship:periapsis, "ap", centralBod).
	print DV.
	set mf to ship:mass/(constant:E^(abs(DV)/ve)).												// final mass of the vehicle
	set mdot to (ship:mass-mf)/3.																// mass flow for a 3-second burn
	set T to (mdot*ve)/ship:maxthrust.	

	set t0 to time:seconds + eta:apoapsis - 20.
	kuniverse:timewarp:warpto(time:seconds + eta:apoapsis - 20).
	wait until time:seconds >= t0.
}

// burn at periapsis
else {
	print "burning at periapsis".
	set DV to visViva(ship:periapsis, pe, "ap", centralBod) - visViva(ship:apoapsis, ship:periapsis, "pe", centralBod).
	print DV.
	set mf to ship:mass/(constant:E^(abs(DV)/ve)).												// final mass of the vehicle
	set mdot to (ship:mass-mf)/3.																// mass flow for a 3-second burn
	set T to (mdot*ve)/ship:maxthrust.

	set t0 to time:seconds + eta:periapsis - 20. 
	kuniverse:timewarp:warpto(t0).
	wait until time:seconds >= t0.
}

if DV < 0 {
	lock steering to ship:retrograde.
}

else {
	lock steering to ship:prograde.
}

wait 20. 
lock throttle to T. 
wait 3. 
lock throttle to 0.
wait 3.