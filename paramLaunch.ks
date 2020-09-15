clearscreen.

declare parameter hParking.
declare parameter incl.
declare parameter lan.
declare parameter argPe.
declare parameter apfinal.
declare parameter pefinal.

// =============================================================================
// FUNCTIONS
declare function vCirc {
	declare parameter altitude.
	return sqrt((body:mu)/(kerbin:radius+altitude)).
}

declare function angle1to2 {
	declare parameter vector1.
	declare parameter vector2.

	set phi to arccos((vector1*vector2)/(vector1:mag*vector2:mag)).
	set cross to vectorcrossproduct(vector1, vector2).

	if cross:y < 0 {
		set phi to 360 - phi.
	}

	return phi. 
}

declare function visViva {
	declare local parameter ha.
	declare local parameter hp.
	declare local parameter aporpe. 
	
	local ap to ha + kerbin:radius.
	local pe to hp + kerbin:radius.
	local a to (ap+pe)/2.

	if aporpe = true {
		local va to sqrt(kerbin:mu*(2/ap - 1/a)).
		return va. 
	}

	else if aporpe = false {
		local vp to sqrt(kerbin:mu*(2/pe - 1/a)).
		return vp.
	}
	
}

declare function vecHeading {
	declare parameter vec.

	set east to vectorcrossproduct(ship:up:vector, ship:north:vector).
	set x to ship:north:vector*vec.
	set y to east*vec.
	
	set head to arctan2(y,x).
	if head < 0 {
		return 360 + head.
	}

	else {
		return head.
	}
}

declare function burnTimeCalc {
	declare parameter DV.
	list engines in eng.
	local isp is eng[0]:isp.
	local ve is isp*constant:g0.
	local mf is ship:mass/(constant:E^(abs(DV)/ve)).		// final mass of the vehicle
	local mdot is ship:maxthrust/ve. 							// engine mass flow
	local Tburn is (ship:mass-mf)/mdot. 						// burn time of the manuever
	return Tburn.
}

// =============================================================================
// MAIN CODE

// CALCULATE LAUNCH AZIMUTH GIVEN INCLINATION AND PARKING ORBIT ALTITUDE
// need to adjust this for two launch windows and
set vr to (2*constant:pi*kerbin:radius)/kerbin:rotationperiod.
set vi to vCirc(hParking).
set A to arcsin(cos(incl)/cos(0)).
set vy to vi*cos(A).
set vx to vi*sin(A) - vr. 
set A to arctan(vx/vy).

print "===BOOST PHASE===" at(0,0).
print "Launch azimuth: " + round(A,2) at(0,1).
print "Pitch angle: " + 90.00 at(0,2).

// WARP TO PROPER LAUNCH WINDOW TO LAUNCH INTO CORRENT LAN
if incl <> 0 and incl <> 180 {
	set lng to ship:geoposition:lng.
	set Dtheta to abs(lng)+(lan-kerbin:rotationangle) - 1.6.
	if Dtheta < 0 {
		set Dtheta to 360 + Dtheta. 
	}

	set Dt to Dtheta/(360/kerbin:rotationperiod).
	kuniverse:timewarp:warpto(time:seconds + Dt).
	set ti to time:seconds.
	wait until time:seconds >= ti + Dt.
}






// SET THE VEHICLE TO STAGE WHEN THRUST IS GONE

set F0 to ship:maxthrust.
when ship:maxthrust < F0 or ship:maxthrust = 0 then {
	stage.
	set f0 to ship:maxthrust.
	preserve. 
}

set fairing to false.

// ASCEND TO ORBIT
lock throttle to 1.0.
set theta to 90.
lock steering to heading(A,theta,0).

until ship:apoapsis > hParking {
	if ship:velocity:surface:mag >= 75 {
		set theta to 90 - (10/125)*ship:velocity:surface:mag.
		if theta <= 10 {
			// set theta to arcsin((constant:g0*ship:mass)/(ship:maxthrust-getDrag)).
			set theta to 10.
		}
		lock steering to heading(A, theta, 0).
		print "Pitch angle (deg): " + round(theta,2) at(0,2).
	}

	if stage:solidfuel > 0 {
		if stage:solidfuel/stage:resources[2]:capacity = 0 {
			stage. 
		}
	}

	if ship:altitude > 40000 and fairing = false{
		for part in ship:modulesnamed("moduleproceduralfairing") {
			part:doevent("deploy").
			set fairing to true.
		}
	}
}

lock throttle to 0.






// CIRCULARIZE ORBIT
clearscreen. 
print "===CIRCULARIZATION===" at(0,0).

set ta to time:seconds + eta:apoapsis.
lock steering to ship:prograde.

if ship:altitude < 70000 {
	wait until ship:altitude >= 70000.
}

set kuniverse:timewarp:warp to 0.
wait 1.
kuniverse:timewarp:warpto(ta-20).

until time:seconds >= ta-1 {
	print "Time to apoapsis: " + round(eta:apoapsis,0) + " s" at(0,1).
	print "Apoapsis altitude: " + round(ship:apoapsis) + " m" at(0,2).
}

lock throttle to 1.0.
set theta to 0.

set ap to ship:apoapsis.
until 0.99*ship:periapsis <= ap and ap <= 1.01*ship:periapsis {
	print "Time to apoapsis (s): " + round(eta:apoapsis,0) + " s" at(0,1).

	set state to eta:apoapsis. 
	if state > 60 {
		set state to state - ship:orbit:period.
	}
	
	print "Pitch angle (deg): " + round(theta,2) at(0,4).
	
	if ship:periapsis > 0.75*ap {
		lock throttle to 0.5.
	}

	set pid to pidloop(1, -0.0, 1).
	set pid:setpoint to 0. 
	set theta to pid:update(time:seconds, state).
	// print round(state,2) at(0,3).
	// print round(theta,2) at(0,2).

	lock steering to heading(vecHeading(ship:prograde:vector),theta,0).
}

lock throttle to 0.
wait 5.

// GET APOAPIS TO CORRECT ALTITUDE
clearscreen.

// warp to the point angle from AN equals desired argPe
if apfinal <> hParking {
	set DV to visViva(apfinal, orbit:periapsis, true) - ship:velocity:orbit:mag .
	set tburn to DV/(ship:maxthrust/ship:mass).

	set ta to ship:orbit:trueanomaly.
	set ap to ship:orbit:argumentofperiapsis. 
	set thetafroman to ap + ta.

	if abs(ship:orbit:inclination) < 2 {
		set LP to ship:orbit:lan + ship:orbit:argumentofperiapsis. // longnitude of periapsis (equinox direction to periapsis)
		if LP > 360 {
			set LP to LP - 360.
		}
		set TA0 to LP + ship:orbit:trueanomaly.
		if TA0 > 360 {
			set TA0 to TA0 - 360.
		}
		set thetafroman to TA0.
	}

	if thetafroman > 360 {
		set thetafroman to thetafroman - 360.
	}

	set Dtheta to argPE - thetafroman.

	if Dtheta < 0 {
		set Dtheta to 360 + Dtheta.
	}

	set n to 360/ship:orbit:period.
	set Dt to Dtheta/n - tburn/2 - 10.
	set tb to time:seconds + Dt.

	set kuniverse:timewarp:warp to 0.
	wait 1.
	kuniverse:timewarp:warpto(time:seconds + Dt).
	wait until time:seconds >= tb.

	print "warp complete".
	lock steering to ship:prograde:vector.
	wait 10.
	print "burning".
	until ship:apoapsis > apfinal {
		lock throttle to 1.
	}
}

lock throttle to 0.
wait 10.

// GET PERIAPSIS TO THE CORRECT ALTITUDE
if pefinal <> hParking {
	set DV to visViva(orbit:apoapsis, pefinal, true) - visViva(orbit:apoapsis, orbit:periapsis, true).
	set tburn to burnTimeCalc(DV).

	// print tburn.
	// print eta:apoapsis - Dt

	set tb to time:seconds + eta:apoapsis - tburn/2 - 10.
	set kuniverse:timewarp:warp to 0.
	wait 1.
	kuniverse:timewarp:warpto(tb).
	wait until time:seconds >= tb.

	lock steering to ship:prograde:vector.
	wait 10.
	lock throttle to 1. 
	wait tburn.
	lock throttle to 0.
}

lock throttle to 0.
wait 3.