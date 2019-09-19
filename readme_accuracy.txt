RoR set to 70.00
Start Temp set to 20.00
Going from 20.00C to 20.50C in 7ms
Ramping up:  7
Going from 20.50C to 20.00C in -7ms
Ramping down:  7
Start Temp set to 20.00
Going from 20.00C to 20.50C in 7.14ms or 111 tics
Ramping up:  111
Going from 20.50C to 20.00C in -7.14ms or -111 tics
Ramping down:  111

That's what we did: First using SET up and down, then SETPREC up and down

Pulse 1: 7010467968 ps
Pulse 2: 7011051264 ps

DIFF up-down 583296 = 0.5833 us

then SETPREC

Pulse 1: 7100392768 ps
Pulse 2: 7100420544 ps


GAnz korrekt wäre 7142857143 

DIFF up-down 27776 = 0.0278us

SUMMARY; SETPREC uses more finegrained up and down and might get more stable results.  