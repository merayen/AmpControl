# Phases

## Third version
- Move all the amp to the new case
- Install in total 4 fans
	- [ ] Control the fans using a huge transister instead of PWM
	- [ ] Maybe read PWM from fans?

## Second version
- [ ] Move 25% of the amp to a new case, with a new 630VA-transformer.
- [ ] Rebuild PSU-board:
	- [ ] 3 fuses for each transformator (12V-0-12V transformer, 24V-0-24V transformer and 9V transformer)
	- [ ] 12V regulator (from audio transformer)
	- [ ] -12V regulator (from audio transformer)
	- [ ] 5V regulator (from digital transformer)
- [ ] Fans should still run on max speed, directly connected to the 9V-transformer
- [ ] Implement control electronics
	- [ ] Implement PIC-chip
	- [ ] Implement PSU-control
		- [ ] Turn-on
		- [ ] Slow-start/stop circuit
	- [ ] Implement PGA-chip for volume control

## First version (finished)
Build the amp in a too small case with bad cooling, cables laying everywhere, stuff floating around and only 25% of the amp can be used.

This version sucked, blew one tweeter most likely due to human error controlling the volume badly, but has otherwise kept parties alive, 5 times or so.
