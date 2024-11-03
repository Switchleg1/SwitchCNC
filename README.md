Modified version of Repetier focusing soley on CNC (Laser/Spindle).  

* Trades extruder axis for a 4th axis
* Adds tool height probing (G38)
* Resolves arc issues (smooth 4 axis control)
* Allows 32,000 evenly spaced steps on mega2560
* Retains Repetiers binary gcode transfer
* Adds TMC driver control
* Adds high speed/resolution PWM channel for laser control on D45
* Adds vacuum/coolant control (M7/M8/M9/M10/M11)
* Adds partial gcode for movements support for laserGRBL files (M901 S<0/1)