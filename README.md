Modified version of Repetier focusing solely on CNC (Laser/Spindle).  

* Trades extruder axis for a 4th axis
* Adds tool height probing (G38)
* Resolves arc issues (smooth 4 axis control)
* Allows 32,000 evenly spaced steps on mega2560
* Retains Repetier's binary gcode transfer using Repetier Host
* Adds TMC driver control
* Adds high speed/resolution PWM channel for laser control on D45
* Adds vacuum/coolant control (M7/M8/M9/M10/M11)
* Adds partial gcode for movements support for laserGRBL files (M901 S<0/1>)
* Adds pause switch with user defined acceleration/deceleration
* Adds real time feed rate dial
* Distortion correction area can be defined in real time which allows the user to probe the current work piece like a PCB board.