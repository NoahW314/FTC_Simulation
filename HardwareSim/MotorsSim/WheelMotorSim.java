package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim;

import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.HardwareMapSim;

public class WheelMotorSim extends MotorSim{
	public WheelMotorSim(MotorSim motor) {
		super(motor);
	}
	public WheelMotorSim(HardwareMapSim hwMap, String name) {
		this(hwMap.dcMotor.get(name));
		hwMap.dcMotor.replace(name, this, this.motorObject);
	}
}
