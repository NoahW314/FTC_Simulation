package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim;

import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.HardwareMapSim;

public class LiftMotorSim extends LinearSlideMotorSim {

	public LiftMotorSim(MotorSim motor) {
		super(motor);
	}
	
	public LiftMotorSim(HardwareMapSim hwMap, String name) {
		this(hwMap.dcMotor.get(name));
		hwMap.dcMotor.replace(name, this, this.motorObject);
	}
	
	public LiftMotorSim(MotorSim motor, Direction direction) {
		this(motor);
		this.extendDirection = direction;
	}
	
	public LiftMotorSim(HardwareMapSim hwMap, String name, Direction direction) {
		this(hwMap, name);
		this.extendDirection = direction;
	}
}
