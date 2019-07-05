package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors;

import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareMap;

public class LiftMotor extends LinearSlideMotor {

	public LiftMotor(Motor motor) {
		super(motor);
	}
	
	public LiftMotor(HardwareMap hwMap, String name) {
		this(hwMap.dcMotor.get(name));
		hwMap.dcMotor.replace(name, this, (Motor)this.controller.getMotor());
	}
	
	public LiftMotor(Motor motor, Direction direction) {
		this(motor);
		this.extendDirection = direction;
	}
	
	public LiftMotor(HardwareMap hwMap, String name, Direction direction) {
		this(hwMap, name);
		this.extendDirection = direction;
	}
}
