package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors;

import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareMap;

public class WheelMotor extends Motor{
	public WheelMotor(Motor motor) {
		super(motor);
	}
	public WheelMotor(HardwareMap hwMap, String name) {
		this(hwMap.dcMotor.get(name));
		hwMap.dcMotor.replace(name, this, (Motor)this.controller.getMotor());
	}
}
