package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim;

import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

public class SwifferMotorSim extends MotorSim {
	public Direction inDirection = Direction.FORWARD;
	public SwifferMotorSim(MotorSim motor) {
		super();
	}
	public SwifferMotorSim(MotorSim motor, Direction inDirection) {
		super();
		this.inDirection = inDirection;
	}
	
	public void runIn(double power) {
		power = Math.abs(power);
		this.setPower(inDirection == Direction.FORWARD ? power : -power);
		this.run();
	}
	public void runIn() {
		this.runIn(1);
	}
	
	public void runOut(double power) {
		power = Math.abs(power);
		this.setPower(inDirection == Direction.FORWARD ? -power : power);
		this.run();
	}
	public void runOut() {
		this.runOut(1);
	}
	
	public void stop() {
		this.setPower(0);
		this.run();
	}
}
