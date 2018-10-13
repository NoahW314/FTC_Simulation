package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.DriveTests;

import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.DriveSim.turnDirection;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.TankDriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.TankDriveSim.driveDirection;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.OpModeSim;

public class TankDriveTestSim extends OpModeSim {
	
	private MotorSim leftMotor;
	private MotorSim rightMotor;
	
	private TankDriveSim drive;

	@Override
	public void init() {
		leftMotor = hardwareMap.dcMotor.get("leftMotor");
		rightMotor = hardwareMap.dcMotor.get("rightMotor");
		
		drive = new TankDriveSim(new MotorSim[]{leftMotor, rightMotor});
	}

	@Override
	public void loop() {
		drive.speed = 1;
		
		drive.turnSpeed = gamepad1.right_trigger;
		drive.turn(turnDirection.CW);
		drive.turnSpeed = gamepad1.left_trigger;
		drive.turn(turnDirection.CCW);
		
		drive.driveSpeed = gamepad1.left_stick_y;
		drive.driveDirection(driveDirection.FORWARD);
		
		drive.drive();
	}

}
