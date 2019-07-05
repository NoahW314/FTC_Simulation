package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.DriveTests;

import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.Drive;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.TankDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TankDriveTest extends OpMode {
	
	//wheel motors and drivetrain
	private DcMotor leftMotor;
	private DcMotor rightMotor;
	
	private TankDrive drive;

	@Override
	public void init() {
		//initialize motors and drivetrain
		leftMotor = hardwareMap.dcMotor.get("leftMotor");
		rightMotor = hardwareMap.dcMotor.get("rightMotor");
		
		drive = new TankDrive(new DcMotor[]{leftMotor, rightMotor});
	}

	@Override
	public void loop() {
		//always drive at full speed
		drive.speed = 1;
		
		//turn clockwise with the right trigger
		drive.turnSpeed = gamepad1.right_trigger;
		drive.turn(Drive.turnDirection.CW);
		//turn clockwise with the left trigger
		drive.turnSpeed = gamepad1.left_trigger;
		drive.turn(Drive.turnDirection.CCW);
		
		//drive according to the left joystick
		drive.driveSpeed = gamepad1.left_stick_y;
		drive.driveDirection(TankDrive.driveDirection.FORWARD);
		
		//drive!
		drive.drive();
	}

}
