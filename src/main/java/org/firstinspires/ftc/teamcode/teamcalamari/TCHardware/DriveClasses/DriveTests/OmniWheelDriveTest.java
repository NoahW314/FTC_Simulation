package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.DriveTests;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.Drive;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.OmniWheelDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OmniWheelDriveTest extends OpMode {

	//the angle of the front of the robot relative to the blue wheel
    private Angle angle = new Angle(45, AngleUnit.DEGREES);
    
    //wheel motor and drivetrain
	private DcMotor WhiteMotor;
    private DcMotor GreenMotor;
    private DcMotor BlueMotor;
    private DcMotor RedMotor;
    
	private OmniWheelDrive drive;
	
	@Override
	public void init() {
		//initialize motors
		WhiteMotor = hardwareMap.dcMotor.get("WhiteMotor");
        GreenMotor = hardwareMap.dcMotor.get("GreenMotor");
        BlueMotor = hardwareMap.dcMotor.get("BlueMotor");
        RedMotor = hardwareMap.dcMotor.get("RedMotor");

        //set the motors to run at a set speed
        BlueMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WhiteMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GreenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RedMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //set the motors to brake. Only required for the hub
        WhiteMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GreenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BlueMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
        //initialize drivetrain
		drive = new OmniWheelDrive(new DcMotor[]{BlueMotor,WhiteMotor,RedMotor,GreenMotor}, angle, OpModeType.AUTO);
	}

	@Override
	public void loop() {
		//full speed ahead
		drive.speed = 1;
		//start driving foward when the right bumper is pressed and stop when 
		//the left bumper is pressed
		if(gamepad1.right_bumper) {
			drive.driveSpeed = 1;
			drive.driveAtAngle(new Angle(0, AngleUnit.DEGREES));
		}
		else if(gamepad1.left_bumper) {
			drive.speed = 0;
		}
		
		//turn using the left and right triggers
		if(gamepad1.right_trigger > 0) {
			drive.turnSpeed = gamepad1.right_trigger;
			drive.turn(Drive.turnDirection.CW);
		}
		if(gamepad1.left_trigger > 0) {
			drive.turnSpeed = gamepad1.left_trigger;
			drive.turn(Drive.turnDirection.CCW);
		}
		
		//drive!
		drive.drive();
	}

}
