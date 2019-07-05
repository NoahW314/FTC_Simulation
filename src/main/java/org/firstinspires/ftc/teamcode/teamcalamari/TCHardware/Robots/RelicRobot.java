package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Robots;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareMap;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.Drive.turnDirection;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.OmniWheelDrive;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.Gamepad;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.Motor;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors.BNO055IMU;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Servos.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RelicRobot extends BaseRobot{

	public Motor BlueMotor;
	public Motor GreenMotor;
	public Motor RedMotor;
	public Motor WhiteMotor;
	
	public OmniWheelDrive drive;
	
	public Servo claw;
	public Servo knocker;
	
	public BNO055IMU imu;
	
	public double speed = 0.75;

	public RelicRobot(Program program) {
		super(program);
	}
	
	@Override
	public void init(HardwareMap hwMap) {
		
		BlueMotor = new Motor(hwMap.dcMotor.get("BlueMotor"));
        GreenMotor = new Motor(hwMap.dcMotor.get("GreenMotor"));
        RedMotor = new Motor(hwMap.dcMotor.get("RedMotor"));
        WhiteMotor = new Motor(hwMap.dcMotor.get("WhiteMotor"));

        WhiteMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GreenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BlueMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BlueMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WhiteMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GreenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RedMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive = new OmniWheelDrive(new Motor[]{BlueMotor, WhiteMotor, RedMotor, GreenMotor}, new Angle(45), program.getOpModeType());
        
        knocker = hwMap.servo.get("knocker");
        claw = hwMap.servo.get("claw");
        
        BNO055IMU.Parameters parameterU = new BNO055IMU.Parameters();
        parameterU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameterU.loggingEnabled      = true;
        parameterU.loggingTag          = "IMU";

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterU);
        
        super.init(hwMap);
        disable("claw");
	}
	
	public boolean wheelMotorsEnabled() {
		return enabledDevices.get("BlueMotor") && enabledDevices.get("WhiteMotor") &&
				enabledDevices.get("RedMotor") && enabledDevices.get("GreenMotor");

	}
	
	public void grab() {
		if(enabledDevices.get("claw")) {
			claw.setPosition(0.4);
		}
	}
	public void release() {
		if(enabledDevices.get("claw")) {
			claw.setPosition(0.7);
		}
	}
	
	public void knockLeftOff() {
		if(enabledDevices.get("knocker")) {
			knocker.setPosition(0);
		}
	}
	public void knockRightOff() {
		if(enabledDevices.get("knocker")) {
			knocker.setPosition(1);
		}
	}
	
	@Override
	public void update(Gamepad gamepad1, Gamepad gamepad2) {
		//store the joystick values in variables for easier access
        double glx = gamepad1.left_stick_x;
        double gly = gamepad1.left_stick_y;
        
        if(wheelMotorsEnabled()) {
	        drive.driveSpeed = Math.sqrt(Math.pow(glx, 2) + Math.pow(gly, 2));
	        drive.driveAtAngle(new Angle(Math.atan2(gly, glx), AngleUnit.RADIANS));
	        //if we are moving turn at half the speed
	        if (glx == 0 && gly == 0) {
	        	drive.speed = Math.abs(gamepad1.right_trigger-gamepad1.left_trigger)*speed;
	        	drive.turnSpeed = gamepad1.right_trigger;
	        	drive.turn(turnDirection.CW);
	        	drive.turnSpeed = gamepad1.left_trigger;
	        	drive.turn(turnDirection.CCW);
	        	drive.drive();
	        } else {
	        	drive.speed = Math.sqrt(Math.pow(glx, 2) + Math.pow(gly, 2)) * speed; 
	        	drive.turnSpeed = gamepad1.right_trigger/2;
	        	drive.turn(turnDirection.CW);
	        	drive.turnSpeed = gamepad1.left_trigger/2;
	        	drive.turn(turnDirection.CCW);
	        	drive.drive();
	        }
        }
        
        if(gamepad2.dpad_down){
        	release();
        }
        if(gamepad2.dpad_up){
        	grab();
        }
        
        if(gamepad2.left_bumper){
        	knockLeftOff();
        }
        if(gamepad2.right_bumper){
        	knockRightOff();
        }
	}
	
	public enum RelicPrograms implements Program{
		DRIVE{
			public OpModeType getOpModeType() {return OpModeType.TELE;}
		},
		DRIVEABS{
			public OpModeType getOpModeType() {return OpModeType.TELE;}
		},
		BLUE1{
			public OpModeType getOpModeType() {return OpModeType.AUTO;}
		},
		BLUE2{
			public OpModeType getOpModeType() {return OpModeType.AUTO;}
		},
		RED1{
			public OpModeType getOpModeType() {return OpModeType.AUTO;}
		},
		RED2{
			public OpModeType getOpModeType() {return OpModeType.AUTO;}
		}
	}

}
