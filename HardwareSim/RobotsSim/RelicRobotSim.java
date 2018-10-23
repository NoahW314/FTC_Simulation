package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.RobotsSim;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.GamepadSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.HardwareMapSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.ServoSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.DriveSim.DriveSim.turnDirection;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.DriveSim.OmniWheelDriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.SensorsSim.BNO055IMUSim;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RelicRobotSim extends BaseRobotSim{

	public MotorSim BlueMotor;
	public MotorSim GreenMotor;
	public MotorSim RedMotor;
	public MotorSim WhiteMotor;
	
	public OmniWheelDriveSim drive;
	
	public ServoSim claw;
	public ServoSim knocker;
	
	public BNO055IMUSim imu;
	
	public double speed = 0.75;

	public RelicRobotSim(Program program) {
		super(program);
	}
	
	@Override
	public void init(HardwareMapSim hwMap) {
		
		BlueMotor = new MotorSim(hwMap.dcMotor.get("BlueMotor"));
        GreenMotor = new MotorSim(hwMap.dcMotor.get("GreenMotor"));
        RedMotor = new MotorSim(hwMap.dcMotor.get("RedMotor"));
        WhiteMotor = new MotorSim(hwMap.dcMotor.get("WhiteMotor"));

        WhiteMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GreenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BlueMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BlueMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WhiteMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GreenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RedMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive = new OmniWheelDriveSim(new MotorSim[]{BlueMotor, WhiteMotor, RedMotor, GreenMotor}, new Angle(45), program.getOpModeType());
        
        knocker = hwMap.servo.get("knocker");
        claw = hwMap.servo.get("claw");
        
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled      = true;
        parametersIMU.loggingTag          = "IMU";

        imu = hwMap.get(BNO055IMUSim.class, "imu");
        imu.initialize(parametersIMU);
        
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
	public void update(GamepadSim gamepad1, GamepadSim gamepad2) {
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
