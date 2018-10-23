package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.AutonomousNavigationSim;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.AutonomousNavigationSim.AutonomousNavigationSim.RobotStates;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.DriveSim.OmniWheelDriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.SensorsSim.BNO055IMUSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeSim.LinearOpModeSim;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Omni Nav Test", group="Nav Tests")
public class AutoOmniNavTestSim extends LinearOpModeSim {
	
	//the angle of the front of the robot relative to the blue wheel
    private Angle angle = new Angle(45, AngleUnit.DEGREES);
    //the speed of the robot
    private double speed = 1;
    private double driveSpeed = 0.5;
    
    private double stillTurnSpeed = 0.75;
    private double stillTurnTurnSpeed = 0.5;
    
    private double moveTurnSpeed = 0.5;
	
	//the motors, servos, sensors, servo values, and drivetrain
    private MotorSim WhiteMotor;
    private MotorSim GreenMotor;
    private MotorSim BlueMotor;
    private MotorSim RedMotor;
    
    private Servo claw;
    
    private BNO055IMU imu;
    
    private OmniWheelDriveSim drive;
    
    //navigation variables
    private int targetNum = 4;
    private AllDirectionsNavigationSim nav = new AllDirectionsNavigationSim(targetNum, speed, driveSpeed, stillTurnSpeed, stillTurnTurnSpeed, moveTurnSpeed, this);
    
    RobotAction log = new RobotAction() {
    	ElapsedTime time = new ElapsedTime();
    	public boolean start() {
    		time.reset();
    		return true;
    	}
    	public boolean act() {
    		if(time.seconds() > 1) {
    			return true;
    		}
    		else {
    			telemetry.addData("Drive Action", "Running");
    		}
    		return false;
    	}
    };
    
	@Override
	public void runOpMode() throws InterruptedException {
		nav.targets.set(0, new VectorF(0, 0));
	    nav.targets.set(1, new VectorF(24, 0));
	    nav.targets.set(2, new VectorF(24, 24));
	    nav.targets.set(3, new VectorF(0, 0));
	    
	    nav.stationaryTurns.set(0, new Angle(-45.0, AngleUnit.DEGREES));
	    nav.stationaryTurns.set(1, new Angle(125.0, AngleUnit.DEGREES));
	    
	    nav.actions.set(0, new RobotAction() {
	    	public boolean act() {
	    		claw.setPosition(0.7);
	    		return true;
	    	}
	    });
	    nav.actions.set(1, new RobotAction() {
	    	public boolean act() {
	    		claw.setPosition(0.4);
	    		return true;
	    	}
	    });
	    nav.actionFirst.set(1, true);
	    
	    nav.drivingTurns.set(1, new Angle(0.0, AngleUnit.DEGREES));
	    nav.drivingActions.set(0, log);
	    
	    nav.finalCode = new Runnable() {
	    	public void run() {
	    		System.out.println("Running final code...");
	    	}
	    };
	    		
		WhiteMotor = hardwareMap.dcMotor.get("WhiteMotor");
        GreenMotor = hardwareMap.dcMotor.get("GreenMotor");
        BlueMotor = hardwareMap.dcMotor.get("BlueMotor");
        RedMotor = hardwareMap.dcMotor.get("RedMotor");
        claw = hardwareMap.servo.get("claw");

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
        
        MotorConfigurationType motorType = new MotorConfigurationType();
        motorType.processAnnotation(NeveRest40Gearmotor.class.getDeclaredAnnotation(MotorType.class));
        
        WhiteMotor.setMotorType(motorType);
        GreenMotor.setMotorType(motorType);
        BlueMotor.setMotorType(motorType);
        RedMotor.setMotorType(motorType);
        
        //IMU initialization
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled      = true;
        parametersIMU.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);
        
        //drivetrain initialization
        drive = new OmniWheelDriveSim(new MotorSim[] {BlueMotor,WhiteMotor,RedMotor,GreenMotor}, angle, OpModeType.AUTO);
        drive.updateEncoders();
        drive.inchesPerTick = Math.PI*4/BlueMotor.getMotorType().getTicksPerRev();
        drive.turnError = new Angle(2);
        drive.driveError = 2;
        
        //navigation initialization
	    nav.update(new VectorF(0, 0), new Angle(0, AngleUnit.DEGREES));
        nav.initialize(drive);
        
        waitForStart();
        
    	telemetry.logMotors(false);
        
    	int n = 0;
    	((BNO055IMUSim)imu).lastReturned = -60;
    	boolean updateIMU = true;
    	
        while(opModeIsActive()) {
        	n++;
        	if(nav.i < 4) {
        		if(nav.robotState == RobotStates.STATIONARY_TURN ||
        		((nav.robotState == RobotStates.DRIVING && nav.drivingTurns.get(nav.i-1) != null)
        		&& !(nav.getHeading().getDegree() == nav.prevSetHeading.getDegree()))) {
        		((BNO055IMUSim)imu).turning = true;
        		}
        		else {((BNO055IMUSim)imu).turning = false;}
        	}
    		else {((BNO055IMUSim)imu).turning = false;}
        	if(nav.robotState == RobotStates.STATIONARY_TURN && nav.i == 1 && updateIMU) {
        		updateIMU = false;
        		((BNO055IMUSim)imu).lastReturned = 100;
        	}
        	if(nav.robotState == RobotStates.DRIVING && nav.i == 2 && !updateIMU) {
        		updateIMU = true;
        		((BNO055IMUSim)imu).lastReturned = -50;
        	}
        	Angle heading = new Angle(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES);
        	drive.updateEncoderMotion(heading);
        	nav.updateHeading(heading);
        	Position pose = drive.getEncoderMotion().position;
        	nav.updatePosition(new VectorF((float)pose.x, (float)pose.y));
        	nav.run();
        	drive.updateEncoders();
        	if(n%1 == 0) {
        		telemetry.addData("State", nav.robotState+" #"+nav.i);
            	telemetry.update();
        	}
        }
        System.out.println("Out of While Loop");
	}
}
