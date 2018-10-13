package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeTests.CompareAutos;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.DataLoggerSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.AutonomousNavigationSim.AllDirectionsNavigationSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.OmniWheelDriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.SensorsSim.BNO055IMUSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.LinearOpModeSim;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CompareAutoEncoder3 extends LinearOpModeSim {
	
	private String sensorName = "Encoder";
	private final long testNum = System.currentTimeMillis(); 
	private final int stepNum = 3;
	private DataLoggerSim logger = new DataLoggerSim(sensorName+" "+stepNum+" "+testNum);

	//the angle of the front of the robot relative to the blue wheel
    private Angle angle = new Angle(45, AngleUnit.DEGREES);
    //the speed of the robot
    private double speed = 1;
    private double driveSpeed = 0.5;
    
    //no turning in these routines
    private double stillTurnSpeed = 0;
    private double stillTurnTurnSpeed = 0;
    private double moveTurnSpeed = 0;
	
	//the motors, servos, sensors, servo values, and drivetrain
    private MotorSim WhiteMotor;
    private MotorSim GreenMotor;
    private MotorSim BlueMotor;
    private MotorSim RedMotor;
    
    private BNO055IMUSim imu;
    
    private OmniWheelDriveSim drive;
    
    //navigation variables
    private int targetNum = 4;
    private AllDirectionsNavigationSim nav = new AllDirectionsNavigationSim(targetNum, speed, driveSpeed, stillTurnSpeed, stillTurnTurnSpeed, moveTurnSpeed, this);
    
    RobotAction wait = new RobotAction() {
    	ElapsedTime time = new ElapsedTime();
    	public boolean start() {
    		//for safety
    		drive.speed = 0;
    		drive.drive();
    		
    		time.reset();
    		return true;
    	}
		@Override
		public boolean act() {
			return time.seconds() > 10;
		}
    };
    
	@Override
	public void runOpMode() throws InterruptedException {
		nav.targets.set(0, new VectorF(0, 0));
	    nav.targets.set(1, new VectorF(-24, 24));
	    nav.targets.set(2, new VectorF(0, 24));
	    nav.targets.set(3, new VectorF(0, 48));
	    
	    nav.finalCode = new Runnable() {
	    	public void run() {
	    		logger.closeDataLogger();
	    	}
	    };
	    
	    nav.actions.set(1, wait);
	    nav.actions.set(2, wait);
	    
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
        
        //IMU initialization
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled      = true;
        parametersIMU.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMUSim.class, "imu");
        imu.initialize(parametersIMU);
        
        //drivetrain initialization
        drive = new OmniWheelDriveSim(new MotorSim[] {BlueMotor,WhiteMotor,RedMotor,GreenMotor}, angle, OpModeType.AUTO);
        drive.updateEncoders();
        drive.inchesPerTick = Math.PI*4/1120;
        drive.turnError = new Angle(2);
        drive.driveError = 1;
        
        //navigation initialization
	    nav.update(new VectorF(0, 0), new Angle(0, AngleUnit.DEGREES));
        nav.initialize(drive);
        
        //data logger initialization
        logger.addField("Heading");
        logger.addField("Position X");
        logger.addField("Position Y");
        logger.newLine();
        
        telemetry.addData("TestNum ", testNum);
        telemetry.update();
        
        waitForStart();
        
        while(opModeIsActive()) {
        	Angle heading = new Angle(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES);
        	drive.updateEncoderMotion(heading);
        	nav.updateHeading(heading);
        	Position pose = drive.getEncoderMotion().position;
        	nav.updatePosition(new VectorF((float)pose.x, (float)pose.y));
        	nav.run();
        	drive.updateEncoders();
        	
        	logger.addField(nav.getHeading().getDegree());
        	logger.addField(nav.getPosition().get(0));
        	logger.addField(nav.getPosition().get(1));
        	logger.newLine();
        }
	}

}
