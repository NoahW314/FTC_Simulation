package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeTests.CompareAutos;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.AxesSet;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.DataLoggerSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.AutonomousNavigationSim.AllDirectionsNavigationSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.AutonomousNavigationSim.AutonomousNavigationSim.RobotActions;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.OmniWheelDriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.SensorsSim.BNO055IMUSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.SensorsSim.DistanceSensorSimTC;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.LinearOpModeSim;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CompareAutoUltra3 extends LinearOpModeSim {
	
	private String sensorName = "Ultrasonic";
	private final long testNum = System.currentTimeMillis(); 
	private final int stepNum = 3;
	private DataLoggerSim logger = new DataLoggerSim(sensorName+" "+stepNum+" "+testNum);

	//the angle of the front of the robot relative to the blue wheel
    private Angle angle = new Angle(45, AngleUnit.DEGREES);
    //the speed of the robot
    private double speed = 1;
    private double driveSpeed = 0.5;
    
    //no turning in these routines
    private double stillTurnSpeed = 0.75;
    private double stillTurnTurnSpeed = 0.5;
    private double moveTurnSpeed = 0;
	
	//the motors, servos, sensors, servo values, and drivetrain
    private MotorSim WhiteMotor;
    private MotorSim GreenMotor;
    private MotorSim BlueMotor;
    private MotorSim RedMotor;
    
    private BNO055IMUSim imu;
    
    private DistanceSensorSimTC ultraX;
    private DistanceSensorSimTC ultraY;
    
    private double maxX = 24;
    private double maxY = 72;
    private double ultraXToRobotCenter = 9;
    private double ultraYToRobotCenter = 9; 
    private Angle ultraXFieldOffset = new Angle(0, AngleUnit.DEGREES, AxesSet.FIELD_AXES);
    private Angle ultraYFieldOffset = new Angle(90, AngleUnit.DEGREES, AxesSet.FIELD_AXES);
    
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
			return time.seconds() > 5;
		}
    };
    
    ElapsedTime totalTime = new ElapsedTime();
    
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
	    
	    nav.stationaryTurns.set(0, new Angle(-15, AngleUnit.DEGREES));
	    
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
        
        //ultrasonic sensor initialization
        ultraX = new DistanceSensorSimTC(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultraX"));
        ultraY = new DistanceSensorSimTC(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultraY"));
        
        //drivetrain initialization
        drive = new OmniWheelDriveSim(new MotorSim[] {BlueMotor,WhiteMotor,RedMotor,GreenMotor}, angle, OpModeType.AUTO);
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
        telemetry.logHardware = false;
        
        double[] maxUltraX = new double[] {24, 48, 24, 24};
        double[] maxUltraY = new double[] {72, 48, 48, 24};
        
        waitForStart();
        totalTime.reset();
        
        while(opModeIsActive()) {
        	if(nav.robotAction == RobotActions.DRIVING) {
	        	if(nav.i > 0 && nav.i < 4) {
	        		double time = totalTime.seconds();
	        		telemetry.addData("Time", time);
	        		ultraX.distance = (maxUltraX[nav.i]-maxUltraX[nav.i-1])/5*time+maxUltraX[nav.i-1]-ultraXToRobotCenter;
	        		ultraY.distance = (maxUltraY[nav.i]-maxUltraY[nav.i-1])/5*time+maxUltraY[nav.i-1]-ultraYToRobotCenter;
	        	}
	        	else if(nav.i < 4) {
	        		ultraX.distance = maxUltraX[0]-ultraXToRobotCenter;
	        		ultraY.distance = maxUltraY[0]-ultraYToRobotCenter;
	        	}
	        	else {
	        		ultraX.distance = maxUltraX[3]-ultraXToRobotCenter;
	        		ultraY.distance = maxUltraY[3]-ultraYToRobotCenter;
	        	}
        	}
        	else {
        		totalTime.reset();
        	}
        	Angle heading = new Angle(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES);
        	nav.updateHeading(heading);
        	double ultraCorrX = ultraX.getHeadingCorrectedDistance(DistanceUnit.INCH, 0, Angle.add(heading, ultraXFieldOffset));
        	double ultraCorrY = ultraY.getHeadingCorrectedDistance(DistanceUnit.INCH, 1, Angle.add(heading, ultraYFieldOffset));
        	double x = maxX-ultraCorrX-ultraXToRobotCenter;
        	double y = maxY-ultraCorrY-ultraYToRobotCenter;
        	nav.updatePosition(new VectorF((float)x, (float)y));
        	nav.run();
        	logger.addField(nav.getHeading().getDegree());
        	logger.addField(nav.getPosition().get(0));
        	logger.addField(nav.getPosition().get(1));
        	logger.newLine();
        }
        logger.closeDataLogger();
	}

}
