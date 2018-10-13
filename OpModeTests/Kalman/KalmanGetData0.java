package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeTests.Kalman;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.DataLoggerSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.VuforiaSimTC;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.OmniWheelDriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.SensorsSim.BNO055IMUSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.SensorsSim.DistanceSensorSimTC;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.LinearOpModeSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Tests.CompareAutos.Accelerometer.Accelerometer;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class KalmanGetData0 extends LinearOpModeSim {
	
	private ElapsedTime runtime = new ElapsedTime();
	private String sensorName = "Kalman";
	private final long testNum = System.currentTimeMillis(); 
	private final int stepNum = 0;
	private DataLoggerSim logger = new DataLoggerSim(sensorName+" "+stepNum+" "+testNum);
	
	//accelerometer polling interval in milliseconds
    private int interval = 50;
	
    //imu
	private BNO055IMUSim imu;
	
	//the motors and drivetrain
    private MotorSim WhiteMotor;
    private MotorSim GreenMotor;
    private MotorSim BlueMotor;
    private MotorSim RedMotor;
    private OmniWheelDriveSim drive;
    
    //vuforia
    private VuforiaSimTC vuforia;
    //conversion constant for inches and millimeters
    private float inToMM = 25.4f;
    
    //ultrasonic fields
    private DistanceSensorSimTC ultraX;
    private DistanceSensorSimTC ultraY;
    
    private double maxX = 24;
    private double maxY = 72;
    private double ultraXToRobotCenter = 9;
    private double ultraYToRobotCenter = 9;
    private Angle ultraXFieldOffset = new Angle(0, AngleUnit.DEGREES);
    private Angle ultraYFieldOffset = new Angle(90, AngleUnit.DEGREES);
    
	@Override
	public void runOpMode() throws InterruptedException {
		//vuforia initialization
        VuforiaSimTC.Parameters parametersVuforia = new VuforiaSimTC.Parameters(hardwareMap);
        parametersVuforia.useExtendedTracking = false;
		parametersVuforia.cameraDirection = CameraDirection.BACK;
		
		try {
			vuforia = new VuforiaSimTC(parametersVuforia, "RelicVuMark");
		} catch (Exception e) {
			e.printStackTrace();
		}
		/*vuforia.setTargetLocation(OpenGLMatrix.translation(0f, 72f*inToMM, 5.25f*inToMM)
				.multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,
						90, 0, 0)));
		vuforia.setPhoneLocation(OpenGLMatrix.translation(0, -7*inToMM, 4*inToMM)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,
                        -90, -90, 0)));*/
		vuforia.activate();
        
        //IMU initialization
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled      = true;
        parametersIMU.loggingTag          = "IMU";
        parametersIMU.accelerationIntegrationAlgorithm = Accelerometer.newIntegratorAlgorithm();

        imu = hardwareMap.get(BNO055IMUSim.class, "imu");
        imu.initialize(parametersIMU);
        
        //ultrasonic sensor initialization
        ultraX = new DistanceSensorSimTC(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultraX"));
        ultraY = new DistanceSensorSimTC(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultraY"));
        
        //motor initialization
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
        
        //drivetrain initialization
        drive = new OmniWheelDriveSim(new MotorSim[] {BlueMotor,WhiteMotor,RedMotor,GreenMotor}, null, OpModeType.AUTO);
        drive.updateEncoders();
        drive.inchesPerTick = Math.PI*4/1120;
        
        //data logger initialization
        logger.addField("Heading");
        logger.addField("Vuforia X");
        logger.addField("Vuforia Y");
        logger.addField("Ultra X");
        logger.addField("Ultra Y");
        logger.addField("Accel X");
        logger.addField("Accel Y");
        logger.addField("Encoder X");
        logger.addField("Encoder Y");
        logger.newLine();
        
        telemetry.addData("TestNum ", testNum);
        telemetry.update();
        
        //data faking
        vuforia.trackables.get(0).robotPose = new Position();
        imu.accelX = 0;
        imu.accelY = 0;
        ultraX.distance = 15;
        ultraY.distance = 63;
        
        waitForStart();
        
        runtime.reset();
        imu.startAccelerationIntegration(new Position(), new Velocity(), interval);
        
        while(opModeIsActive() && runtime.seconds() < 10) {
        	//get and store data from sensors
        	Angle heading = new Angle(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES);
        	drive.updateEncoderMotion(heading);
        	double ultraCorrX = ultraX.getHeadingCorrectedDistance(DistanceUnit.INCH, 0, Angle.add(heading, ultraXFieldOffset));
        	double ultraCorrY = ultraY.getHeadingCorrectedDistance(DistanceUnit.INCH, 1, Angle.add(heading, ultraYFieldOffset));
        	
        	//put sensor data into position variable
        	Position encPose = drive.getEncoderMotion().position;
        	Position accelPose = imu.getPosition().toUnit(DistanceUnit.INCH);
        	Position ultraPose = new Position(DistanceUnit.INCH, maxX-ultraCorrX-ultraXToRobotCenter, maxY-ultraCorrY-ultraYToRobotCenter, 0, 0);
        	Position vufPose = vuforia.getPositionOnField();
        	
        	drive.updateEncoders();
        	logger.addField(heading.getDegree());
        	logger.addField(vufPose.x);
        	logger.addField(vufPose.y);
        	logger.addField(ultraPose.x);
        	logger.addField(ultraPose.y);
        	logger.addField(accelPose.x);
        	logger.addField(accelPose.y);
        	logger.addField(encPose.x);
        	logger.addField(encPose.y);
        	logger.newLine();
        }
        imu.stopAccelerationIntegration();
        logger.closeDataLogger();
        this.requestOpModeStop();
	}
}