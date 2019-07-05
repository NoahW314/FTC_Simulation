package org.teamcalamari.OpModeTests.KalmanGetData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.teamcalamari.Angle;
import org.teamcalamari.DataLogger;
import org.teamcalamari.HardwareSim.DriveSim.KiwiDrive;
import org.teamcalamari.HardwareSim.MotorsSim.Motor;
import org.teamcalamari.HardwareSim.MotorsSim.WheelMotor;
import org.teamcalamari.HardwareSim.SensorsSim.BNO055IMU;
import org.teamcalamari.HardwareSim.SensorsSim.DistanceSensorTC;
import org.teamcalamari.OpModeSim.LinearOpMode;
import org.teamcalamari.OpModeSim.OpModeType;
import org.teamcalamari.OpModeTests.CompareAuto.Accelerometer.Accelerometer;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class KalmanGetData0 extends LinearOpMode {
	
	private ElapsedTime runtime = new ElapsedTime();
	private String sensorName = "Kalman";
	private final long testNum = System.currentTimeMillis(); 
	private final int stepNum = 0;
	private DataLogger logger = new DataLogger(sensorName+" "+stepNum+" "+testNum);
	
    //imu
	private BNO055IMU imu;

	//the motors and drivetrain
    public WheelMotor RedMotor;
    public WheelMotor BlueMotor;
    public WheelMotor GreenMotor;
    public KiwiDrive drive;
    
    //ultrasonic fields
    private DistanceSensorTC ultraX;
    private DistanceSensorTC ultraY;
    
    private double maxX = 60;
    private double maxY = 36;
    private double ultraXToRobotCenter = 8;
    private double ultraYToRobotCenter = 8;
    
	@Override
	public void runOpMode() throws InterruptedException {
        
        //IMU initialization
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled      = true;
        parametersIMU.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);
        
        //ultrasonic sensor initialization
        ultraX = new DistanceSensorTC(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultraX"));
        ultraY = new DistanceSensorTC(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultraY"));

        //get the motors
        RedMotor = new WheelMotor(hardwareMap, "RedMotor");
        BlueMotor = new WheelMotor(hardwareMap, "BlueMotor");
        GreenMotor = new WheelMotor(hardwareMap, "GreenMotor");

        //set the motors to brake. only required for the hub
        RedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BlueMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GreenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reset the encoders before running
        RedMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BlueMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        GreenMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set the motors to run at a set speed
        RedMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BlueMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GreenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initialize the drivetrain
        drive = new KiwiDrive(new Motor[]{RedMotor, BlueMotor, GreenMotor}, new Angle(0), OpModeType.AUTO);
        drive.updateEncoders();
        drive.inchesPerTick = Math.PI*4/1120;
        drive.turnError = new Angle(2);
        drive.driveError = 1;
        
        //data logger initialization
        logger.addField("Time");
        logger.addField("Heading");
        logger.addField("Ultra X");
        logger.addField("Ultra Y");
        logger.addField("Ultra X Corr");
        logger.addField("Ultra Y Corr");
        logger.addField("Encoder X");
        logger.addField("Encoder Y");
        logger.newLine();
        
        telemetry.addData("TestNum ", testNum);
        telemetry.update();
        
        waitForStart();
        
        runtime.reset();
        
        ultraX.distance = obscureData(52);
        ultraY.distance = obscureData(28);

        while(opModeIsActive() && runtime.seconds() < 10) {
        	//get and store data from sensors
        	Angle heading = new Angle(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES);
        	drive.updateEncoderMotion(heading);
        	double ultraXData = ultraX.getData(DistanceUnit.INCH);
        	double ultraYData = ultraY.getData(DistanceUnit.INCH);
        	double ultraCorrX = correctData(ultraXData);
        	double ultraCorrY = correctData(ultraYData);
        	
        	//put sensor data into position variable
        	Position encPose = drive.getEncoderMotion().position;
        	Position ultraPose = new Position(DistanceUnit.INCH, maxX-ultraXData-ultraXToRobotCenter,  -(maxY-ultraYData-ultraYToRobotCenter), 0, 0);
        	Position ultraCorrPose = new Position(DistanceUnit.INCH, maxX-ultraCorrX-ultraXToRobotCenter, -(maxY-ultraCorrY-ultraYToRobotCenter), 0, 0);

        	logger.addField(runtime.seconds());
        	logger.addField(heading.getDegree());
        	logger.addField(ultraPose.x);
        	logger.addField(ultraPose.y);
        	logger.addField(ultraCorrPose.x);
        	logger.addField(ultraCorrPose.y);
        	logger.addField(encPose.x);
        	logger.addField(encPose.y);
        	logger.newLine();
        }
        logger.closeDataLogger();
        this.requestOpModeStop();
	}

    private double correctData(double data){
        return 0.9828*Math.pow(data, 1.0247);
    }
    
    private double obscureData(double data) {
    	return Math.pow(data/0.9828, 1/1.0247);
    }

}
