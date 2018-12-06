package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeTests.AccelTestsHeading.NavTests;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.DataLoggerSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.FileLoggingAccelerationIntegratorHeadingSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.AutonomousNavigationSim.AllDirectionsNavigationSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.DriveSim.KiwiDriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.WheelMotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.SensorsSim.BNO055IMUSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeSim.LinearOpModeSim;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AccelDataNavTestTwo extends LinearOpModeSim {

	private final long testNum = System.currentTimeMillis();
	private DataLoggerSim IMUlogger = new DataLoggerSim("2 Step Nav Test "+testNum);

    //the speed of the robot
    private double speed = 0.5;
    private double driveSpeed = 1;

    //no turning in these routines
    private double stillTurnSpeed = 0;
    private double stillTurnTurnSpeed = 0;
    private double moveTurnSpeed = 0;
	
	//the motors, servos, sensors, servo values, and drivetrain
    public WheelMotorSim RedMotor;
    public WheelMotorSim YellowMotor;
    public WheelMotorSim GreenMotor;
    public KiwiDriveSim drive;
        
    private BNO055IMUSim imu;
    
    //navigation variables
    private int targetNum = 3;
    private AllDirectionsNavigationSim nav = new AllDirectionsNavigationSim(targetNum, speed, driveSpeed, stillTurnSpeed, stillTurnTurnSpeed, moveTurnSpeed, this);
    
	@Override
	public void runOpMode() throws InterruptedException {
		nav.targets.set(0, new VectorF(0, 0));
	    nav.targets.set(1, new VectorF(48, 0));
	    nav.targets.set(2, new VectorF(48, 48));
	    
	    nav.finalCode = new Runnable() {
	    	public void run() {
	    		IMUlogger.closeDataLogger();
	    	}
	    };

        //get the motors
        RedMotor = new WheelMotorSim(hardwareMap, "RedMotor");
        YellowMotor = new WheelMotorSim(hardwareMap, "YellowMotor");
        GreenMotor = new WheelMotorSim(hardwareMap, "GreenMotor");

        //set the motors to brake. only required for the hub
        RedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        YellowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GreenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set the motors to run at a set speed
        RedMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        YellowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GreenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initialize the drivetrain
        drive = new KiwiDriveSim(new MotorSim[]{RedMotor, GreenMotor, YellowMotor}, new Angle(0), OpModeType.AUTO);
        drive.updateEncoders();
        drive.inchesPerTick = Math.PI*4/1120;
        drive.turnError = new Angle(2);
        drive.driveError = 1;
        
        //IMU initialization
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled      = true;
        parametersIMU.loggingTag          = "IMU";
        parametersIMU.accelerationIntegrationAlgorithm = new FileLoggingAccelerationIntegratorHeadingSim(IMUlogger);

        imu = hardwareMap.get(BNO055IMUSim.class, "imu");
        imu.initialize(parametersIMU);
        
        //navigation initialization
	    nav.update(new VectorF(0, 0), new Angle(0, AngleUnit.DEGREES));
        nav.initialize(drive);
        
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
        	
        	telemetry.addData("X", nav.getPosition().get(0));
        	telemetry.addData("Y", nav.getPosition().get(1));
        }
	}

}
