package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeTests.AccelTestsHeading;

import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.DataLoggerSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.FileLoggingAccelerationIntegratorHeadingSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.OldDrives.OmniWheelDriveAtAngleAutoSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.SensorsSim.BNO055IMUSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.LinearOpModeSim;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CompareAccelDataTest extends LinearOpModeSim {

	//the possible states of the program
	private enum states{drive, done};
	//the current state of the program
	private states state = states.drive;
	
	//the angle of the front of the robot relative to the blue wheel
    private int angle = 135;
    //the speed of the robot
    private double speed = 0.5;
    
    //the angle the robot will drive at
    private int driveAngle = 0;
    //the time the robot will drive for
    private double driveTime = 3;
    
    //accelerometer polling interval in milliseconds
    private int interval = 50;

    //the motors, servos, sensor, and servo values
    private MotorSim WhiteMotor;
    private MotorSim GreenMotor;
    private MotorSim BlueMotor;
    private MotorSim RedMotor;
    
    //the imus
    private BNO055IMUSim imu;
    private BNO055IMUSim imu2;
    
    //a timer
    private ElapsedTime time = new ElapsedTime();
    
    //the number test we are doing
    private final long testNum = System.currentTimeMillis();
    
    //the data loggers
    private DataLoggerSim IMU1logger = new DataLoggerSim("Compare Test 1 "+testNum);
    private DataLoggerSim IMU2logger = new DataLoggerSim("Compare Test 2 "+testNum);

	@Override
	public void runOpMode() throws InterruptedException {
		
		//get the motors
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
        parametersIMU.accelerationIntegrationAlgorithm = new FileLoggingAccelerationIntegratorHeadingSim(IMU1logger);
        
        imu = hardwareMap.get(BNO055IMUSim.class, "imu");
        imu.initialize(parametersIMU);
        
        //IMU initialization
        BNO055IMU.Parameters parametersIMU2 = new BNO055IMU.Parameters();
        parametersIMU2.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU2.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU2.loggingEnabled      = true;
        parametersIMU2.loggingTag          = "IMU";
        parametersIMU2.accelerationIntegrationAlgorithm = new FileLoggingAccelerationIntegratorHeadingSim(IMU2logger);
        
        imu2 = hardwareMap.get(BNO055IMUSim.class, "imu2");
        imu2.initialize(parametersIMU2);
        
        //drivetrain initialization
        OmniWheelDriveAtAngleAutoSim robotDrive = new OmniWheelDriveAtAngleAutoSim(new MotorSim[]{BlueMotor, WhiteMotor, RedMotor, GreenMotor}, angle);

        telemetry.addData("Test Num: ", testNum);
        telemetry.update();
        telemetry.logMotors = false;
        
        waitForStart();
        
        //acceleration integrator initializing
        imu.startAccelerationIntegration(null, null, interval);
        imu2.startAccelerationIntegration(null, null, interval);
        
        time.reset();
        
        while(opModeIsActive()) {
        	switch(state) {
        	//drive for driveTime seconds, logging the accelerometer data throughout, then stop
        		case drive:
        			telemetry.addData("State: ", "Driving");
        			//drive
        			robotDrive.driveAtAngle(driveAngle, 1);
        			robotDrive.drive(speed);
        			if(time.seconds() > driveTime) {
        				//stop everything
        				state = states.done;
        				robotDrive.drive(0);
        				imu.stopAccelerationIntegration();
        				imu2.stopAccelerationIntegration();
        			}
        			break;
				default:
					telemetry.addData("State: ", "Finished");
					break;
        	}
        	telemetry.update();
        }
	}
}


