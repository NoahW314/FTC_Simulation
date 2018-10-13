package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeTests.AccelTestsHeading;

import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.DataLoggerSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.FileLoggingAccelerationIntegratorHeadingSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.OldDrives.OmniWheelDriveAtAngleAutoSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.SensorsSim.BNO055IMUSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.LinearOpModeSim;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//attempting to get some data from the accelerometer in the form of a .txt file
@TeleOp(name="Two Step", group="testing")
@Disabled
public class AccelDataTestTwo extends LinearOpModeSim {

	//the possible states of the program
	private enum states{drive1, drive2, done};
	//the current state of the program
	private states state = states.drive1;
	
	//the angle of the front of the robot relative to the blue wheel
    private int angle = 135;
    //the speed of the robot
    private double speed = 0.5;
    
    //the angle the robot will drive at in the first step
    private int driveAngle1 = 0;
    //the time the robot will drive for in the first step
    private double driveTime1 = 3;
    //the angle the robot will drive at in the second step
    private int driveAngle2 = 90;
    //the time the robot will drive for in the second step
    private double driveTime2 = 2;
    
    //accelerometer polling interval in milliseconds
    private int interval = 50;
    

    //the motors, servos, sensor, and servo values
    private MotorSim WhiteMotor;
    private MotorSim GreenMotor;
    private MotorSim BlueMotor;
    private MotorSim RedMotor;
    
    //the imu
    private BNO055IMUSim imu;
    
    //a timer
    private ElapsedTime time = new ElapsedTime();
    
    //the number test we are doing
    private final long testNum = System.currentTimeMillis();
    
    //the data logger
    private DataLoggerSim IMUlogger = new DataLoggerSim("Two Step Test "+testNum);

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
        parametersIMU.accelerationIntegrationAlgorithm = new FileLoggingAccelerationIntegratorHeadingSim(IMUlogger);
        
        imu = hardwareMap.get(BNO055IMUSim.class, "imu");
        imu.initialize(parametersIMU);
        
        //drivetrain initialization
        OmniWheelDriveAtAngleAutoSim robotDrive = new OmniWheelDriveAtAngleAutoSim(new MotorSim[]{BlueMotor, WhiteMotor, RedMotor, GreenMotor}, angle);

        telemetry.addData("Test Num: ", testNum);
        telemetry.update();
        
        waitForStart();
        
        //acceleration integrator initializing
        imu.startAccelerationIntegration(null, null, 100);
        
        time.reset();
        
        while(opModeIsActive()) {
        	switch(state) {
        	//drive for driveTime1 seconds, then change direction and drive for driveTime2 seconds,
        	//logging the accelerometer data throughout, then stop
        		case drive1:
        			telemetry.addData("State: ", "Driving 1");
        			//drive
        			robotDrive.driveAtAngle(driveAngle1, 1);
        			robotDrive.drive(speed);
        			if(time.seconds() > driveTime1) {
        				//move to the next step
        				state = states.drive2;
        				time.reset();
        			}
        			break;
        		case drive2:
        			telemetry.addData("State: ", "Driving 2");
        			//drive
        			robotDrive.driveAtAngle(driveAngle2, 1);
        			robotDrive.drive(speed);
        			if(time.seconds() > driveTime2) {
        				//stop everything
        				state = states.done;
        				robotDrive.drive(0);
        				imu.stopAccelerationIntegration();
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