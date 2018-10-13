package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeTests.AccelTestsHeading;

import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.DataLoggerSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.FileLoggingAccelerationIntegratorHeadingSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.SensorsSim.BNO055IMUSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.LinearOpModeSim;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//attempting to get some data from the accelerometer in the form of a .txt file
@TeleOp(name="Zero Step", group="testing")
@Disabled
public class AccelDataTestStill extends LinearOpModeSim {

	//the possible states of the program
	private enum states{record, done};
	//the current state of the program
	private states state = states.record;
	
	//the number of seconds to record the accelerometer data for
	private double recordTime = 5;
	
    //accelerometer polling interval in milliseconds
    private int interval = 50;

    //the imu
    private BNO055IMUSim imu;
    
    //a timer
    private ElapsedTime time = new ElapsedTime();
    
    //the number test we are doing
    private final long testNum = System.currentTimeMillis();
    
    //the data logger
    private DataLoggerSim IMUlogger = new DataLoggerSim("Still Test "+testNum);

	@Override
	public void runOpMode() throws InterruptedException {
        
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
        
        telemetry.addData("Test Num: ", testNum);
        telemetry.update();
        
        waitForStart();
        
        //acceleration integrator initializing
        imu.startAccelerationIntegration(null, null, interval);
        
        time.reset();
        
        while(opModeIsActive()) {
        	switch(state) {
        	//log the accelerometer data for recordTime seconds
        		case record:
        			telemetry.addData("State: ", "Recording");
        			if(time.seconds() > recordTime) {
        				//stop everything
        				state = states.done;
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

