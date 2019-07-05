package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors;

import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.teamcalamari.FileLoggingAccelerationIntegratorHeading;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareDevice;

import com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;

public class BNO055IMU implements com.qualcomm.hardware.bosch.BNO055IMU, HardwareDevice{
	public double accelX = 0;
	public double accelY = 0;
	public double accelZ = 0;
	public double veloX = 0;
	public double veloY = 0;
	public double veloZ = 0;
	public boolean setXVelo = false;
	public boolean setYVelo = false;
	public boolean setZVelo = false;
	Parameters parameters;
	public int lastReturned = 0;
	public boolean turning = false;
	protected AccelerationIntegrator accelerationAlgorithm;
	protected final Object startStopLock = new Object();
    protected final Object dataLock = new Object();
    protected ExecutorService accelerationMananger;
	

	public boolean initialize(Parameters parameters) {
		this.parameters = parameters.clone();
		this.accelerationAlgorithm = parameters.accelerationIntegrationAlgorithm;
		return true;
	}

	@Override
	public Parameters getParameters() {
		return parameters;
	}

	@Override
	public void close() {
		stopAccelerationIntegration();
	}

	@Override
	public Orientation getAngularOrientation() {
		Orientation orientation = new Orientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, 
				org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES, 
				lastReturned, lastReturned, lastReturned, 0L);
		if(turning) {lastReturned++;}
		return orientation;
	}
	
	@Override
	public Orientation getAngularOrientation(AxesReference reference, AxesOrder order,
			org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit) {
		Orientation orientation = new Orientation(reference, order, angleUnit, 
				lastReturned, lastReturned, lastReturned, 0L);
		if(turning) {lastReturned++;}
		return orientation;
	}

	@Override
	public Acceleration getOverallAcceleration() {
		return new Acceleration();
	}

	@Override
	public AngularVelocity getAngularVelocity() {
		return new AngularVelocity();
	}

	@Override
	public Acceleration getLinearAcceleration() {
		return new Acceleration(DistanceUnit.MM, accelX,accelY,accelZ, System.nanoTime());
	}

	@Override
	public Acceleration getGravity() {
		return new Acceleration();
	}

	@Override
	public Temperature getTemperature() {
		return new Temperature();
	}

	@Override
	public MagneticFlux getMagneticFieldStrength() {
		return new MagneticFlux();
	}

	@Override
	public Quaternion getQuaternionOrientation() {
		return new Quaternion();
	}

	@Override
	public Position getPosition() {
		Position pose = this.accelerationAlgorithm.getPosition();
		return pose == null ? new Position() : pose;
	}

	@Override
	public Velocity getVelocity() {
		Velocity velo = this.accelerationAlgorithm.getVelocity();
		return velo == null ? new Velocity() : velo;
	}

	@Override
	public Acceleration getAcceleration() {
		return new Acceleration();
	}

	@Override
	public void startAccelerationIntegration(Position initalPosition, Velocity initialVelocity, int msPollInterval)
    // Start integrating acceleration to determine position and velocity by polling for acceleration every while
        {
        synchronized (this.startStopLock)
            {
            // Stop doing this if we're already in flight
            this.stopAccelerationIntegration();

            // Set the current position and velocity
            this.accelerationAlgorithm.initialize(this.parameters, initalPosition, initialVelocity);

            // Make a new thread on which to do the integration
            /*this.accelerationMananger = ThreadPool.newSingleThreadExecutor("imu acceleration");

            // Start the whole schebang a rockin...
            this.accelerationMananger.execute(new AccelerationManagerFileLog(msPollInterval));*/
            new Thread(new AccelerationManagerFileLog(msPollInterval)).start();
            }
        }
    @Override
    public void stopAccelerationIntegration() // needs a different lock than 'synchronized(this)'
        {
        synchronized (this.startStopLock)
            {
        	try {
	        	if(((FileLoggingAccelerationIntegratorHeading)(this.accelerationAlgorithm)).initialized) {
	        		((FileLoggingAccelerationIntegratorHeading)(this.accelerationAlgorithm)).logger.closeDataLogger();
	        	}
        	}
        	catch(ClassCastException cce) {
        		
        	}
            // Stop the integration thread
            if (this.accelerationMananger != null)
                {
                this.accelerationMananger.shutdownNow();
                ThreadPool.awaitTerminationOrExitApplication(this.accelerationMananger, 10, TimeUnit.SECONDS, "IMU acceleration", "unresponsive user acceleration code");
                this.accelerationMananger = null;
                }
            }
        }
    
    class AccelerationManagerFileLog implements Runnable{
    	protected final int msPollInterval;
        protected final static long nsPerMs = ElapsedTime.MILLIS_IN_NANO;
        
        AccelerationManagerFileLog(int msPollInterval)
            {
            this.msPollInterval = msPollInterval;
            }
        
        @Override public void run()
            {
            // Don't let inappropriate exceptions sneak out
            try
                {
                // Loop until we're asked to stop
                while (!isStopRequested())
                    {
                    // Read the latest available acceleration
                    final Acceleration linearAcceleration = BNO055IMU.this.getLinearAcceleration();
                    final double heading = BNO055IMU.this.getAngularOrientation(
                    							AxesReference.EXTRINSIC, 
                    							AxesOrder.XYZ, 
                    							org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES)
                    							.thirdAngle;

                    // Have the algorithm do its thing
                    synchronized (dataLock)
                        {
                    	try {
                    		((FileLoggingAccelerationIntegratorHeading)accelerationAlgorithm).update(linearAcceleration, heading);
                    	}
                    	catch(ClassCastException cce) {
                    		/*try{
                    			double setVeloX = setXVelo ? veloX : accelerationAlgorithm.getVelocity().xVeloc;
                    			double setVeloY = setYVelo ? veloY : accelerationAlgorithm.getVelocity().yVeloc;
                    			double setVeloZ = setZVelo ? veloZ : accelerationAlgorithm.getVelocity().zVeloc;
                    			((NoFilter)accelerationAlgorithm).setVelocity(
                    					new Velocity(DistanceUnit.MM, 
                    							setVeloX, setVeloY, setVeloZ, System.nanoTime()));
                    		}
                    		catch(ClassCastException ce) {
                    			
                    		}*/
                    		accelerationAlgorithm.update(linearAcceleration);
                    	}
                        }
                    
                    // Wait an appropriate interval before beginning again
                    if (msPollInterval > 0)
                        {
                        long msSoFar = (System.nanoTime() - linearAcceleration.acquisitionTime) / nsPerMs;
                        long msReadFudge = 5;   // very roughly accounts for delta from read request to acquisitionTime setting
                        Thread.sleep(Math.max(0,msPollInterval - msSoFar - msReadFudge));
                        }
                    else
                        Thread.yield(); // never do a hard spin
                    }
                }
            catch (InterruptedException|CancellationException e)
                {
                return;
                }
            }
    }
    
    protected boolean isStopRequested()
    {
    return Thread.currentThread().isInterrupted();
    }

	@Override
	public SystemStatus getSystemStatus() {
		return SystemStatus.UNKNOWN;
	}

	@Override
	public SystemError getSystemError() {
		return SystemError.NO_ERROR;
	}

	@Override
	public CalibrationStatus getCalibrationStatus() {
		return new CalibrationStatus(0);
	}

	@Override
	public boolean isSystemCalibrated() {
		return true;
	}

	@Override
	public boolean isGyroCalibrated() {
		return true;
	}

	@Override
	public boolean isAccelerometerCalibrated() {
		return true;
	}

	@Override
	public boolean isMagnetometerCalibrated() {
		return true;
	}

	@Override
	public CalibrationData readCalibrationData() {
		return new CalibrationData();
	}

	@Override
	public void writeCalibrationData(CalibrationData data) {
	}

	@Override
	public byte read8(Register register) {
		return 0;
	}

	@Override
	public byte[] read(Register register, int cb) {
		return new byte[cb];
	}

	@Override
	public void write8(Register register, int bVal) {
	}

	@Override
	public void write(Register register, byte[] data) {
	}
	
	//Hardware Device 

	@Override
	public Manufacturer getManufacturer() {
		return Manufacturer.Other;
	}

	@Override
	public String getDeviceName() {
		return "BNO055IMU ulation";
	}

	@Override
	public String getConnectionInfo() {
		return "Connection info does not exist for a ulated sensor";
	}

	@Override
	public int getVersion() {
		return 1;
	}

	@Override
	public void resetDeviceConfigurationForOpMode() {
	}

	@Override
	public void move() {
	}

	@Override
	public String log(String deviceName) {
		return "";
	}

}
