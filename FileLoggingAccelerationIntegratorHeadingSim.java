package org.firstinspires.ftc.teamcode.teamcalamari.Simulation;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;

public class FileLoggingAccelerationIntegratorHeadingSim implements BNO055IMU.AccelerationIntegrator {

	/**the parameters with which the IMU was set*/
	BNO055IMU.Parameters parameters;
	/**the current acceleration*/
    Acceleration acceleration;
    /**the variable for logging all the data*/
    public DataLoggerSim logger;
    /**If the integrator has been initialized*/
    public boolean initialized = false;
	
    //the constructor
    public FileLoggingAccelerationIntegratorHeadingSim(DataLoggerSim logger) {
		this.logger = logger;
		logger.addField("Time");
		logger.addField("xAccel");
		logger.addField("yAccel");
		logger.addField("zAccel");
		logger.addField("Heading");
		logger.newLine();
	}

    //getters for the position, velocity, and acceleration
	@Override public Position getPosition() { return new Position(); }
    @Override public Velocity getVelocity() { return new Velocity(); }
    @Override 
    public Acceleration getAcceleration(){
        return this.acceleration == null ? new Acceleration() : this.acceleration;
    }

    //initialize
    @Override
	public void initialize(Parameters parameters, Position initialPosition, Velocity initialVelocity) {
		this.parameters = parameters;
		this.acceleration = null;
		initialized = true;
	}

	//update the acceleration and heading
	public void update(Acceleration linearAcceleration, double heading) {
		// We should always be given a timestamp here
        if (linearAcceleration.acquisitionTime != 0){
            acceleration = linearAcceleration;
            //log the acceleration and heading
            logger.addField(linearAcceleration.acquisitionTime);
            logger.addField(linearAcceleration.xAccel);
            logger.addField(linearAcceleration.yAccel);
            logger.addField(linearAcceleration.zAccel);
            logger.addField(heading);
            logger.newLine();
        }
	}

	@Override
	public void update(Acceleration linearAcceleration) {
	}
}
