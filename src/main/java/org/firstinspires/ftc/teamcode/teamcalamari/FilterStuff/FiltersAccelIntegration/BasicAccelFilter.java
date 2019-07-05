package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FiltersAccelIntegration;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.hardware.bosch.BNO055IMU.AccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;

public abstract class BasicAccelFilter implements AccelerationIntegrator  {
	public ArrayList<Acceleration> values = new ArrayList<Acceleration>();
	public Acceleration currValue;
	public Position position;
	public Velocity velocity;
	public ArrayList<Acceleration> filtValues = new ArrayList<Acceleration>();
	public Parameters parameters;
	public Acceleration prevAccel;
	public Velocity prevVelocity;
	public Velocity initialVelocity;
	public Position initialPosition;
	
	@Override public Position getPosition() {return position;}
	@Override public Velocity getVelocity() {return velocity;}
	@Override public Acceleration getAcceleration() {return filtValues.get(filtValues.size()-1);}
	
	@Override
	public void initialize(Parameters parameters, Position initialPosition, Velocity initialVelocity) {
		this.initialPosition = initialPosition != null ? initialPosition : new Position();
        this.initialVelocity = initialVelocity != null ? initialVelocity : new Velocity();
		this.parameters = parameters;
		this.position = this.initialPosition;
        this.velocity = this.initialVelocity;
	}
	
	/**Returns the corresponding axis value based on the int.<br>
	0 maps to x, 1 to y, and 2 to z.*/
	public double AccelAxisFromInt(Acceleration accel, int i) {
		switch(i) {
			case 0:	return accel.xAccel;
			case 1: return accel.yAccel;
			case 2: return accel.zAccel;
			default: throw new IllegalArgumentException(i+" is not a valid argument for the method AccelAxisFromInt");
		}
	}
	/**Returns the corresponding axis value based on the int.<br>
	0 maps to x, 1 to y, and 2 to z.*/
	public double VeloAxisFromInt(Velocity velo, int i) {
		switch(i) {
			case 0: return velo.xVeloc;
			case 1: return velo.yVeloc;
			case 2: return velo.zVeloc;
			default: throw new IllegalArgumentException(i+" is not a valid argument for the method VeloAxisFromInt");
		}
	}
	/**Returns the corresponding axis value based on the int.<br>
	0 maps to x, 1 to y, and 2 to z.*/
	public double PoseAxisFromInt(Position pose, int i) {
		switch(i) {
			case 0: return pose.x;
			case 1: return pose.y;
			case 2: return pose.z;
			default: throw new IllegalArgumentException(i+" is not a valid argument for the method PoseAxisFromInt");
		}
	}

	/**Converts a double array to an acceleration object*/
	public Acceleration DoubleArrayToAccel(double[] array, long timestamp) {
		return new Acceleration(DistanceUnit.METER, array[0], array[1], array[2], timestamp);
	}
	/**Converts a double array to an velocity object*/
	public Velocity DoubleArrayToVelocity(double[] array, long timestamp) {
		return new Velocity(DistanceUnit.METER, array[0], array[1], array[2], timestamp);
	}
	/**Converts a double array to an position object*/
	public Position DoubleArrayToPosition(double[] array, long timestamp) {
		return new Position(DistanceUnit.METER, array[0], array[1], array[2], timestamp);
	}
}
