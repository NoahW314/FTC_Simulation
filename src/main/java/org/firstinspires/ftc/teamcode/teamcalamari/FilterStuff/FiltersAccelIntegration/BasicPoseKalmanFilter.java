package org.teamcalamari.FilterStuff.FiltersAccelIntegration;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.hardware.bosch.BNO055IMU.AccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;

public abstract class BasicPoseKalmanFilter implements AccelerationIntegrator{
	public int length = 0;
	public ArrayList<ArrayList<Double>> currSensorValues = new ArrayList<>();
	public Long currTimestamp;
	public Long prevTimestamp;
	
	protected int stateNum;
	protected int sensorNum;
	protected int dims;
	
	/**Current State Matrix*/
	protected SimpleMatrix[] X;
	/**Previous State Matrix*/
	protected SimpleMatrix[] pX;
	/**Previous Prediction Error*/
	protected SimpleMatrix[] pP;
	/**Current Prediction Error*/
	protected SimpleMatrix[] P;
	/**Noisy Measurements*/
	protected SimpleMatrix[] Z;
	/**Kalman Gain (variable to store value for use in two places)*/
	protected SimpleMatrix[] G;
	/**State Transition Model*/
	protected SimpleMatrix A = new SimpleMatrix(stateNum, stateNum);
	/**How much each control input contributes to each state*/
	protected SimpleMatrix B = new SimpleMatrix(stateNum, 1);
	/**How much each sensor contributes to each state*/
	protected SimpleMatrix C = new SimpleMatrix(sensorNum, stateNum);
	/**Control input*/
	protected SimpleMatrix U = new SimpleMatrix(1, 1);
	/**Covariance of Process Noise*/
	protected SimpleMatrix Q = new SimpleMatrix(stateNum, stateNum);
	/**Covariance of Sensor Noise (Diagonal contains sensor variance)*/
	protected SimpleMatrix[] R;
	
	protected BasicPoseKalmanFilter(double[][] a, double[][] c, double[][] q, double[] r, double[][] b, int dimensions) {
		dims = dimensions;
		stateNum = a.length;
		sensorNum = r.length;
		
		for(int i = 0; i < sensorNum; i++) {
			currSensorValues.add(new ArrayList<Double>());
		}
		
		X = new SimpleMatrix[dims];
		pX = new SimpleMatrix[dims];
		pP = new SimpleMatrix[dims];
		P = new SimpleMatrix[dims];
		Z = new SimpleMatrix[dims];
		G = new SimpleMatrix[dims];
		R = new SimpleMatrix[dims];
		
		//initialize and set all matrixes
		for(int i = 0; i < dims; i++) {
			X[i] = new SimpleMatrix(stateNum, 1);
			pX[i] = new SimpleMatrix(stateNum, 1);
			pP[i] = new SimpleMatrix(fillDoubleArray(1, stateNum, stateNum));
			P[i] = new SimpleMatrix(stateNum, stateNum);
			Z[i] = new SimpleMatrix(sensorNum, 1);
			G[i] = new SimpleMatrix(stateNum, sensorNum);
			R[i] = SimpleMatrix.diag(r);
			
			for(int j = 0; j < sensorNum; j++) {
				currSensorValues.get(j).add(0d);
			}
		}
		A = new SimpleMatrix(a);
		B = new SimpleMatrix(b);
		C = new SimpleMatrix(c);
		Q = new SimpleMatrix(q);
	}
	
	protected double[][] fillDoubleArray(int value, int rows, int columns) {
		double[][] a = new double[rows][columns];
		for(int i = 0; i < rows; i++) {
			for(int j = 0; j < columns; j++) {
				a[i][j] = value;
			}
		}
		return a;
	}
	
	public DistanceUnit unit;
	
	public Position position;
	public Velocity velocity;
	public Acceleration acceleration;
	public Parameters parameters;
	public Velocity initialVelocity;
	public Position initialPosition;
	
	@Override public Position getPosition() {return position;}
	@Override public Velocity getVelocity() {return velocity;}
	@Override public Acceleration getAcceleration() {return acceleration;}
	
	@Override
	public void initialize(Parameters parameters, Position initialPosition, Velocity initialVelocity) {
		this.initialPosition = initialPosition != null ? initialPosition : new Position();
        this.initialVelocity = initialVelocity != null ? initialVelocity : new Velocity();
		this.parameters = parameters;
		this.position = this.initialPosition;
        this.velocity = this.initialVelocity;
	}
	
	@Override
	public void update(Acceleration linearAcceleration) {}
	
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
