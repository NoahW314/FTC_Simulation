package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Location;
import org.firstinspires.ftc.teamcode.teamcalamari.Math2;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses.Drive;

public abstract class DriveSim {
	/**The array storing the wheel motors*/
	public MotorSim[] wheelMotors;
	/**The last encoder position of each wheel motor*/
	public double[] lastEncoderPositions;
	/**The max speed for any drive motor*/
	public double speed;
	/**The amount turning in place contributes to the power of the motors.
	Values of 0-1 are standard, though the ratio between <code>driveSpeed</code> and <code>turnSpeed</code> is the only thing that matters.
	For driveTo, turnTo, and other similar methods turnSpeed has a slightly different use.
	@see Drive#turnTo*/
	public double turnSpeed;
	/**The amount of error allowed when turning to a set angle.
	a value of 1 allows the actual angle to be within 1 unit of the target angle*/
	public Angle turnError;
	/**Whether or not this is the first time turnTo, driveTo, or turnDriveTo has been called since one of those returned true*/
	protected boolean firstMoveToCall = true;
	/**The maximum possible difference between the targetAngle and the currentAngle in the method turnTo*/
	public final Angle maxDiffAngle = new Angle(180, AngleUnit.DEGREES);
	/**The amount driving (moving not turning in place) contributes to the power of the motors.
	Values of 0-1 are standard, though the ratio between <code>driveSpeed</code> and <code>turnSpeed</code> is the only thing that matters.
	For driveTo, turnTo, and other similar methods driveSpeed has a slightly different use.
	@see Drive#driveTo*/
	public double driveSpeed;
	/**The amount of error allowed when driving to a set position.
	Values are measured in the units that the positions are given in,
	so a value of 1 allows the actual position to be 1 unit away from the target position in the xy plane*/
	public double driveError;
	/**The starting (and therefore maximum) difference between the targetPose and the currentPose in the method driveTo*/
	protected Float maxDiffDist = null;
	/**The location of the robot as given by the encoders*/
	public Location encoderMotion = new Location();
	/**The distance between a wheel and the turning point of the robot*/
	public double turnRadius;
	/**The number of inches per tick of the motor encoder.
	The formula is PI*WheelDiameter/(TicksPerRevolution)*/
	public double inchesPerTick;
	
	
	
	public DriveSim(MotorSim[] motors) {
		wheelMotors = motors;
		lastEncoderPositions = new double[motors.length];
	}
	
	/**turn the robot*/
	public abstract void turn(turnDirection direction);
	
	/**Turn to a set angle. NO OTHER DRIVING METHODS SHOULD BE USED WHEN THIS METHOD IS CALLED.
	You do not need to call <code>drive()</code> when using this method.
	<code>turnSpeed</code> is the fraction of a half rotation (180 degrees) during which the robot is turning at <code>speed</code>.
	<code>turnSpeed</code> should be in the interval [0,1).
	Values very close to 1 should be avoided as the robot may not be able to slow down in time and may overshoot the target.
	@return if the robot has turned to the specified angle*/
	public boolean turnTo(Angle currentAngle, Angle targetAngle) throws IllegalStateException {
		Angle newTargetAngle = Angle.convertAngle(targetAngle, currentAngle);
		if(Math2.round(Angle.abs(Angle.subtract(currentAngle, newTargetAngle)), Angle.mult(turnError, 2)).getDegree() == 0) {
			speed = 0;
			drive();
			return true;
		}
		else {
			double setSpeed = speed;
			speed = (Angle.abs(Angle.subtract(newTargetAngle, currentAngle))).div(maxDiffAngle)*setSpeed/(1-turnSpeed);
			if(speed > setSpeed) speed = setSpeed;
			
			turn((newTargetAngle.greaterThan(currentAngle)) ? turnDirection.CCW : turnDirection.CW);
			drive();
			
			speed = setSpeed;
			return false;
		}
	}
	
	/**Drive to a set position. NO OTHER DRIVING METHODS SHOULD BE USED WHEN THIS METHOD IS CALLED.
	You do not need to call <code>drive()</code> when using this method.
	<code>driveSpeed</code> is the fraction of the distance during which the robot is moving at <code>speed</code>.
	<code>driveSpeed</code> should be in the interval [0,1).
	Values very close to 1 should be avoided as the robot may not be able to slow down in time and may overshoot the target.
	@return if the robot has driven to the specificed point*/
	public abstract boolean driveTo(VectorF currentPose, VectorF targetPose, Angle heading);
	
	/**set the calculated powers to the wheel motors*/
	public void drive() {}
	
	public void updateEncoders() {
		for(int i = 0; i < lastEncoderPositions.length; i++) {
			lastEncoderPositions[i] = wheelMotors[i].getCurrentPosition();
		}
	}
	public abstract void updateEncoderMotion();
	
	public Location getEncoderMotion() {
		return encoderMotion;
	}
	public VectorF getEncoderDistance() {
		Position pose = getEncoderMotion().position;
		return new VectorF((float)pose.x, (float)pose.y);
	}
	public Angle getEncoderHeading() {
		Orientation o = getEncoderMotion().orientation;
		return new Angle(o.thirdAngle, o.angleUnit);
	}
	
	public enum turnDirection{CW, CCW}
}
