package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcalamari.Math2;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.DistanceMeasure;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Location;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareMap;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.Motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

public abstract class Drive {
	/**The array storing the wheel motors*/
	public Motor[] wheelMotors;
	/**The last encoder position of each wheel motor*/
	public int[] lastEncoderPositions;
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
	/**DEPRECATED - Use getDriveError() and setDriveError() or allowedDriveError instead<br><br>
    The amount of error allowed when driving to a set position.
	Values are measured in the units that the positions are given in,
	so a value of 1 allows the actual position to be 1 unit away from the target position in the xy plane*/
	@Deprecated
	public double driveError;
    /**The amount of error allowed when driving to a set position.
    A value of 1 would allow the actual position to be 1 unit away from the target position in the xy plane*/
	protected DistanceMeasure allowedDriveError;
	/**The starting (and therefore maximum) difference between the targetPose and the currentPose in the method driveTo*/
	protected DistanceMeasure maxDiffDist = null;
	/**The location of the robot as given by the encoders*/
	protected Location encoderMotion = new Location();
	/**The distance between a wheel and the turning point of the robot*/
	public DistanceMeasure turnRadius;
	/**The number of inches per tick of the motor encoder.
	The formula is PI*WheelDiameter/(TicksPerRevolution)*/
	public double inchesPerTick;
	
	
	
	public Drive(Motor[] motors) {
		wheelMotors = motors;
		lastEncoderPositions = new int[motors.length];
	}
	public Drive(DcMotor[] motors) {
		this(Motor.DcMotorsToMotors(motors));
	}
	public Drive(HardwareMap map, String... motorNames){
	    wheelMotors = new Motor[motorNames.length];
	    for(int i = 0; i < motorNames.length; i++){
	        wheelMotors[i] = new Motor(motorNames[i], map, true);
        }
		lastEncoderPositions = new int[wheelMotors.length];
	}

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
	    for(Motor motor : wheelMotors){
	        motor.setZeroPowerBehavior(behavior);
        }
    }

    public void setMode(DcMotor.RunMode mode){
        for(Motor motor : wheelMotors){
            motor.setMode(mode);
        }
    }

    public void setDriveError(DistanceMeasure m) { allowedDriveError = m.copy();/*paranoia*/ }
    public void setDriveError(double value, DistanceUnit unit){ allowedDriveError = new DistanceMeasure(value, unit);}
    public DistanceMeasure getDriveError(){ return allowedDriveError.copy();/*paranoia*/ }

    public void setInchesPerTick(Class<?> clazz, double wheelDiameter){
	    if(clazz.isAnnotationPresent(MotorType.class)){
	        MotorType motorType = clazz.getAnnotation(MotorType.class);
	        inchesPerTick = Math.PI*wheelDiameter/motorType.ticksPerRev();
        }
        else throw new IllegalArgumentException("Class "+clazz.getSimpleName()+" doesn't contain the motor type annotation!");
    }

    /**turn the robot*/
	public abstract void turn(turnDirection direction);
	
	/**Turn to a set angle. NO OTHER DRIVING METHODS SHOULD BE USED WHEN THIS METHOD IS CALLED.
	You do not need to call <code>drive()</code> when using this method.
	<code>turnSpeed</code> is the fraction of a half rotation (180 degrees) during which the robot is turning at <code>speed</code>.
	<code>turnSpeed</code> should be in the interval [0,1).
	Values very close to 1 should be avoided as the robot may not be able to slow down in time and may overshoot the target.
	@return if the robot has turned to the specified angle*/
	public boolean turnTo(Angle currentAngle, Angle targetAngle) {
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
    /**Turn to a set angle. NO OTHER DRIVING METHODS SHOULD BE USED WHEN THIS METHOD IS CALLED.
     You do not need to call {@code drive()} when using this method.
     {@code turnSpeed} is used to control how fast the robot slows down after reaching {@code distanceTillP}. Larger values
     cause the robot to slow down faster. A value of 0 will stop the robot immediately.  A value of 1 will cause the robot
     to slow down like if {@code continuous} were true. A value over 1 will keep the robot running at full speed for longer
     before slowing down.
     @param distanceTillP the robot will turn at <code>speed</code> until it is less than <code>distanceTillP</code>
     units away from the target, then it will turn at a speed proportional to the distance away from the target it is, as
     indicated by the {@code continuous} parameter. Should be a positive angle.<br><br>
     @param continuous If true then <code>turnSpeed</code> has no effect and the speed will start at full speed then
     decrease linearly with position.  If false, then the robot will slow down suddenly and then its speed will decrease linearly
     with position based on {@code driveSpeed}.  Only takes effect after the robot is closer than {@code distanceTillP} units away
     from the target.
     @return if the robot has turned to the specified angle*/
    public boolean turnTo(Angle currentAngle, Angle targetAngle, Angle distanceTillP, boolean continuous){
        Angle newTargetAngle = Angle.convertAngle(targetAngle, currentAngle);
        Angle error = Angle.abs(Angle.subtract(currentAngle, newTargetAngle));
        double setSpeed = speed;
        if(Math2.round(error, Angle.mult(turnError, 2)).getDegree() == 0){
            speed = 0;
            drive();
            return true;
        }
        /*Drive at a proportional speed if the error is less than distanceTillP*/
        else if(error.lessThan(distanceTillP)){
            if(continuous){
                speed = error.div(distanceTillP)*setSpeed;
            }
            else{
                speed = error.div(distanceTillP)*setSpeed*turnSpeed;
            }
            if(speed > setSpeed) speed = setSpeed; //paranoia
        }

        turn((newTargetAngle.greaterThan(currentAngle)) ? turnDirection.CCW : turnDirection.CW);
        drive();

        speed = setSpeed;
        return false;
    }

	
	/**Drive to a set position. NO OTHER DRIVING METHODS SHOULD BE USED WHEN THIS METHOD IS CALLED.
	You do not need to call <code>drive()</code> when using this method.
	<code>driveSpeed</code> is the fraction of the distance during which the robot is moving at <code>speed</code>.
	<code>driveSpeed</code> should be in the interval [0,1).
	Values very close to 1 should be avoided as the robot may not be able to slow down in time and may overshoot the target.
	@return if the robot has driven to the specified point*/
	@Deprecated
	public abstract boolean driveTo(VectorF currentPose, VectorF targetPose, Angle heading);

    /**Drive to a set position. NO OTHER DRIVING METHODS SHOULD BE USED WHEN THIS METHOD IS CALLED.
     You do not need to call <code>drive()</code> when using this method.
     <code>driveSpeed</code> is the fraction of the distance during which the robot is moving at <code>speed</code>.
     <code>driveSpeed</code> should be in the interval [0,1).
     Values very close to 1 should be avoided as the robot may not be able to slow down in time and may overshoot the target.
     @return if the robot has driven to the specified point*/
	public abstract boolean driveTo(Position currentPose, Position targetPose, Angle heading);

	/**Drive to a set position. NO OTHER DRIVING METHODS SHOULD BE USED WHEN THIS METHOD IS CALLED.
     You do not need to call <code>drive()</code> when using this method.  The robot will drive at full speed until it is
     under <code>distanceTillP</code> units away from the target, then the robot will drive at a speed proportional to
     the distance away from the target it is, like in the other driveTo method. {@code driveSpeed} is used to control how
     fast the robot slows down after reaching {@code distanceTillP}. Larger values cause the robot to slow down faster.
     A value of 0 will stop the robot immediately.  A value of 1 will cause the robot to slow down like if {@code continuous}
     were true. A value over 1 will keep the robot running at full speed for longer before slowing down.  If {@code continuous}
     is true then <code>driveSpeed</code> has no effect and the speed will start at full speed then decrease linearly with
     position.  If false, then the robot will slow down suddenly and then its speed will decrease linearly with position
     based on <code>driveSpeed</code>.  Only takes effect after the robot is closer than {@code distanceTillP}
     units away from the target.
	 @return if the robot has driven to the specified point*/
	public abstract boolean driveTo(Position currentPose, Position targetPose, Angle heading, DistanceMeasure distanceTillP, boolean continuous);

	
	/**set the calculated powers to the wheel motors*/
	public void drive() {
        //set the wheel motor powers
        for(Motor wheelMotor : wheelMotors) {
            wheelMotor.run();
        }
    }

	/**Stop the drivetrain*/
	public void stop(){
	    this.speed = 0;
	    for(int i = 0; i < wheelMotors.length; i++) {
            this.wheelMotors[i].resetPower(0);//paranoia
        }
        this.drive();
    }
	
	public void updateEncoders() {
		for(int i = 0; i < lastEncoderPositions.length; i++) {
			lastEncoderPositions[i] = wheelMotors[i].getCurrentPosition();
		}
	}
	public abstract void updateEncoderMotion();
	public abstract void updateEncoderMotion(Angle heading);
	
	public Location getEncoderMotion() {
		return encoderMotion;
	}
    /**DEPRECATED - use getEncoderPosition()*/
    @Deprecated
	public VectorF getEncoderDistance() {
		Position pose = getEncoderMotion().getPosition();
		return new VectorF((float)pose.x, (float)pose.y);
	}
	public Position getEncoderPosition(){
	    return getEncoderMotion().getPosition();
    }
	public Angle getEncoderHeading() {
		Orientation o = getEncoderMotion().getOrientation();
		return new Angle(o.thirdAngle, o.angleUnit);
	}
	
	public enum turnDirection{CW, CCW}
}
