package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Math2;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.DistanceMeasure;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareMap;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.Motor;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.MotorSimple;

import com.qualcomm.robotcore.hardware.DcMotor;


/**An abstract class for drivetrains that can drive in any direction without turning*/
public abstract class AllDirectionsDrive extends Drive {
	/**The type of OpMode, AUTO (Autonomous) or TELE (TeleOp) */
	public OpModeType opModeType;
	/**the angle of the front of the robot relative to the first wheel motor*/
    public Angle headAngle;

	/**The wheel motors should be listed clockwise.
    A line drawn from the center of the robot to the first wheel motor listed indicates the positive x-axis.
    headOffset is measured counterclockwise from the positive x-axis and is used in determining the autonomous axes.
    The autonomous axes are the axes in which the "front" of the robot is positioned at zero degrees (on the positive x-axis).
    This is different than the teleop axes in which the y-axis is flipped (since the gamepad y-axis is flipped) and the "front"
    of the robot is position at -90 degrees (on the negative y-axis)*/
	public AllDirectionsDrive(DcMotor[] motors, Angle headOffset, OpModeType type) {
		super(motors);
		headAngle = headOffset;
		opModeType = type;
	}
	/**The wheel motors should be listed clockwise.
    A line drawn from the center of the robot to the first wheel motor listed indicates the positive x-axis.
    headOffset is measured counterclockwise from the positive x-axis and is used in determining the autonomous axes.
    The autonomous axes are the axes in which the "front" of the robot is positioned at zero degrees (on the positive x-axis).
    This is different than the teleop axes in which the y-axis is flipped (since the gamepad y-axis is flipped) and the "front"
    of the robot is position at -90 degrees (on the negative y-axis)*/
	public AllDirectionsDrive(Motor[] motors, Angle headOffset, OpModeType type) {
		super(motors);
		headAngle = headOffset;
		opModeType = type;
	}

	public AllDirectionsDrive(HardwareMap map, Angle headOffset, OpModeType type, String... motorNames){
	    super(map, motorNames);
	    headAngle = headOffset;
	    opModeType = type;
    }

    /**Drives the robot at a set angle, relative to the robot user axes*/
	public abstract void driveAtAngle(Angle angle);

	/**Converts a joystick drive angle (from a tele op program) 
	to an autonomous drive angle (used for determining the wheel powers).
	This conversion should be valid for all AllDirections drivetrains as long as the autonomous "front" is
	the same thing on the robot as the teleop "front".
	<br><br>
	The goal is to have a drive angle defined relative to the teleop robot axes converted to
	an angle defined relative to the autonomous axes without changing the angle relative to the robot itself.
	
	@see AllDirectionsDrive#AllDirectionsDrive(Motor[], Angle, OpModeType)*/
	public Angle teleToAutoDriveAngle(Angle teleDriveAngle) {
		return Angle.negate(new Angle(90, AngleUnit.DEGREES).add(teleDriveAngle));
	}
	
	/**Drives the robot to a set position while turning to a set angle.
	Requires that the robot be at the target position and target angle before returning true. 
	NO OTHER DRIVING METHODS SHOULD BE USED WHEN THIS METHOD IS CALLED.
	You do not need to call <code>drive()</code> when using this method.
	<code>turnSpeed</code> and <code>driveSpeed</code> work in a similar manner as in <code>turnTo</code> and <code>driveTo</code>,
	but there is not a simple relation. 
	@return if the robot has driven to the set position and turned to the set angle*/
	@Deprecated
	public boolean turnDriveTo(VectorF currentPose, VectorF targetPose, Angle currentAngle, Angle targetAngle) {
		Angle newTargetAngle = Angle.convertAngle(targetAngle, currentAngle);
		if(Math2.round(Math2.abs(currentPose.subtracted(targetPose)).magnitude(), driveError*2) == 0
		&& Math2.round(Angle.abs(Angle.subtract(currentAngle, newTargetAngle)), Angle.mult(turnError, 2)).getDegree() == 0) {
			firstMoveToCall = true;
			maxDiffDist = null;
			return true;
		}
		else {
			if(firstMoveToCall) {
				maxDiffDist = new DistanceMeasure(Math.abs(targetPose.subtracted(currentPose).magnitude()));
			}
			
			double prevTurnSpeed = turnSpeed;
			double prevDriveSpeed = driveSpeed;
			double setSpeed = speed;
			
			if(maxDiffDist == null) throw new IllegalStateException("firstMoveToCall was not set to true");
			else {
				turnSpeed = Angle.abs(Angle.subtract(newTargetAngle, currentAngle)).div(maxDiffAngle)*setSpeed/(1-prevTurnSpeed);
				driveSpeed = Math.abs(targetPose.subtracted(currentPose).magnitude())*setSpeed/((1-prevDriveSpeed)*maxDiffDist.value);
				speed = turnSpeed+driveSpeed;
			}
			if(speed > setSpeed) speed = setSpeed;
			
			Angle driveAngle = new Angle(Math.atan2(targetPose.get(1)-currentPose.get(1), targetPose.get(0)-currentPose.get(0)), AngleUnit.RADIANS);
			
			driveAtAngle(Angle.subtract(driveAngle, Angle.add(currentAngle, headAngle)));
			turn((newTargetAngle.greaterThan(currentAngle)) ? turnDirection.CCW : turnDirection.CW);
			drive();
			
			turnSpeed = prevTurnSpeed;
			driveSpeed = prevDriveSpeed;
			speed = setSpeed;
			
			firstMoveToCall = false;
			return false;
		}
	}
    /**Drives the robot to a set position while turning to a set angle.
     Requires that the robot be at the target position and target angle before returning true.
     NO OTHER DRIVING METHODS SHOULD BE USED WHEN THIS METHOD IS CALLED.
     You do not need to call <code>drive()</code> when using this method.
     <code>turnSpeed</code> and <code>driveSpeed</code> work in a similar manner as in <code>turnTo</code> and <code>driveTo</code>,
     but there is not a simple relation.
     @return if the robot has driven to the set position and turned to the set angle*/
	public boolean turnDriveTo(Position currentPose, Position targetPose, Angle currentAngle, Angle targetAngle){
        Angle newTargetAngle = Angle.convertAngle(targetAngle, currentAngle);
        Angle angleError = Angle.abs(Angle.subtract(newTargetAngle, currentAngle));
        Position newTargetPose = targetPose.createWithUnit(currentPose.unit);
        DistanceMeasure distanceError = new DistanceMeasure(Math.abs(currentPose.subtracted(newTargetPose).magnitude()), currentPose.unit);

        if(Math2.round(distanceError.value, getDriveError().getValue(currentPose.unit)*2) == 0
                && Math2.round(angleError, Angle.mult(turnError, 2)).getDegree() == 0) {
            firstMoveToCall = true;
            maxDiffDist = null;
            return true;
        }
        else {
            if(firstMoveToCall) {
                maxDiffDist = distanceError;
            }

            double prevTurnSpeed = turnSpeed;
            double prevDriveSpeed = driveSpeed;
            double setSpeed = speed;

            if(maxDiffDist == null) throw new IllegalStateException("firstMoveToCall was not set to true");
            else {
                turnSpeed = angleError.div(maxDiffAngle)*setSpeed/(1-prevTurnSpeed);
                driveSpeed = distanceError.div(maxDiffDist)*setSpeed/(1-prevDriveSpeed);
                speed = turnSpeed+driveSpeed;
            }
            if(speed > setSpeed) speed = setSpeed;

            Angle driveAngle = new Angle(Math.atan2(newTargetPose.y-currentPose.y, newTargetPose.x-currentPose.x), AngleUnit.RADIANS);

            driveAtAngle(Angle.subtract(driveAngle, Angle.add(currentAngle, headAngle)));
            turn((newTargetAngle.greaterThan(currentAngle)) ? turnDirection.CCW : turnDirection.CW);
            drive();

            turnSpeed = prevTurnSpeed;
            driveSpeed = prevDriveSpeed;
            speed = setSpeed;

            firstMoveToCall = false;
            return false;
        }
    }
	
	/**Drives the robot to a set position while turning to a set angle.
  	Requires that the robot be at the target position, but not the target angle before returning true. 
	NO OTHER DRIVING METHODS SHOULD BE USED WHEN THIS METHOD IS CALLED.
	You do not need to call <code>drive()</code> when using this method.
	<code>turnSpeed</code> and <code>driveSpeed</code> work in a similar manner as in <code>turnTo</code> and <code>driveTo</code>,
	but there is not a simple relation. 
	@return if the robot has driven to the set position*/
	@Deprecated
	public boolean driveToWithTurn(VectorF currentPose, VectorF targetPose, Angle currentAngle, Angle targetAngle) {
		Angle newTargetAngle = Angle.convertAngle(targetAngle, currentAngle);
		if(Math2.round(Math2.abs(currentPose.subtracted(targetPose)).magnitude(), driveError*2) == 0) {
			firstMoveToCall = true;
			maxDiffDist = null;
			return true;
		}
		else {
			if(firstMoveToCall) {
				maxDiffDist = new DistanceMeasure(Math.abs(targetPose.subtracted(currentPose).magnitude()));
			}
			
			double prevTurnSpeed = turnSpeed;
			double prevDriveSpeed = driveSpeed;
			double setSpeed = speed;
			
			if(maxDiffDist == null) throw new IllegalStateException("firstMoveToCall was not set to true");
			else {
				turnSpeed = Angle.abs(Angle.subtract(newTargetAngle, currentAngle)).div(maxDiffAngle)*setSpeed/(1-prevTurnSpeed);
				driveSpeed = Math.abs(targetPose.subtracted(currentPose).magnitude())*setSpeed/((1-prevDriveSpeed)*maxDiffDist.value);
				speed = turnSpeed+driveSpeed;
			}
			if(speed > setSpeed) speed = setSpeed;
			
			Angle driveAngle = new Angle(Math.atan2(targetPose.get(1)-currentPose.get(1), targetPose.get(0)-currentPose.get(0)), AngleUnit.RADIANS);
			
			driveAtAngle(Angle.subtract(driveAngle, Angle.add(currentAngle, headAngle)));
			turn((newTargetAngle.greaterThan(currentAngle)) ? turnDirection.CCW : turnDirection.CW);
			drive();
			
			turnSpeed = prevTurnSpeed;
			driveSpeed = prevDriveSpeed;
			speed = setSpeed;
			
			firstMoveToCall = false;
			return false;
		}
	}
    /**Drives the robot to a set position while turning to a set angle.
     Requires that the robot be at the target position, but not the target angle before returning true.
     NO OTHER DRIVING METHODS SHOULD BE USED WHEN THIS METHOD IS CALLED.
     You do not need to call <code>drive()</code> when using this method.
     <code>turnSpeed</code> and <code>driveSpeed</code> work in a similar manner as in <code>turnTo</code> and <code>driveTo</code>,
     but there is not a simple relation.
     @return if the robot has driven to the set position*/
	public boolean driveToWithTurn(Position currentPose, Position targetPose, Angle currentAngle, Angle targetAngle){
        Angle newTargetAngle = Angle.convertAngle(targetAngle, currentAngle);
        Angle angleError = Angle.abs(Angle.subtract(newTargetAngle, currentAngle));
        Position newTargetPose = targetPose.createWithUnit(currentPose.unit);
        DistanceMeasure distanceError = new DistanceMeasure(Math.abs(currentPose.subtracted(newTargetPose).magnitude()), currentPose.unit);

        if(Math2.round(distanceError.value, getDriveError().getValue(currentPose.unit)*2) == 0) {
            firstMoveToCall = true;
            maxDiffDist = null;
            return true;
        }
        else {
            if(firstMoveToCall) {
                maxDiffDist = distanceError.copy();
            }

            double prevTurnSpeed = turnSpeed;
            double prevDriveSpeed = driveSpeed;
            double setSpeed = speed;

            if(maxDiffDist == null) throw new IllegalStateException("firstMoveToCall was not set to true");
            else {
                turnSpeed = angleError.div(maxDiffAngle)*setSpeed/(1-prevTurnSpeed);
                driveSpeed = distanceError.div(maxDiffDist)*setSpeed/(1-prevDriveSpeed);
                speed = turnSpeed+driveSpeed;
            }
            if(speed > setSpeed) speed = setSpeed;

            Angle driveAngle = new Angle(Math.atan2(newTargetPose.y-currentPose.y, newTargetPose.x-currentPose.x), AngleUnit.RADIANS);

            driveAtAngle(Angle.subtract(driveAngle, Angle.add(currentAngle, headAngle)));
            turn((newTargetAngle.greaterThan(currentAngle)) ? turnDirection.CCW : turnDirection.CW);
            drive();

            turnSpeed = prevTurnSpeed;
            driveSpeed = prevDriveSpeed;
            speed = setSpeed;

            firstMoveToCall = false;
            return false;
        }
	}
	
	/**Turns the robot in the given direction*/
    @Override
    public void turn(turnDirection direction){
        for(Motor wheelMotor : wheelMotors) {
        	if(direction == turnDirection.CCW) wheelMotor.setPower(turnSpeed);
        	else wheelMotor.setPower(-turnSpeed);
        }
    }
	
	@Override
    @Deprecated
	public boolean driveTo(VectorF currentPose, VectorF targetPose, Angle heading) {
		if(Math2.round(Math2.abs(currentPose.subtracted(targetPose)).magnitude(), driveError*2) == 0) {
			firstMoveToCall = true;
			maxDiffDist = null;
			return true;
		}
		else {
			double setSpeed = speed;
			if(firstMoveToCall) {
				maxDiffDist = new DistanceMeasure(Math.abs(targetPose.subtracted(currentPose).magnitude()));
			}
			if(maxDiffDist == null) throw new IllegalStateException("firstMoveToCall was not set to true");
			else speed = Math.abs(targetPose.subtracted(currentPose).magnitude())*setSpeed/((1-driveSpeed)*maxDiffDist.value);
			
			if(speed > setSpeed) speed = setSpeed;
			
			Angle driveAngle = new Angle(Math.atan2(targetPose.get(1)-currentPose.get(1), targetPose.get(0)-currentPose.get(0)), AngleUnit.RADIANS); 
			driveAtAngle(Angle.subtract(driveAngle, Angle.add(heading, headAngle)));
			
			drive();
			
			speed = setSpeed;
			
			firstMoveToCall = false;
			return false;
		}
	}

    @Override
    public boolean driveTo(Position currentPose, Position targetPose, Angle heading) {
        Position newTargetPose = targetPose.createWithUnit(currentPose.unit);
        DistanceMeasure error = new DistanceMeasure(Math.abs(currentPose.subtracted(newTargetPose).magnitude()), currentPose.unit);
        if(Math2.round(error.value, allowedDriveError.getValue(currentPose.unit)*2) == 0) {
            firstMoveToCall = true;
            maxDiffDist = null;
            return true;
        }
        else {
            double setSpeed = speed;
            if(firstMoveToCall) {
                maxDiffDist = error.copy();
            }
            if(maxDiffDist == null) throw new IllegalStateException("firstMoveToCall was not set to true");
            else speed = error.div(maxDiffDist)*setSpeed/(1-driveSpeed);

            if(speed > setSpeed) speed = setSpeed;

            Angle driveAngle = new Angle(Math.atan2(targetPose.y-currentPose.y, targetPose.x-currentPose.x), AngleUnit.RADIANS);
            driveAtAngle(Angle.subtract(driveAngle, Angle.add(heading, headAngle)));

            drive();

            speed = setSpeed;

            firstMoveToCall = false;
            return false;
        }
    }
    @Override
    public boolean driveTo(Position currentPose, Position targetPose, Angle heading, DistanceMeasure distanceTillP, boolean continuous) {
        Position newTargetPose = targetPose.createWithUnit(currentPose.unit);
        DistanceMeasure error = new DistanceMeasure(Math.abs(currentPose.subtracted(newTargetPose).magnitude()), currentPose.unit);
        distanceTillP.toUnit(currentPose.unit);
        double setSpeed = speed;

        if(Math2.round(error.value, allowedDriveError.getValue(currentPose.unit)*2) == 0) {
            firstMoveToCall = true;
            maxDiffDist = null;
            return true;
        }
        else if(error.lessThan(distanceTillP)){
            if(continuous){
                speed = error.div(distanceTillP)*setSpeed;
            }
            else{
                speed = error.div(distanceTillP)*setSpeed*driveSpeed;
            }
            if(speed > setSpeed) speed = setSpeed;
        }

        Angle driveAngle = new Angle(Math.atan2(targetPose.y-currentPose.y, targetPose.x-currentPose.x), AngleUnit.RADIANS);
        driveAtAngle(Angle.subtract(driveAngle, Angle.add(heading, headAngle)));

        drive();

        speed = setSpeed;

        firstMoveToCall = false;
        return false;
    }

	@Override
    public void drive () {
        //scale the motor powers down so that the absolute values don't exceed speed
        MotorSimple.normalizePowers(speed, wheelMotors);

        super.drive();
    }

    /**Update the encoder motion given the robot's heading*/
    public abstract void updateEncoderMotion(Angle heading);
    /**Update the encoder motion assuming that the robot was turning and driving at the same time.*/
    public abstract void updateEncoderMotion(Angle prevHeading, Angle heading);

    /**Returns the last angle passed to {@link #driveAtAngle(Angle angle)}*/
    public abstract Angle getDriveAngle();
}
