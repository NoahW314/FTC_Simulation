package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcalamari.Math2;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.DistanceMeasure;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareMap;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.Motor;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.MotorSimple;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TankDrive extends Drive {
	/**The angle the robot should drive at to reach the target*/
	public Angle driveAngle;
	
	/**The state the robot is in in the driveTo method*/
	public DriveToStates driveToState = DriveToStates.START;

	/**The wheels should be listed left to right then front to back.
	So in a typical four wheel tank drive the wheels should be listed in this order: 
	frontLeft, frontRight, backLeft, backRight*/
	public TankDrive(DcMotor[] motors) {
		super(motors);
	}
	/**The wheels should be listed left to right then front to back.
	So in a typical four wheel tank drive the wheels should be listed in this order: 
	frontLeft, frontRight, backLeft, backRight*/
	public TankDrive(Motor[] motors) {
		super(motors);
	}
	public TankDrive(HardwareMap map, String... motorNames){super(map, motorNames);}

	@Override
	public void turn(turnDirection direction) {
		for(Motor wheelMotor : wheelMotors) {
			if(direction == turnDirection.CCW) wheelMotor.setPower(turnSpeed);
			else wheelMotor.setPower(-turnSpeed);
		}
	}

	/**Drive to a set position. NO OTHER DRIVING METHODS SHOULD BE USED WHEN THIS METHOD IS CALLED.
	You do not need to call <code>drive()</code> when using this method.
	<code>driveSpeed</code> is <i>roughly</i> the fraction of the distance during which the robot is moving at <code>speed</code>.
	This is not exact as the drive will try to correct its heading while driving toward the target.
	<code>driveSpeed</code> should be in the interval [0,1).
	Values very close to 1 should be avoided as the robot may not be able to slow down in time and may overshoot the target.
	@return if the robot has driven to the specified point*/
	@Override
    @Deprecated
	public boolean driveTo(VectorF currentPose, VectorF targetPose, Angle heading) {
		switch(driveToState) {
			case START:
				driveAngle = new Angle(Math.atan2(targetPose.get(1)-currentPose.get(1), targetPose.get(0)-currentPose.get(0))+Math.PI, AngleUnit.RADIANS);
				driveToState = DriveToStates.TURN;
			case TURN:
				if(turnTo(heading, driveAngle)) {
					driveToState = DriveToStates.DRIVE;
				}
				break;
			case DRIVE:
				if(driveCorrectiveTo(new Position(currentPose), new Position(targetPose), heading)) {
					driveToState = DriveToStates.START;
					return true;
				}
				break;
		}
		return false;
	}

    /**Drive to a set position. NO OTHER DRIVING METHODS SHOULD BE USED WHEN THIS METHOD IS CALLED.
     You do not need to call <code>drive()</code> when using this method.
     <code>driveSpeed</code> is <i>roughly</i> the fraction of the distance during which the robot is moving at <code>speed</code>.
     This is not exact as the drive will try to correct its heading while driving toward the target.
     <code>driveSpeed</code> should be in the interval [0,1).
     Values very close to 1 should be avoided as the robot may not be able to slow down in time and may overshoot the target.
     @return if the robot has driven to the specified point*/
    public boolean driveTo(Position currentPose, Position targetPose, Angle heading){
        Position newTargetPose = targetPose.createWithUnit(currentPose.unit);
        switch(driveToState) {
            case START:
                driveAngle = new Angle(Math.atan2(newTargetPose.y-currentPose.y, newTargetPose.x-currentPose.x)+Math.PI, AngleUnit.RADIANS);
                driveToState = DriveToStates.TURN;
            case TURN:
                if(turnTo(heading, driveAngle)) {
                    driveToState = DriveToStates.DRIVE;
                }
                break;
            case DRIVE:
                if(driveCorrectiveTo(currentPose, newTargetPose, heading)) {
                    driveToState = DriveToStates.START;
                    return true;
                }
                break;
        }
        return false;
    }

    public boolean driveTo(Position currentPose, Position targetPose, Angle heading, DistanceMeasure distanceTillP, boolean continuous){
        Position newTargetPose = targetPose.createWithUnit(currentPose.unit);
        switch(driveToState) {
            case START:
                driveAngle = new Angle(Math.atan2(newTargetPose.y-currentPose.y, newTargetPose.x-currentPose.x)+Math.PI, AngleUnit.RADIANS);
                driveToState = DriveToStates.TURN;
            case TURN:
                if(turnTo(heading, driveAngle)) {
                    driveToState = DriveToStates.DRIVE;
                }
                break;
            case DRIVE:
                if(driveCorrectiveTo(currentPose, newTargetPose, heading, distanceTillP, continuous)) {
                    driveToState = DriveToStates.START;
                    return true;
                }
                break;
        }
        return false;
    }
	
	private boolean driveCorrectiveTo(Position currentPose, Position targetPose, Angle heading) {
        Position newTargetPose = targetPose.createWithUnit(currentPose.unit);
        allowedDriveError.toUnit(currentPose.unit);
        double error = Math.abs(currentPose.subtracted(newTargetPose).magnitude());
		if(Math2.round(error, allowedDriveError.value*2) == 0) {
			firstMoveToCall = true;
			maxDiffDist = null;
			return true;
		}
		else {
			double setSpeed = speed;
			if(firstMoveToCall) {
				maxDiffDist = new DistanceMeasure(error, currentPose.unit);
			}

			if(maxDiffDist == null) throw new IllegalStateException("firstMoveToCall was not set to true");
			else {
			    maxDiffDist.toUnit(currentPose.unit);
			    speed = error*setSpeed/((1-driveSpeed)*maxDiffDist.value);
            }
			
			if(speed > setSpeed) speed = setSpeed;
			
			Angle targetAngle = new Angle((Math.atan2(currentPose.y-newTargetPose.y, currentPose.y-newTargetPose.y)+Math.PI), AngleUnit.RADIANS);
			Angle convertedTargetAngle = Angle.convertAngle(targetAngle, heading);
			double setTurnSpeed = turnSpeed;
			turnSpeed = setSpeed*setTurnSpeed*Angle.abs(Angle.subtract(convertedTargetAngle, heading)).getDegree();
			turn((convertedTargetAngle.greaterThan(heading)) ? turnDirection.CCW : turnDirection.CW);
			
			driveDirection((Angle.abs(Angle.subtract(convertedTargetAngle, heading)).getDegree() <= 90) ? driveDirection.FORWARD : driveDirection.BACKWARD);
			
			drive();
			
			speed = setSpeed;
			turnSpeed = setTurnSpeed;
			firstMoveToCall = false;
			return false;
		}
	}
	private boolean driveCorrectiveTo(Position currentPose, Position targetPose, Angle heading, DistanceMeasure distanceTillP, boolean continuous){
        Position newTargetPose = targetPose.createWithUnit(currentPose.unit);
        allowedDriveError.toUnit(currentPose.unit);
        distanceTillP.toUnit(currentPose.unit);
        DistanceMeasure error = new DistanceMeasure(Math.abs(currentPose.subtracted(newTargetPose).magnitude()), currentPose.unit);
        double setSpeed = speed;
        if(Math2.round(error.value, allowedDriveError.value*2) == 0) {
            firstMoveToCall = true;
            maxDiffDist = null;
            return true;
        }
        else if(error.lessThan(distanceTillP)){
            if(continuous) {
                speed = error.div(distanceTillP)*setSpeed;
            }
            else {
                speed = error.div(distanceTillP)*setSpeed*driveSpeed;
            }
            if(speed > setSpeed) speed = setSpeed;
        }
        Angle targetAngle = new Angle((Math.atan2(currentPose.y-newTargetPose.y, currentPose.y-newTargetPose.y)+Math.PI), AngleUnit.RADIANS);
        Angle convertedTargetAngle = Angle.convertAngle(targetAngle, heading);
        double setTurnSpeed = turnSpeed;
        turnSpeed = setSpeed*setTurnSpeed*Angle.abs(Angle.subtract(convertedTargetAngle, heading)).getDegree();
        turn((convertedTargetAngle.greaterThan(heading)) ? turnDirection.CCW : turnDirection.CW);

        driveDirection((Angle.abs(Angle.subtract(convertedTargetAngle, heading)).getDegree() <= 90) ? driveDirection.FORWARD : driveDirection.BACKWARD);

        drive();

        speed = setSpeed;
        turnSpeed = setTurnSpeed;
        firstMoveToCall = false;
        return false;
    }

	/**Drives the robot forwards or backwards*/
	public void driveDirection(driveDirection direction) {
		switch(direction) {
			case BACKWARD:
				for(int i = 0; i < wheelMotors.length; i++) {
					wheelMotors[i].setPower(-(driveSpeed*(-(i+1)%2*2+1)));
				}
				break;
			case FORWARD:
				for(int i = 0; i < wheelMotors.length; i++) {
					wheelMotors[i].setPower((driveSpeed*(-(i+1)%2*2+1)));
				}
				break;
			default:
				throw new IllegalArgumentException("You can only drive forwards or backwards");
		}
	}

	@Override
	public void drive() {
		//scale the motor powers down so that the absolute values don't exceed speed
        MotorSimple.normalizePowers(speed, wheelMotors);

        super.drive();
	}
	
	@Override
	public void updateEncoderMotion() {
		int[] encoderDifferences = new int[2];
		for(int i = 0; i < 2; i++) {
			encoderDifferences[i] = wheelMotors[i].getCurrentPosition()-lastEncoderPositions[i];
		}
        double driveDistance = (encoderDifferences[1]-encoderDifferences[0])/2*inchesPerTick;
		double turnDistance = (encoderDifferences[0]+encoderDifferences[1])/2*inchesPerTick;
		
		//degrees
        Orientation orientation = encoderMotion.getOrientation();
		orientation.thirdAngle+=(turnDistance/turnRadius.getValue(DistanceUnit.INCH)*180/Math.PI);
		orientation.thirdAngle = (float) Math2.to360(orientation.thirdAngle);
		encoderMotion.setOrientation(orientation);
		
		//radians
		double heading = orientation.thirdAngle*Math.PI/180;
		Position pose = encoderMotion.getPosition();
        pose.x+=driveDistance*Math.cos(heading);
        pose.y+=driveDistance*Math.sin(heading);
        encoderMotion.setPosition(pose);
	}
	@Override
    public void updateEncoderMotion(Angle heading){
        int[] encoderDifferences = new int[2];
        for(int i = 0; i < 2; i++) {
            encoderDifferences[i] = wheelMotors[i].getCurrentPosition()-lastEncoderPositions[i];
        }
        double driveDistance = (encoderDifferences[1]-encoderDifferences[0])/2*inchesPerTick;

        Position pose = encoderMotion.getPosition();
        pose.x+=driveDistance*Math.cos(heading.getRadian());
        pose.y+=driveDistance*Math.sin(heading.getRadian());
        encoderMotion.setPosition(pose);
    }
	
	public enum DriveToStates{START, TURN, DRIVE}
	public enum driveDirection{FORWARD, BACKWARD}
}
