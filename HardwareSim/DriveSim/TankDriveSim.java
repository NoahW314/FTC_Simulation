package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.DriveSim;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Math2;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.MotorSim;

public class TankDriveSim extends DriveSim {
	
	public Angle driveAngle;
	
	public DriveToStates driveToState = DriveToStates.START;

	/**The wheels should be listed left to right then front to back.
	So in a typical four wheel tank drive the wheels should be listed in this order: 
	frontLeft, frontRight, backLeft, backRight*/
	public TankDriveSim(MotorSim[] motors) {
		super(motors);
	}

	@Override
	public void turn(turnDirection direction) {
		for(int i = 0; i < wheelMotors.length; i++) {
			if(direction == turnDirection.CCW) wheelMotors[i].setPower(turnSpeed);
			else wheelMotors[i].setPower(-turnSpeed);
		}
	}

	@Override
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
				if(driveCorrectiveTo(currentPose, targetPose, heading)) {
					driveToState = DriveToStates.START;
					return true;
				}
				break;
		}
		return false;
	}
	
	private boolean driveCorrectiveTo(VectorF currentPose, VectorF targetPose, Angle heading) {
		if(Math2.round(Math2.abs(currentPose.subtracted(targetPose)).magnitude(), driveError*2) == 0) {
			firstMoveToCall = true;
			maxDiffDist = null;
			return true;
		}
		else {
			double setSpeed = speed;
			if(firstMoveToCall) {
				maxDiffDist = Math.abs(targetPose.subtracted(currentPose).magnitude());
			}
			
			if(maxDiffDist == 0) throw new IllegalStateException("target position and starting position are equal");
			else if(maxDiffDist == null) throw new IllegalStateException("firstMoveToCall was not set to true"); 
			else speed = Math.abs(targetPose.subtracted(currentPose).magnitude())*setSpeed/((1-driveSpeed)*maxDiffDist);
			
			if(speed > setSpeed) speed = setSpeed;
			
			Angle targetAngle = new Angle((Math.atan2(targetPose.get(1)-currentPose.get(1), targetPose.get(0)-currentPose.get(0))+Math.PI), AngleUnit.RADIANS);
			Angle convertedTargetAngle = Angle.convertAngle(targetAngle, heading);
			double setTurnSpeed = turnSpeed;
			Angle angleDiff = (Angle.abs(Angle.subtract(convertedTargetAngle, heading)));
			turnSpeed = setSpeed*setTurnSpeed*angleDiff.getDegree();
			turn((convertedTargetAngle.greaterThan(heading)) ? turnDirection.CCW : turnDirection.CW);
			
			driveDirection((Angle.abs(Angle.subtract(convertedTargetAngle, heading)).getDegree() >= 90) ? driveDirection.FORWARD : driveDirection.BACKWARD);
			
			drive();
			
			speed = setSpeed;
			turnSpeed = setTurnSpeed;
			firstMoveToCall = false;
			return false;
		}
	}
	/**Drives the robot forwards or backwards*/
	public void driveDirection(driveDirection direction) throws IllegalArgumentException {
		switch(direction) {
			case BACKWARD:
				for(int i = 0; i < wheelMotors.length; i++) {
					wheelMotors[i].setPower(-(driveSpeed*((-(i+1)%2*2)+1)));
				}
				break;
			case FORWARD:
				for(int i = 0; i < wheelMotors.length; i++) {
					wheelMotors[i].setPower((driveSpeed*((-(i+1)%2*2)+1)));
				}
				break;
			default:
				throw new IllegalArgumentException("You can only drive forwards or backwards");
		}
	}

	@Override
	public void drive() {
		super.drive();
		//scale the motor powers down so that the absolute values don't exceed 1

		//determine the maximum wheel power
		double maxi = Math.abs(wheelMotors[0].getCurrentPower());
		for(int i = 1; i < wheelMotors.length; i++){
		    if(Math.abs(wheelMotors[i].getCurrentPower()) > maxi){
		    	maxi = Math.abs(wheelMotors[i].getCurrentPower());
		    }
		}

        /*divide all the motor powers by the maximum power to preserve
        the ratios between the wheels while keeping the powers under 1*/
        if(maxi != 0){
            for(int i = 0; i < wheelMotors.length; i++){
                wheelMotors[i].resetPower(wheelMotors[i].getCurrentPower()*speed/maxi);
            }
        }

        //set the wheel motor powers
        for(int i = 0; i < wheelMotors.length; i++) {
            wheelMotors[i].run();
        }
	}
	
	@Override
	public void updateEncoderMotion() {
		double[] encoderDifferences = new double[2];
		for(int i = 0; i < 2; i++) {
			encoderDifferences[i] = wheelMotors[i].getCurrentPosition()-lastEncoderPositions[i];
		}
		double driveDistance = (encoderDifferences[1]-encoderDifferences[0])/2*inchesPerTick;
		double turnDistance = (encoderDifferences[0]+encoderDifferences[1])/2*inchesPerTick;
		
		//degrees
		encoderMotion.orientation.thirdAngle+=(turnDistance/turnRadius*180/Math.PI);
		encoderMotion.orientation.thirdAngle = (float) Math2.to360(encoderMotion.orientation.thirdAngle); 
		
		//radians
		double heading = encoderMotion.orientation.thirdAngle*Math.PI/180;
		encoderMotion.position.x+=driveDistance*Math.cos(heading);
		encoderMotion.position.y+=driveDistance*Math.sin(heading);
	}
	
	public enum DriveToStates{START, TURN, DRIVE}
	public enum driveDirection{FORWARD, BACKWARD}
}
