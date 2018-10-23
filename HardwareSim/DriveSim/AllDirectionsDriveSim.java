package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.DriveSim;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Math2;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.MotorSim;


/**An abstract class for drivetrains that can drive in any direction without turning*/
public abstract class AllDirectionsDriveSim extends DriveSim {
	/**The type of OpMode, AUTO (Autonomous) or TELE (TeleOp) */
	public OpModeType opModeType;
	/**the angle of the front of the robot relative to the first wheel motor*/
    public Angle headAngle;

	/**The wheel motors should be listed clockwise.
    A line drawn from the center of the robot to the first wheel motor listed indicates the positive x-axis.
    headOffset is measured in degrees, counterclockwise from the positive x-axis and is used in determing the autonomous axes.
    The autonomous axes are the axes in which the "front" of the robot is positioned at zero degrees (on the positive x-axis).
    This is different than the teleop axes in which the y-axis is flipped (since the gamepad y-axis is flipped) and the "front"
    of the robot is position at -90 degrees (on the negative y-axis)*/
	public AllDirectionsDriveSim(MotorSim[] motors, Angle headOffset, OpModeType type) {
		super(motors);
		headAngle = headOffset;
		opModeType = type;
	}

    /**Drives the robot at a set angle, relative to the robot user axes
     * @return */
	public abstract void driveAtAngle(Angle angle);
	
	/**Converts a joystick drive angle (from a tele op program) 
	to an autonomous drive angle (used for determining the wheel powers).
	This conversion should be valid for all AllDirections drivetrains as long as the autonomous "front" is
	the same thing on the robot as the teleop "front".
	<br><br>
	The goal is to have a drive angle defined relative to the teleop robot axes converted to
	an angle defined relative to the autonomous axes without changing the angle relative to the robot itself.
	
	@see AllDirectionsDriveSim#AllDirectionsDriveSim(MotorSim[], Angle, OpModeType)*/
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
				maxDiffDist = Math.abs(targetPose.subtracted(currentPose).magnitude());
			}
			
			double prevTurnSpeed = turnSpeed;
			double prevDriveSpeed = driveSpeed;
			double setSpeed = speed;
			
			if(maxDiffDist == 0) throw new IllegalStateException("target position and starting position are equal");
			else if(maxDiffDist == null) throw new IllegalStateException("firstMoveToCall was not set to true"); 
			else {
				turnSpeed = Angle.abs(Angle.subtract(newTargetAngle, currentAngle)).div(maxDiffAngle)*setSpeed/(1-prevTurnSpeed);
				driveSpeed = Math.abs(targetPose.subtracted(currentPose).magnitude())*setSpeed/((1-prevDriveSpeed)*maxDiffDist);
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
	@return if the robot has driven to the set position and turned to the set angle*/
	public boolean driveToWithTurn(VectorF currentPose, VectorF targetPose, Angle currentAngle, Angle targetAngle) {
		Angle newTargetAngle = Angle.convertAngle(targetAngle, currentAngle);
		if(Math2.round(Math2.abs(currentPose.subtracted(targetPose)).magnitude(), driveError*2) == 0) {
			firstMoveToCall = true;
			maxDiffDist = null;
			return true;
		}
		else {
			if(firstMoveToCall) {
				maxDiffDist = Math.abs(targetPose.subtracted(currentPose).magnitude());
			}
			
			double prevTurnSpeed = turnSpeed;
			double prevDriveSpeed = driveSpeed;
			double setSpeed = speed;
			
			//TODO: rethink if first if statement is needed
			if(maxDiffDist == 0) throw new IllegalStateException("target position and starting position are equal");
			else if(maxDiffDist == null) throw new IllegalStateException("firstMoveToCall was not set to true"); 
			else {
				turnSpeed = Angle.abs(Angle.subtract(newTargetAngle, currentAngle)).div(maxDiffAngle)*setSpeed/(1-prevTurnSpeed);
				driveSpeed = Math.abs(targetPose.subtracted(currentPose).magnitude())*setSpeed/((1-prevDriveSpeed)*maxDiffDist);
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
	
	/**Turns the robot in the given direction*/
    @Override
    public void turn(turnDirection direction){
        for(int i = 0; i < wheelMotors.length; i++) {
        	if(direction == turnDirection.CCW) wheelMotors[i].setPower(turnSpeed);
        	else wheelMotors[i].setPower(-turnSpeed);
        }
    }
	
	@Override
	public boolean driveTo(VectorF currentPose, VectorF targetPose, Angle heading) {
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
			
			Angle driveAngle = new Angle(Math.atan2(targetPose.get(1)-currentPose.get(1), targetPose.get(0)-currentPose.get(0)), AngleUnit.RADIANS); 
			driveAtAngle(Angle.subtract(driveAngle, Angle.add(heading, headAngle)));
			
			drive();
			
			speed = setSpeed;
			
			firstMoveToCall = false;
			return false;
		}
	}

	@Override
    public void drive () {
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
}
