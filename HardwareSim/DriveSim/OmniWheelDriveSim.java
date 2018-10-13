package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Math2;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim.MotorSim;

public class OmniWheelDriveSim extends AllDirectionsDriveSim {

    //the constructor
    /**The wheel motors should be listed clockwise.
    A line drawn from the center of the robot to the first wheel motor listed indicates the positive x-axis.
    headOffset is measured counterclockwise from the positive x-axis.*/
    public OmniWheelDriveSim(MotorSim[] wheels, Angle headOffset, OpModeType type){
    	super(wheels, headOffset, type);
    }
    
    /**Drives the robot at a set angle*/
    public void driveAtAngle(Angle driveAngle) {
    	//since the teleop and autonomous axes differ (see below method javadoc) convert 
    	//from a teleop drive angle to an autonomous drive angle
    	if(opModeType == OpModeType.TELE) {
    		driveAngle = teleToAutoDriveAngle(driveAngle);
    	}
    	
        //extract the x and y components from the vector
        double x = Math.cos(driveAngle.getRadian());
        double y = Math.sin(driveAngle.getRadian());

        double headAngleR = headAngle.getRadian();
        //rotates the x and y components to account for the front being offset
        double px = x*Math.cos(headAngleR)-y*Math.sin(headAngleR);
        double py = y*Math.cos(headAngleR)+x*Math.sin(headAngleR);

        //scale the components by the speed
        px*=driveSpeed;
        py*=driveSpeed;

        //store the powers of the wheels in an array so we can loop through them
        double[] f = new double[]{py, px, -py, -px};

        //add the calculated powers to the current wheel powers
        for(int i = 0; i < 4; i++) {
            wheelMotors[i].setPower(f[i]);
        }
    }
	@Override
	public void updateEncoderMotion() {
		double[] encoderDifferences = new double[4];
		for(int i = 0; i < 4; i++) {
			encoderDifferences[i] = wheelMotors[i].getCurrentPosition()-lastEncoderPositions[i];
		}
		double turnDistance = (encoderDifferences[0]+encoderDifferences[2])/2*inchesPerTick;
		
		encoderMotion.orientation.thirdAngle+=(turnDistance/turnRadius*180/Math.PI);
		encoderMotion.orientation.thirdAngle = (float) Math2.to360(encoderMotion.orientation.thirdAngle); 
		
		updateEncoderMotion(new Angle(encoderMotion.orientation.thirdAngle, AngleUnit.DEGREES));
	}
	public void updateEncoderMotion(Angle heading) {
		double[] encoderDifferences = new double[4];
		for(int i = 0; i < 4; i++) {
			encoderDifferences[i] = wheelMotors[i].getCurrentPosition()-lastEncoderPositions[i];
		}
		double driveX = (encoderDifferences[1]-encoderDifferences[3])/2*inchesPerTick;
		double driveY = (encoderDifferences[0]-encoderDifferences[2])/2*inchesPerTick;
		double driveDistance = Math.sqrt(Math.pow(driveX, 2)+Math.pow(driveY, 2));
		double theta = heading.getRadian()+Math.atan2(driveY, driveX);
		encoderMotion.position.x+=driveDistance*Math.cos(theta);
		encoderMotion.position.y+=driveDistance*Math.sin(theta);
	}
}
