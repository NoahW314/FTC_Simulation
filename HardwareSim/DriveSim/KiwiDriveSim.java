package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.DriveSim;

import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.MotorSim;

public class KiwiDriveSim extends OmniWheelDriveSim {

	/**The wheel motors should be listed clockwise.
    A line drawn from the center of the robot to the first wheel motor listed indicates the positive x-axis.
    headOffset is measured in degrees and is measured counterclockwise from the positive x-axis.*/
	public KiwiDriveSim(MotorSim[] motors, Angle headOffset, OpModeType type) {
		super(motors, headOffset, type);
	}
	
	@Override
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
        double[] f = new double[]{py, -py/2+Math.sqrt(3)/2*px, -py/2-Math.sqrt(3)/2*px};

        //add the calculated powers to the current wheel powers
        for(int i = 0; i < 3; i++) {
            wheelMotors[i].setPower(f[i]);
        }
	}

	@Override
    public void updateEncoderMotion(Angle heading) {
		double[] encoderDifferences = new double[3];
		double[] encPose = new double[3];
        for(int i = 0; i < 3; i++) {
            encPose[i] = wheelMotors[i].getCurrentPosition();
        }
        for(int i = 0; i < 3; i++){
            encoderDifferences[i] = encPose[i]-lastEncoderPositions[i];
            lastEncoderPositions[i] = encPose[i];
        }

        double driveX = (encoderDifferences[1]-encoderDifferences[2])/Math.sqrt(3)*inchesPerTick;
        double driveY = (encoderDifferences[0]-(encoderDifferences[2]+encoderDifferences[1])/2)*2/3*inchesPerTick;

        double driveDistance = Math.sqrt(Math.pow(driveX, 2)+Math.pow(driveY, 2));
        double theta = heading.getRadian()+Math.atan2(driveY, driveX);

        encoderMotion.position.x+=driveDistance*Math.cos(theta);
        encoderMotion.position.y+=driveDistance*Math.sin(theta);        
    }
}
