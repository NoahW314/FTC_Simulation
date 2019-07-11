package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.DriveClasses;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcalamari.Math2;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.DistanceMeasure;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareMap;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.Motor;

import com.qualcomm.robotcore.hardware.DcMotor;

public class KiwiDrive extends OmniWheelDrive {

	/**The wheel motors should be listed clockwise.
    A line drawn from the center of the robot to the first wheel motor listed indicates the positive x-axis.
    headOffset is measured counterclockwise from the positive x-axis.*/
	public KiwiDrive(Motor[] motors, Angle headOffset, OpModeType type) {
		super(motors, headOffset, type);
	}
	/**The wheel motors should be listed clockwise.
    A line drawn from the center of the robot to the first wheel motor listed indicates the positive x-axis.
    headOffset is measured counterclockwise from the positive x-axis.*/
	public KiwiDrive(DcMotor[] motors, Angle headOffset, OpModeType type) {
		super(motors, headOffset, type);
	}

	public KiwiDrive(HardwareMap map, Angle headOffset, OpModeType type, String... motorNames){
	    super(map, headOffset, type, motorNames);
    }
	
	@Override
	public void driveAtAngle(Angle driveAngle) {
        this.driveAngle = driveAngle.copy();

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
    public void updateEncoderMotion(){
        int[] encoderDifferences = new int[wheelMotors.length];
        for(int i = 0; i < wheelMotors.length; i++) {
            encoderDifferences[i] = wheelMotors[i].getCurrentPosition()-lastEncoderPositions[i];
        }
        DistanceMeasure turnDistance = new DistanceMeasure((encoderDifferences[0]+encoderDifferences[1]+encoderDifferences[2])/3*inchesPerTick, DistanceUnit.INCH);

        Orientation o = encoderMotion.getOrientation();
        o.thirdAngle+=(turnDistance.div(turnRadius)*180/Math.PI);
        o.thirdAngle = (float) Math2.to360(o.thirdAngle);
        encoderMotion.setOrientation(o);

        updateEncoderMotion(new Angle(o.thirdAngle, AngleUnit.DEGREES));
    }

	@Override
    public void updateEncoderMotion(Angle heading) {
		
		int[] encoderDifferences = new int[3];
		int[] encPose = new int[3];
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

        Position pose = encoderMotion.getPosition();
        pose.x+=driveDistance*Math.cos(theta);
        pose.y+=driveDistance*Math.sin(theta);
        encoderMotion.setPosition(pose);
    }
	
	public void updateEncoderMotionTurn(Angle prevHeading) {
		int[] encoderDifferences = new int[wheelMotors.length];
        for(int i = 0; i < wheelMotors.length; i++) {
            encoderDifferences[i] = wheelMotors[i].getCurrentPosition()-lastEncoderPositions[i];
        }
        DistanceMeasure turnDistance = new DistanceMeasure((encoderDifferences[0]+encoderDifferences[1]+encoderDifferences[2])/3*inchesPerTick, DistanceUnit.INCH);

        Orientation o = encoderMotion.getOrientation();
        o.thirdAngle+=(turnDistance.div(turnRadius)*180/Math.PI);
        o.thirdAngle = (float) Math2.to360(o.thirdAngle);
        encoderMotion.setOrientation(o);
        
        updateEncoderMotion(prevHeading, getEncoderHeading());
	}
	
    @Override
    public void updateEncoderMotion(Angle prevHeading, Angle heading){
	    //paranoia.  This probably doesn't need to be overridden
        //if the angles are equal default to normal driving encoder calculations
        if(Angle.abs(Angle.subtract(heading, prevHeading)).getDegree() == 0) {
            updateEncoderMotion(heading);
            return;
        }
        super.updateEncoderMotion(prevHeading, heading);
    }

    @Override
    public Angle getWheelAngleDiff(){
	    return new Angle(-120);
    }

}
