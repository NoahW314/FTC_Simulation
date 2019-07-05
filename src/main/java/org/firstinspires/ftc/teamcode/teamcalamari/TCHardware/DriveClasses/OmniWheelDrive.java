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

public class OmniWheelDrive extends AllDirectionsDrive {

    //the constructors
    /**The wheel motors should be listed clockwise.
    A line drawn from the center of the robot to the first wheel motor listed indicates the positive x-axis.
    headOffset is measured counterclockwise from the positive x-axis.*/
    public OmniWheelDrive(Motor[] wheels, Angle headOffset, OpModeType type){
    	super(wheels, headOffset, type);
    }
    /**The wheel motors should be listed clockwise.
    A line drawn from the center of the robot to the first wheel motor listed indicates the positive x-axis.
    headOffset is measured counterclockwise from the positive x-axis.*/
    public OmniWheelDrive(DcMotor[] wheels, Angle headOffset, OpModeType type){
    	super(wheels, headOffset, type);
    }

    public OmniWheelDrive(HardwareMap map, Angle headOffset, OpModeType type, String... motorNames){
        super(map, headOffset, type, motorNames);
    }

    public Angle driveAngle = null;
    public Angle getDriveAngle(){return driveAngle.copy();}

    /**Drives the robot at a set angle, relative to the robot user axes*/
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
        double[] f = new double[]{py, px, -py, -px};

        //add the calculated powers to the current wheel powers
        for(int i = 0; i < 4; i++) {
            wheelMotors[i].setPower(f[i]);
        }
    }
	@Override
	public void updateEncoderMotion() {
		int[] encoderDifferences = new int[wheelMotors.length];
		for(int i = 0; i < wheelMotors.length; i++) {
			encoderDifferences[i] = wheelMotors[i].getCurrentPosition()-lastEncoderPositions[i];
		}
		double turnDistance = (encoderDifferences[0]+encoderDifferences[2])/2*inchesPerTick;

		Orientation o = encoderMotion.getOrientation();
		o.thirdAngle+=(turnDistance/turnRadius.value*180/Math.PI);
		o.thirdAngle = (float) Math2.to360(o.thirdAngle);
		encoderMotion.setOrientation(o);

		updateEncoderMotion(new Angle(o.thirdAngle, AngleUnit.DEGREES));
	}
	@Override
	public void updateEncoderMotion(Angle heading) {
		int[] encoderDifferences = new int[4];
		for(int i = 0; i < 4; i++) {
			encoderDifferences[i] = wheelMotors[i].getCurrentPosition()-lastEncoderPositions[i];
		}
		double driveX = (encoderDifferences[1]-encoderDifferences[3])/2*inchesPerTick;
		double driveY = (encoderDifferences[0]-encoderDifferences[2])/2*inchesPerTick;
		double driveDistance = Math.sqrt(Math.pow(driveX, 2)+Math.pow(driveY, 2));
		double theta = heading.getRadian()+Math.atan2(driveY, driveX);

		Position pose = encoderMotion.getPosition();
		pose.x+=driveDistance*Math.cos(theta);
		pose.y+=driveDistance*Math.sin(theta);
		encoderMotion.setPosition(pose);
	}
	@Override
    public void updateEncoderMotion(Angle prevHeading, Angle heading){
        //if the angles are equal default to normal driving encoder calculations
        if(Angle.abs(Angle.subtract(heading, prevHeading)).getDegree() == 0) {
            updateEncoderMotion(heading);
            return;
        }

        int[] encoderDifferences = new int[3];
        for(int i = 0; i < 3; i++) {
            encoderDifferences[i] = wheelMotors[i].getCurrentPosition()-lastEncoderPositions[i];
        }
        Angle driveAngle = getDriveAngle();
        DistanceMeasure arcLength = calculateArcLength(encoderDifferences[0], encoderDifferences[1], encoderDifferences[2],
                                                    driveAngle, getWheelAngleDiff());
        Position poseDiff = getPoseDiff(arcLength, prevHeading, heading);
        Position startingPose = encoderMotion.getPosition();
        encoderMotion.setPosition(startingPose.added(poseDiff));
    }

    public Angle getWheelAngleDiff(){
        return new Angle(-90);
    }

    public Position getPoseDiff(DistanceMeasure arcLength, Angle prevHeading, Angle heading){
        return new Position(
                arcLength.value*(Math.sin(heading.getRadian())-Math.sin(prevHeading.getRadian()))
                            /Angle.subtract(heading, prevHeading).getRadian(),
                arcLength.value*(Math.cos(prevHeading.getRadian())-Math.cos(heading.getRadian()))
                        /Angle.subtract(heading, prevHeading).getRadian(),
                0, arcLength.unit);
    }
    public DistanceMeasure calculateArcLength(int encoderTicks1, int encoderTicks2, int encoderTicks3,
                                              Angle driveAngle, Angle wheelAngleDiff){
        Angle driveAngleAdjusted = Angle.add(driveAngle, headAngle);
        int ticks1 = encoderTicks1;
        int ticks2 = encoderTicks2;
        Angle theta = Angle.subtract(driveAngleAdjusted, new Angle(0));
        Angle phi = Angle.subtract(driveAngleAdjusted, wheelAngleDiff);
        //if a2 doesn't work use a3
        if(Math.abs(Math.sin(theta.getRadian())-Math.sin(phi.getRadian())) < 0.1){
            ticks2 = encoderTicks3;
            phi = Angle.subtract(driveAngleAdjusted, Angle.mult(wheelAngleDiff, 2));
        }
        //paranoia
        if(Math.abs(Math.sin(theta.getRadian())-Math.sin(phi.getRadian())) < 0.1){
            ticks1 = encoderTicks2;
            theta = Angle.subtract(driveAngleAdjusted, wheelAngleDiff);
        }

        double left = Math.pow(ticks2, 2)*Math.sin(theta.getRadian())-Math.pow(ticks1, 2)*Math.sin(phi.getRadian());
        double inside = Math.pow(left, 2)-Math.pow(Math.pow(ticks1, 2)-Math.pow(ticks2, 2), 2);
        double bottom = 2*(Math.sin(theta.getRadian())-Math.sin(phi.getRadian()));
        double radiusP = Math.sqrt((left+Math.sqrt(inside))/bottom);
        double radiusN = Math.sqrt((left-Math.sqrt(inside))/bottom);
        /*radiusP vs radiusN depends on the sign of bottom.
        It seems like the larger value is always the correct one though.*/
        return new DistanceMeasure(Math.max(radiusP, radiusN)*inchesPerTick, DistanceUnit.INCH);
    }

}
