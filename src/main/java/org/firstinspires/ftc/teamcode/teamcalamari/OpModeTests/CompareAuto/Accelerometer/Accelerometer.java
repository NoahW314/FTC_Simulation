package org.firstinspires.ftc.teamcode.teamcalamari.OpModeTests.CompareAuto.Accelerometer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Angle;

import com.qualcomm.hardware.bosch.BNO055IMU.AccelerationIntegrator;

public class Accelerometer {
	/**Angle measured counterclockwise from robot axes to accelerometer axes*/
	public static final Angle angleDifference = new Angle(0);//TODO: change to correct offset
	//23, 12
	
	public static Position correctPosition(Position pose) {
		Angle alpha = Angle.add(angleDifference, new Angle(Math.atan2(pose.y, pose.x), AngleUnit.RADIANS));
		double distance = Math.sqrt(Math.pow(pose.x, 2)+Math.pow(pose.y, 2));
		Position newPose = new Position(pose.unit, 0,0,0, pose.acquisitionTime);
		newPose.x = distance*Math.cos(alpha.getRadian());
		newPose.y = distance*Math.sin(alpha.getRadian());
		newPose.z = pose.z;
		return newPose;
	}
	
	public static AccelerationIntegrator newIntegratorAlgorithm() {
		return null;//TODO: change to best filter with best parameters
	}
}
