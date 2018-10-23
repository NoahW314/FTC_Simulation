package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.SensorsSim;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;

import com.qualcomm.robotcore.hardware.DistanceSensor;

public class DistanceSensorSimTC {
	public double distance;
	public DistanceSensorSimTC(DistanceSensor sensor) {}
	public double getData(DistanceUnit unit) {
		return distance;
	}
	public double getData() {
		return this.getData(DistanceUnit.INCH);
	}
	
	public Double getHeadingCorrectedDistance(DistanceUnit unit, int axis, Angle heading) {
    	switch(axis) {
    	case 0:
    		return this.getData(unit)*Math.cos(heading.getRadian());
    	case 1:
    		return this.getData(unit)*Math.sin(heading.getRadian());
    	default: throw new IllegalArgumentException("The only acceptable axis integers in "
    			+ "DistanceSensorTC.getHeadingCorrectedDistance are 0(x) and 1(y)");
    	}
    }
}
