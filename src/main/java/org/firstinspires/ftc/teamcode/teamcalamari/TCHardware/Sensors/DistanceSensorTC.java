package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.DistanceMeasure;

import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * Created by Tavis on 2/3/2018.
 */
/**
 * class for distance sensors, like the REV color/distance sensor and the MR range sensor
 * @see com.qualcomm.robotcore.hardware.DistanceSensor DistanceSensor
*/
public class DistanceSensorTC extends SensorTemplate<DistanceSensor, DistanceMeasure, DistanceUnit> {

	/**The constructor.<br>
	Sets the default argument to the inch.
	@see SensorTemplate#SensorTemplate(Object)*/
    public DistanceSensorTC(DistanceSensor distance){
        super(distance);
        defaultArgument = DistanceUnit.INCH;
    }

	/**gets the distance from the sensor
	 * @param unit the distance unit in which the distance will be returned*/
    @Override
    public DistanceMeasure getData (DistanceUnit unit){
        return data.createWithUnit(unit);
    }
    
    public DistanceMeasure getHeadingCorrectedDistance(DistanceUnit unit, int axis, Angle heading) {
    	switch(axis) {
	    	case 0:
	    		return data.createWithUnit(unit).mult(Math.cos(heading.getRadian()));
	    	case 1:
	    		return data.createWithUnit(unit).mult(Math.sin(heading.getRadian()));
	    	default: throw new IllegalArgumentException("The only acceptable axis integers in "
	    			+ "DistanceSensorTC.getHeadingCorrectedDistance are 0(x) and 1(y)");
    	}
    }
}