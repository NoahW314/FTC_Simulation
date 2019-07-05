package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors;

import org.firstinspires.ftc.teamcode.teamcalamari.SensorDataModes;

import com.qualcomm.robotcore.hardware.TouchSensor;

/**class for touch sensors, aka buttons
 * @see com.qualcomm.robotcore.hardware.TouchSensor TouchSensor*/
public class TouchSensorTC extends SensorTemplate<TouchSensor, Double, SensorDataModes> {

	/**the constructor<br>
	 * sets the default argument to NORMALIZED
	 * @see SensorTemplate#SensorTemplate(Object) SensorTemplate.SensorTemplate(sensor)*/
	public TouchSensorTC(TouchSensor sensor) {
		super(sensor);
		defaultArgument = SensorDataModes.NORMALIZED;
	}

	/**Returns the data from the sensor<br>
	 * If the parameter is RAW, {@link com.qualcomm.robotcore.hardware.TouchSensor#getValue() TouchSensor.getValue()} is returned<br>
	 * If the parameter is NORMALIZED, {@link com.qualcomm.robotcore.hardware.TouchSensor#isPressed() TouchSensor.isPressed()} is returned<br>
	 * The double values of 1 and 0 correspond to the boolean values of true and false respectively<br>
	 * For some touch sensors both mode will return the same data*/
	@Override
	public Double getData(SensorDataModes mode) {
		switch(mode) {
			case RAW:
			case NORMALIZED:
				return data;
			default:
				throw new IllegalArgumentException("The sensor data mode '"+mode+"' is not supported by the TouchSensor.getData method");
		}
	}

}
