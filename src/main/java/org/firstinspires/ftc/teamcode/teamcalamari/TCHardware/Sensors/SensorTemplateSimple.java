package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors;

/**
Template for sensor classes.<br><br>

As there is no getData method that accepts an argument in this class,
this template should be used for sensors whose data can't vary much.
The DigitalChannel class is a good example of a sensor that uses this class.
Because its data is a single boolean, an argument is not required for the getData method.
Use SensorTemplate for sensors that need an argument.

@param <H> the hardware class of the sensor
@param <T> the type of data the getData method returns

@see SensorTemplate
 */
public abstract class SensorTemplateSimple<H, T> {
	
	/**the sensor*/
	private H sensor;
	
	/**The data the sensor will return*/
	protected T data;
	
	/**The constructor<br>
	Sets the sensor variable equal to the given sensor
	@param sensor the sensor gotten from the hardwareMap.get() method.*/
	public SensorTemplateSimple(H sensor) {
		this.sensor = sensor;
	}
	
	/**return the data given by the sensor*/
	public T getData() {
		return data;
	}
	
	public void setData(T data) {
		this.data = data;
	}
}
