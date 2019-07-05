package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors;

/**
Template for sensor classes.<br><br>

Use this class for sensors whose getData method requires an argument due 
to the complexity of the data.  Most sensors will extend this class.

@param <H> the hardware class of the sensor
@param <T> the type of data that the getData method returns
@param <A> the type of argument for the getData method

@see SensorTemplateSimple
 */
public abstract class SensorTemplate<H, T, A> extends SensorTemplateSimple<H, T> {
	
	/**The default argument for the getData method.
	It is used when getData is called with no arguments.
	It should be set to a default value in the constructor.
	This means that any changes to the default argument by the
	OpModes should be done <strong>AFTER</strong> the constructor is called
	 */
	public A defaultArgument;
	
	/**The constructor.<br>
	Sets the sensor variable equal to the given sensor.
	@param sensor the sensor gotten from the hardwareMap.get() method.*/
	public SensorTemplate(H sensor) {
		super(sensor);
	}

	/**return the data given by the sensor using the default argument*/
	@Override
	public T getData(){
		return this.getData(defaultArgument);
	}
	
	/**return the data given by the sensor*/
	public abstract T getData(A arg);

}
