package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Sensor;

import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Event;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors.TouchSensorSimpleTC;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class TouchSensorEvent extends SensorEvent<TouchSensorSimpleTC, Boolean> {

    public TouchSensorEvent(String name, TouchSensorSimpleTC sensor){
        super(name, sensor);
    }
    public TouchSensorEvent(String name, TouchSensor sensor){
        this(name, new TouchSensorSimpleTC(sensor));
    }
    public TouchSensorEvent(String name, Boolean value){
        super(name, value);
    }

    @Override
    public Boolean getValue() {
        if(mode == Mode.PRODUCING) return sensor.getData();
        if(mode == Mode.SINGLE_CHECK) return value;
        throw new IllegalStateException("Touch Sensor Events do not have a RANGE_CHECK mode!");
    }

    @Override
    public TouchSensorEvent copy() {
        if(sensor != null) return new TouchSensorEvent(name, sensor);
        return new TouchSensorEvent(name, value);
    }
    
	@Override
	public boolean isSameClass(Event<?> e) {
		return e instanceof TouchSensorEvent;
	}
	@Override
	public boolean rangeCheck(Event<?> e) {
		throw new IllegalStateException("Touch Sensor Events do not have a RANGE_CHECK mode!");
	}
}
