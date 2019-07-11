package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Sensor;

import org.firstinspires.ftc.teamcode.teamcalamari.Range;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Event;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors.SensorTemplateSimple;

public abstract class SensorEvent<S extends SensorTemplateSimple<?,  T>, T extends Comparable<T>> extends Event<T> {

    protected S sensor;
    protected T value;

    public SensorEvent(String name, S sensor){
        super(name, Mode.PRODUCING);
        this.sensor = sensor;
    }
    public SensorEvent(String name, T value){
        super(name, Mode.SINGLE_CHECK);
        this.value = value;
    }
    public SensorEvent(String name, Range<T> range){
        super(name, range);
    }
    
    public boolean rangeCheck(Event<?> e) {
    	return range.contains(((SensorEvent<S, T>)e).getValue());
    }

    @Override
    public T getValue() {
        if(value != null) return value;
        return sensor.getData();
    }
}
