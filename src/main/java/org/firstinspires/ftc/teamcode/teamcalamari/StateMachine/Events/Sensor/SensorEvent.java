package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Sensor;

import android.util.Log;

import org.firstinspires.ftc.teamcode.teamcalamari.Range;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Event;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors.SensorTemplateSimple;

public class SensorEvent<S extends SensorTemplateSimple<?,  T>, T extends Comparable<T>> extends Event<T> {

    protected S sensor;
    protected T value;
    protected Range<T> range;
    protected Mode mode;

    public SensorEvent(String name, S sensor){
        super(name);
        this.sensor = sensor;
        mode = Mode.PRODUCING;
    }
    public SensorEvent(String name, T value){
        super(name);
        this.value = value;
        mode = Mode.SINGLE_CHECK;
    }
    public SensorEvent(String name, Range<T> range){
        super(name);
        this.range = range;
        mode = Mode.RANGE_CHECK;
    }

    @Override
    public T getValue() {
        if(value != null) return value;
        return sensor.getData();
    }

    @Override
    public boolean triggers(Event e){
        if(!(e instanceof SensorEvent)) return false;
        SensorEvent se = (SensorEvent) e;
        if(se.mode != Mode.PRODUCING) return false;
        switch(mode){
            case PRODUCING: throw new IllegalStateException("This event produces values.  It does not check them.");
            case SINGLE_CHECK:
                return !value.getClass().isInstance(se.getValue()) && super.triggers(e);
            case RANGE_CHECK: return isInRange(se);
            default: throw new IllegalStateException("The Sensor Event Mode "+mode+" is not supported by "+this.getClass());
        }
    }

    public boolean isInRange(SensorEvent e){
        throw new IllegalStateException("This method should have been overridden by the implementing class");
    }


    @Override
    public Event<T> copy() {
        switch(mode){
            case PRODUCING: return new SensorEvent<>(name, sensor);
            case SINGLE_CHECK: return new SensorEvent<>(name, value);
            case RANGE_CHECK: return new SensorEvent<>(name, range);
            default: throw new IllegalStateException("The Sensor Event Mode "+mode+" is not supported by "+this.getClass());
        }
    }

    public enum Mode{
        /**This event contains a reference to sensor which produces a value.
         * This value will then be checked against another event for a match.*/
        PRODUCING, //sensor
        /**This event contains a single value and will be given an event whose value will be checked against its.*/
        SINGLE_CHECK, //value
        /**This event contains a range of values and will be given an event whose value will be checked to see if it in the range.*/
        RANGE_CHECK, //range
    }
}
