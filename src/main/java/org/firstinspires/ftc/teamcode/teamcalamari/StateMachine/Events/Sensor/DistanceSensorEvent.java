package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Sensor;

import org.firstinspires.ftc.teamcode.teamcalamari.Range;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Event;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors.DistanceSensorTC;

public class DistanceSensorEvent extends SensorEvent<DistanceSensorTC, Double> {
    protected Range<Double> range;

    public DistanceSensorEvent(String name, DistanceSensorTC sensor) {
        super(name, sensor);
    }

    public DistanceSensorEvent(String name, Range<Double> range) {
        super(name, range);
    }

    @Override
    public boolean isInRange(SensorEvent e){
        if(!(e instanceof DistanceSensorEvent)) return false;
        DistanceSensorEvent dse = (DistanceSensorEvent) e;
        return this.name.equals(e.name) && range.contains(dse.getValue());
    }

    @Override
    public Event<Double> copy(){
        if(range != null) return new DistanceSensorEvent(name, range);
        return new DistanceSensorEvent(name, sensor);
    }
}
