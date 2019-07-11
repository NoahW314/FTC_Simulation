package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Sensor;

import org.firstinspires.ftc.teamcode.teamcalamari.Range;
import org.firstinspires.ftc.teamcode.teamcalamari.Navigation.DistanceMeasure;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Event;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors.DistanceSensorTC;

public class DistanceSensorEvent extends SensorEvent<DistanceSensorTC, DistanceMeasure> {
    protected Range<DistanceMeasure> range;

    public DistanceSensorEvent(String name, DistanceSensorTC sensor) {
        super(name, sensor);
    }

    public DistanceSensorEvent(String name, Range<DistanceMeasure> range) {
        super(name, range);
    }

    @Override
    public DistanceSensorEvent copy(){
        if(mode == Mode.RANGE_CHECK) return new DistanceSensorEvent(name, range);
        if(mode == Mode.PRODUCING) return new DistanceSensorEvent(name, sensor);
        throw new IllegalStateException("Distance Sensor Events do not have a SINGLE_CHECK mode!");
    }

	@Override
	public boolean isSameClass(Event<?> e) {
		return e instanceof DistanceSensorEvent;
	}
}
