package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events;

import org.firstinspires.ftc.teamcode.teamcalamari.Range;

public class TimeEvent extends Event<Double> {
    private double seconds;

    public TimeEvent(String name, double seconds){
        super(name, Mode.PRODUCING);
        this.seconds = seconds;
    }
    public TimeEvent(String name, Range<Double> range) {
    	super(name, range);
    }
    public TimeEvent(double seconds){
        this("TimeEvent", seconds);
    }
    public TimeEvent(Range<Double> range) {
    	this("TimeEvent", range);
    }

    @Override
    public Double getValue() {
        return seconds;
    }
    @Override
    public TimeEvent copy(){
    	switch(mode) {
    		case PRODUCING: return new TimeEvent(name, seconds);
    		case RANGE_CHECK: return new TimeEvent(name, range);
			default: throw new IllegalArgumentException("Invalid mode "+mode+" for a time event!");
    	}
    }
    @Override
    public boolean singleCheck(Event<?> e) {
    	throw new IllegalStateException("Time events do not have a SINGLE_CHECK mode!");
    }
	@Override
	public boolean rangeCheck(Event<?> e) {
		//cast is safe since isSameClass is always called before this method
		return this.range.contains(((TimeEvent)e).getValue());
	}
	@Override
	public boolean isSameClass(Event<?> e) {
		return e instanceof TimeEvent;
	}
}
