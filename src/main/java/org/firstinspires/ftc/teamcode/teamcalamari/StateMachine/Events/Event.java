package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events;

import org.firstinspires.ftc.teamcode.teamcalamari.Range;

public abstract class Event<T extends Comparable<T>> {
    public String name;
    public Mode mode;
    protected Range<T> range;

    public Event(String name, Mode mode){
        this.name = name;
        this.mode = mode;
    }
    public Event(String name, Range<T> range) {
    	this(name, Mode.RANGE_CHECK);
    	this.range = range;
    }

    public abstract T getValue();
    /**In general, the copy of an event should have the same name and return the same value.
     * In addition, <code>event.equals(event.copy())</code> should always be true*/
    public abstract Event<T> copy();

    public String toString(){return name;}
    /**check this event against the event {@code e}.  This event should be a checker, either single or range.
    {@code e} should be a producer.*/
    public boolean check(Event<?> e) {
    	if(e.mode != Mode.PRODUCING) throw new IllegalArgumentException("The given event does not produce values; it checks them!");
    	if(mode == Mode.PRODUCING) throw new IllegalStateException("This event produces values; it does not check them!");
    	if(!this.name.equals(e.name)) return false;
    	if(!isSameClass(e)) return false;
    	if(!otherChecks(e)) return false;
    	switch(mode) {
    		case SINGLE_CHECK: return singleCheck(e);
    		case RANGE_CHECK: return rangeCheck(e);
    		default: throw new IllegalStateException("The Mode "+mode+" is not valid for an Event's mode!"); 
    	}
    }
    public abstract boolean isSameClass(Event<?> e);
    public boolean otherChecks(Event<?> e) {return true;}
    public boolean singleCheck(Event<?> e) {
    	return this.getValue().equals(e.getValue());
    }
    /**We can't implement this method since we can't verify the value returned by e.getValue(), but if we could 
    this would be the code.<br>&emsp;&emsp;
    {@code return range.contains(e.getValue());}*/
    public abstract boolean rangeCheck(Event<?> e);
    
    public void onTriggered(){}
    
    public enum Mode{
        /**This event contains a reference to an object which produces a value.
         * This value will then be checked against another event for a match.*/
        PRODUCING, //value producing object
        /**This event contains a single value and will be given an event whose value will be checked against its.*/
        SINGLE_CHECK, //value
        /**This event contains a range of values and will be given an event whose value will be checked to see if it in the range.*/
        RANGE_CHECK, //range
    }
}
