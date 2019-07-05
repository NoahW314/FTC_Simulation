package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events;


public abstract class Event<ReturnType> {
    public String name;

    public Event(String name){
        this.name = name;
    }

    public abstract ReturnType getValue();
    /**In general, the copy of an event should have the same name and return the same value.
     * In addition, <code>event.equals(event.copy())</code> should always be true*/
    public abstract Event<ReturnType> copy();

    public String toString(){return name;}
    /**@return if this event triggers the event {@code e}*/
    public boolean triggers(Event e) {
        return this.name.equals(e.name) && this.getValue().equals(e.getValue());
    }
    public void onTriggered(){}
}
