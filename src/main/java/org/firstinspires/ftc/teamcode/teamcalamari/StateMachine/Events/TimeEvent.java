package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events;

public class TimeEvent extends Event<Double> {
    private double seconds;

    public TimeEvent(double seconds){
        super("TimeEvent");
        this.seconds = seconds;
    }

    @Override
    public Double getValue() {
        return seconds;
    }
    @Override
    public TimeEvent copy(){
        return new TimeEvent(seconds);
    }
    @Override
    public boolean triggers(Event e){
        //triggered if the given time is greater than the needed time
        return (e instanceof TimeEvent) && getValue() >= ((TimeEvent)e).getValue();
    }
}
