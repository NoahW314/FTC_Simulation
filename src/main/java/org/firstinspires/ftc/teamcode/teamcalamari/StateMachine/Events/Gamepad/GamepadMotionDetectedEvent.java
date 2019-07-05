package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad;

import org.firstinspires.ftc.teamcode.teamcalamari.Range;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Event;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadMotion;

public class GamepadMotionDetectedEvent extends GamepadMotionEvent {
    public GamepadMotionDetectedEvent(GamepadMotion motionControl) {
        super(motionControl, new Range<>(-1f, 1f));
    }

    @Override
    public boolean triggers(Event e){
        if(!(e instanceof GamepadMotionEvent))return false;
        GamepadMotionEvent ge = (GamepadMotionEvent) e;
        return this.control.equals(ge.control) && range.contains(ge.getValue()) && ge.getValue() != 0;
    }
}
