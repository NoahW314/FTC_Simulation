package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad;




import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teamcalamari.Range;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Event;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadMotion;

public class GamepadMotionEvent extends GamepadEvent<Float> {

    protected Range<Float> range;

    public GamepadMotionEvent(GamepadMotion motionControl, Range<Float> range) {
        super(new Gamepad(), motionControl);
        this.range = range;
    }
    public GamepadMotionEvent(GamepadMotion motionControl){
        this(motionControl, new Range<>(-1f, 1f));
    }
    public GamepadMotionEvent(Gamepad gamepad, GamepadMotion motionControl){
        super(gamepad, motionControl);
    }

    @Override
    public boolean triggers(Event e){
        if(!(e instanceof GamepadMotionEvent))return false;
        GamepadMotionEvent ge = (GamepadMotionEvent) e;
        return this.control.equals(ge.control) && range.contains(ge.getValue());
    }

    @Override
    public GamepadMotionEvent copy() {
        return new GamepadMotionEvent((GamepadMotion)control, range);
    }
}
