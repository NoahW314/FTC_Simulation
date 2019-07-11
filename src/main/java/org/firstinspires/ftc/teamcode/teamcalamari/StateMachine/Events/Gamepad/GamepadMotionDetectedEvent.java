package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad;

import org.firstinspires.ftc.teamcode.teamcalamari.Range;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadMotion;

public class GamepadMotionDetectedEvent extends GamepadMotionEvent {
    public GamepadMotionDetectedEvent(GamepadMotion motionControl) {
        super(motionControl, getMotionDetectedRange());
    }
    public GamepadMotionDetectedEvent(GamepadMotion motionControl, GamepadNumber number) {
        super(motionControl, getMotionDetectedRange(), number);
    }
    
    private static Range<Float> getMotionDetectedRange(){
    	Range<Float> r = new Range<Float>(-1f, 1f);
    	r.addExclusion(0f);
    	return r;
    }
}
