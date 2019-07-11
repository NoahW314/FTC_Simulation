package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad;

import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadMotion;

public class GamepadMotionNotDetectedEvent extends GamepadMotionEvent {
    public GamepadMotionNotDetectedEvent(GamepadMotion motionControl) {
        super(motionControl, 0f);
    }
    public GamepadMotionNotDetectedEvent(GamepadMotion motionControl, GamepadNumber number) {
        super(motionControl, 0f, number);
    }
}
