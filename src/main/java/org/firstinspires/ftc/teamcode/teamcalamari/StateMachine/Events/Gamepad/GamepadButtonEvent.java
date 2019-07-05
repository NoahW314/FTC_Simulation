package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Event;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadButton;

public class GamepadButtonEvent extends GamepadEvent<Boolean> {

    public GamepadButtonEvent(Gamepad gamepad, GamepadButton button) {
        super(gamepad, button);
    }

    @Override
    public boolean triggers(Event e){
        if(!(e instanceof GamepadButtonEvent))return false;
        GamepadButtonEvent ge = (GamepadButtonEvent) e;
        return this.control.equals(ge.control) && this.getValue().equals(ge.getValue());
    }

    @Override
    public GamepadButtonEvent copy() {
        return new GamepadButtonEvent(gamepad, (GamepadButton)control);
    }
}
