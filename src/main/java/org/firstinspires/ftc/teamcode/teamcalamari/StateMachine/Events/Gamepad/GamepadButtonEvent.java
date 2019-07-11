package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad;

import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Event;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadButton;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadControl;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadButtonEvent extends GamepadEvent<Boolean, GamepadButton> {

    public GamepadButtonEvent(Gamepad gamepad, GamepadButton button) {
        super(gamepad, button);
    }
    public GamepadButtonEvent(GamepadButton button, Boolean value) {
    	super(button, value);
    }
    
    public GamepadButtonEvent(Gamepad gamepad, GamepadButton button, GamepadNumber number) {
        super(gamepad, button, number);
    }
    public GamepadButtonEvent(GamepadButton button, Boolean value, GamepadNumber number) {
    	super(button, value, number);
    }


    @Override
    public GamepadButtonEvent copy() {
    	switch(mode) {
    		case PRODUCING: return new GamepadButtonEvent(GamepadControl.setValue(control, value), control, gamepad);
    		case SINGLE_CHECK: return new GamepadButtonEvent(control, value, gamepad);
        	default: throw new IllegalStateException("Invalid Mode "+mode+" for a GamepadButtonEvent!");
    	}
    }
	@Override
	public boolean isSameClass(Event<?> e) {
		return e instanceof GamepadButtonEvent;
	}
	
	@Override
	public boolean singleCheck(Event<?> e) {
		return this.getValue().equals(e.getValue());
	}
	@Override
	public boolean rangeCheck(Event<?> e) {
		throw new IllegalStateException("Gamepad Button Events do not have a RANGE_CHECK mode!");
	}
}
