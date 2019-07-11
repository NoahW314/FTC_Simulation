package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad;

import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Event;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadButton;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadControl;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadButtonClickedEvent extends GamepadButtonEvent {

	protected Boolean prevValue = false;
	protected Boolean onDown;
	
	public GamepadButtonClickedEvent(Gamepad gamepad, GamepadButton button) {
		super(gamepad, button);
	}
	public GamepadButtonClickedEvent(GamepadButton button, Boolean onDown) {
		super(button, null);
		this.onDown = onDown;
	}
	
	public GamepadButtonClickedEvent(Gamepad gamepad, GamepadButton button, GamepadNumber number) {
		super(gamepad, button, number);
	}
	public GamepadButtonClickedEvent(GamepadButton button, Boolean onDown, GamepadNumber number) {
		super(button, null, number);
		this.onDown = onDown;
	}
	
	
	@Override
	public boolean singleCheck(Event<?> e) {
		boolean value = ((GamepadButtonEvent)e).getValue();
		return !prevValue.equals(value) && onDown.equals(value);
	}
	
	@Override
    public GamepadButtonClickedEvent copy() {
    	switch(mode) {
    		case PRODUCING: return new GamepadButtonClickedEvent(GamepadControl.setValue(control, value), control, gamepad);
    		case SINGLE_CHECK: return new GamepadButtonClickedEvent(control, onDown, gamepad);
        	default: throw new IllegalStateException("Invalid Mode "+mode+" for a GamepadButtonEvent!");
    	}
    }
	
	public void assignPrevValue(Gamepad gamepad) {
		prevValue = control.getValue(gamepad);
	}
}
