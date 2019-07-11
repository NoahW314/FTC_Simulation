package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad;

import org.firstinspires.ftc.teamcode.teamcalamari.Range;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Event;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadControl;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadMotion;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadMotionEvent extends GamepadEvent<Float, GamepadMotion> {

	/**RANGE_CHECK*/
    public GamepadMotionEvent(GamepadMotion motionControl, Range<Float> range) {
        super(motionControl, range);
    }
    /**RANGE_CHECK*/
    public GamepadMotionEvent(GamepadMotion motionControl){
        this(motionControl, new Range<Float>(-1f, 1f));
    }
    /**SINGLE_CHECK*/
    public GamepadMotionEvent(GamepadMotion motionControl, Float value) {
    	super(motionControl, value);
    }
    /**PRODUCING*/
    public GamepadMotionEvent(Gamepad gamepad, GamepadMotion motionControl){
        super(gamepad, motionControl);
    }
    
    /**RANGE_CHECK*/
    public GamepadMotionEvent(GamepadMotion motionControl, Range<Float> range, GamepadNumber number) {
        super(motionControl, range, number);
    }
    /**RANGE_CHECK*/
    public GamepadMotionEvent(GamepadMotion motionControl, GamepadNumber number){
        this(motionControl, new Range<Float>(-1f, 1f), number);
    }
    /**SINGLE_CHECK*/
    public GamepadMotionEvent(GamepadMotion motionControl, Float value, GamepadNumber number) {
    	super(motionControl, value, number);
    }
    /**PRODUCING*/
    public GamepadMotionEvent(Gamepad gamepad, GamepadMotion motionControl, GamepadNumber number){
        super(gamepad, motionControl, number);
    }

    @Override
    public GamepadMotionEvent copy() {
    	switch(mode) {
    		case PRODUCING: return new GamepadMotionEvent(GamepadControl.setValue(control, value), control, gamepad); 
    		case SINGLE_CHECK: return new GamepadMotionEvent(control, value, gamepad);
    		case RANGE_CHECK: return new GamepadMotionEvent(control, range, gamepad);
    		default: throw new IllegalStateException("Invalid Mode "+mode+" for a Gamepad Motion Event!"); 
    	}
    }
	@Override
	public boolean isSameClass(Event<?> e) {
		return e instanceof GamepadMotionEvent;
	}
	@Override
	public boolean rangeCheck(Event<?> e) {
		return range.contains(((GamepadMotionEvent)e).getValue());
	}
}
