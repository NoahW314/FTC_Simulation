package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad;

import org.firstinspires.ftc.teamcode.teamcalamari.Range;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Event;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadControl;

import com.qualcomm.robotcore.hardware.Gamepad;


public abstract class GamepadEvent<T extends Comparable<T>, C extends GamepadControl<T>> extends Event<T> {
    protected C control;
    protected T value;
    protected GamepadNumber gamepad = GamepadNumber.ANY;
    
    public GamepadEvent(Gamepad gamepad, C control) {
        this(gamepad, control, GamepadNumber.ANY);
    }
    public GamepadEvent(C control, T value) {
    	this(control, value, GamepadNumber.ANY);
    }
    public GamepadEvent(C control, Range<T> range) {
    	this(control, range, GamepadNumber.ANY);
    }
    
    public GamepadEvent(Gamepad gamepad, C control, GamepadNumber number) {
        super("Gamepad "+control.getString(), Mode.PRODUCING);
        value = control.getValue(gamepad);
        this.control = control;
        this.gamepad = number;
    }
    public GamepadEvent(C control, T value, GamepadNumber number) {
    	super("Gamepad "+control.getString(), Mode.SINGLE_CHECK);
    	this.value = value;
    	this.control = control;
    	this.gamepad = number;
    }
    public GamepadEvent(C control, Range<T> range, GamepadNumber number) {
    	super("Gamepad "+control.getString(), range);
    	this.control = control;
    	this.gamepad = number;
    }
    
    public T getValue(){
    	return value;
    }
    public GamepadControl<T> getControl() { return control; }
    public GamepadNumber getGamepad() {return gamepad;}
    
    @Override
    public boolean otherChecks(Event<?> e) {
    	return control.equals(((GamepadEvent<?, ?>)e).getControl()) && gamepad == ((GamepadEvent<?, ?>)e).getGamepad();
    }
    @Override
    public boolean rangeCheck(Event<?> e) {
    	return range.contains(((GamepadEvent<T, ?>)e).getValue());
    }
    
    /**G1 is gamepad1, G2 is gamepad2, and ANY is either gamepad*/
    public static enum GamepadNumber{G1, G2, ANY};
}
