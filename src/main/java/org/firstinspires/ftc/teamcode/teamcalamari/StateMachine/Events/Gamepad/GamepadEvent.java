package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Event;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadControl;


public abstract class GamepadEvent<T> extends Event<T> {
    protected Gamepad gamepad = new Gamepad();
    protected GamepadControl<T> control;
    public GamepadEvent(Gamepad gamepad, GamepadControl<T> control) {
        super("Gamepad "+control.getString());
        try {
            this.gamepad.copy(gamepad);
        } catch(RobotCoreException e){
            e.printStackTrace();
        }
        this.control = control;
    }
    public T getValue(){
        return control.getValue(gamepad);
    }
    public GamepadControl<T> getControl() { return control; }
}
