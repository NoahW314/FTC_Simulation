package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad;

import java.lang.reflect.Field;

import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadEvent.GamepadNumber;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadMotionDetectedEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadMotionEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadMotionNotDetectedEvent;

import com.qualcomm.robotcore.hardware.Gamepad;

public enum GamepadTrigger implements GamepadMotion{
    NONE(""),
    LEFT_TRIGGER("left_trigger"),
    RIGHT_TRIGGER("right_trigger");


    String str;
    GamepadTrigger(String str){ this.str = str; }

    @Override
    public String getString() { return str; }

    @Override
    public Float getValue(Gamepad gamepad){
        if(getString().equals("")){throw new IllegalArgumentException("Can't get the value of NONE on a gamepad");}
        try {
            Field field = Gamepad.class.getField(getString());
            return field.getFloat(gamepad);
        } catch(ReflectiveOperationException e){
            throw new IllegalStateException("The trigger "+getString()+" doesn't exist");
        }
    }

    @Override
    public boolean isNone(){
        return getString().equals("");
    }

    @Override
    public GamepadMotionEvent getEvent(GamepadNumber number){
        return new GamepadMotionEvent(this, number);
    }
    @Override
    public GamepadMotionDetectedEvent getDetectedEvent(GamepadNumber number){
        return new GamepadMotionDetectedEvent(this, number);
    }
    @Override
    public GamepadMotionNotDetectedEvent getNotDetectedEvent(GamepadNumber number){
        return new GamepadMotionNotDetectedEvent(this, number);
    }
}
