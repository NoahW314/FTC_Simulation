package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors;

import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareMap;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadJoystick;

public class ArmMotor extends Motor{
    public ArmMotor(Motor motor) { super(motor); }

    public ArmMotor(String motorName, HardwareMap hwMap) { super(motorName, hwMap); }

    public void correctedSetPower(GamepadJoystick joystick, double powerCorrection, int[] correctionRange){
        if(joystick.isNone()) return;
        
        System.out.println(getCurrentPosition()+" in "+correctionRange[0]+" - "+correctionRange[1]);
        
        if(getCurrentPosition() >= correctionRange[0] && getCurrentPosition() <= correctionRange[1]
        	&& (getMode() != RunMode.RUN_TO_POSITION 
        	|| getTargetPosition() >= correctionRange[0] && getTargetPosition() <= correctionRange[1])) {
        	
        		setMode(RunMode.RUN_USING_ENCODER);
            	setPower(joystick.getValue(gamepad) + powerCorrection);
        }
    }
}