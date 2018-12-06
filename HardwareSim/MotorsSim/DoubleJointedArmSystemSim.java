package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim;

import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.GamepadSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.HardwareMapSim;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadButton;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadJoystick;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

public class DoubleJointedArmSystemSim {
    public ArmMotorSim elbowMotor;
    public ArmMotorSim shoulderMotor;

    private GamepadSim gamepad;

    public boolean manualOverrideShoulder = false;
    public boolean manualOverrideElbow = false;

    private boolean prevButtonMOS = false;
    private boolean prevButtonMOE = false;
    private boolean prevButtonMO = false;

    public DoubleJointedArmSystemSim(ArmMotorSim shoulder, ArmMotorSim elbow){
        elbowMotor = elbow;
        shoulderMotor = shoulder;
    }
    public DoubleJointedArmSystemSim(String shoulderName, String elbowName, HardwareMapSim hwMap){
        elbowMotor = new ArmMotorSim(elbowName, hwMap);
        shoulderMotor = new ArmMotorSim(shoulderName, hwMap);
    }


    public void setMode(RunMode mode){
        elbowMotor.setMode(mode);
        shoulderMotor.setMode(mode);
    }
    public void setGamepad(GamepadSim gamepad){
        this.gamepad = gamepad;
        elbowMotor.setGamepad(gamepad);
        shoulderMotor.setGamepad(gamepad);
    }
    public void stop(){
        elbowMotor.stop();
        shoulderMotor.stop();
    }

    public void runToPosition(GamepadButton button, int[] ticks, double[] power, boolean manualControlShoulder, boolean manualControlElbow) {
        if (button.isNone()) return;

        if (button.getValue(gamepad)) {
        	System.out.println("");
        	System.out.println(button);
        	System.out.println(ticks[0]+" "+ticks[1]);
        	System.out.println(manualControlShoulder+" "+manualControlElbow);
        	System.out.println("");
            
        	manualControlShoulder = false;
        	manualControlElbow = false;
        	
            elbowMotor.setMode(RunMode.RUN_TO_POSITION);
            elbowMotor.setTargetPosition(ticks[1]);
            shoulderMotor.setMode(RunMode.RUN_TO_POSITION);
            shoulderMotor.setTargetPosition(ticks[0]);
        }
        
        if (elbowMotor.getMode() == RunMode.RUN_TO_POSITION && elbowMotor.getTargetPosition() == ticks[1] && !manualControlElbow) {
            elbowMotor.resetPower(power[0]);
        }
        if(shoulderMotor.getMode() == RunMode.RUN_TO_POSITION && shoulderMotor.getTargetPosition() == ticks[0] && !manualControlShoulder){
            shoulderMotor.resetPower(power[1]);
        }
    }

    public void manualOverride(GamepadJoystick shoulderOverride, GamepadJoystick elbowOverride){
        if(manualOverrideShoulder){
            if(!shoulderOverride.isNone()) {
                shoulderMotor.setMode(RunMode.RUN_USING_ENCODER);
                shoulderMotor.resetPower(shoulderOverride.getValue(gamepad));
            }
        }
        if(manualOverrideElbow){
            if(!elbowOverride.isNone()){
                elbowMotor.setMode(RunMode.RUN_USING_ENCODER);
                elbowMotor.resetPower(elbowOverride.getValue(gamepad));
            }
        }
    }
    public void setManualOverrideShoulder(GamepadButton button){
        if(button.isNone()) return;
        boolean buttonValue = button.getValue(gamepad);
        if(buttonValue && !prevButtonMOS){
            manualOverrideShoulder = !manualOverrideShoulder;
        }
        prevButtonMOS = buttonValue;
    }
    public void setManualOverrideElbow(GamepadButton button){
        if(button.isNone()) return;
        boolean buttonValue = button.getValue(gamepad);
        if(buttonValue && !prevButtonMOE){
            manualOverrideElbow = !manualOverrideElbow;
        }
        prevButtonMOE = buttonValue;
    }
    public void setManualOverride(GamepadButton button){
        if(button.isNone()) return;
        boolean buttonValue = button.getValue(gamepad);
        if(buttonValue && !prevButtonMO){
            manualOverrideShoulder = !manualOverrideShoulder;
            manualOverrideElbow = !manualOverrideElbow;
        }
        prevButtonMO = buttonValue;
    }

    public void run(){
        elbowMotor.run();
        shoulderMotor.run();
    }

}
