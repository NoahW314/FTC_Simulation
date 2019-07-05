package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors;

import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.HardwareMap;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.Gamepad;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadButton;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadJoystick;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DoubleJointedArmSystem {
    public ArmMotor elbowMotor;
    public ArmMotor shoulderMotor;

    private Gamepad gamepad;

    public boolean manualOverrideShoulder = false;
    public boolean manualOverrideElbow = false;

    private boolean prevButtonMOS = false;
    private boolean prevButtonMOE = false;
    private boolean prevButtonMO = false;

    public ElapsedTime shoulderTimer = new ElapsedTime();
    public ElapsedTime elbowTimer = new ElapsedTime();

    public DoubleJointedArmSystem(ArmMotor shoulder, ArmMotor elbow){
        elbowMotor = elbow;
        shoulderMotor = shoulder;
    }
    public DoubleJointedArmSystem(String shoulderName, String elbowName, HardwareMap hwMap){
        elbowMotor = new ArmMotor(elbowName, hwMap);
        shoulderMotor = new ArmMotor(shoulderName, hwMap);
    }


    public void setMode(RunMode mode){
        elbowMotor.setMode(mode);
        shoulderMotor.setMode(mode);
    }
    public void setGamepad(Gamepad gamepad){
        this.gamepad = gamepad;
        elbowMotor.setGamepad(gamepad);
        shoulderMotor.setGamepad(gamepad);
    }
    public void stop(){
        elbowMotor.stop();
        shoulderMotor.stop();
    }

    public boolean inPosition(){
        return shoulderMotor.getCurrentPower() == shoulderMotor.getTargetPosition()
                && elbowMotor.getCurrentPosition() == elbowMotor.getCurrentPosition();
    }

    public void runToPosition(GamepadButton button, int[] ticks, double[] power, boolean manualControlShoulder, boolean manualControlElbow) {
        runToPosition(button, ticks, power, new double[]{0,0}, manualControlShoulder, manualControlElbow);
    }
    public void runToPosition(GamepadButton button, int[] ticks, double[] power, double[] delays,
                              boolean manualControlShoulder, boolean manualControlElbow) {
        if (button.isNone()) return;

        if (button.getValue(gamepad)) {
            manualControlShoulder = false;
            manualControlElbow = false;
            manualOverrideShoulder = false;
            manualOverrideElbow = false;

            shoulderTimer.reset();
            elbowTimer.reset();

            elbowMotor.setMode(RunMode.RUN_TO_POSITION);
            elbowMotor.setTargetPosition(ticks[1]);
            shoulderMotor.resetPower(0);
            shoulderMotor.setMode(RunMode.RUN_TO_POSITION);
            shoulderMotor.setTargetPosition(ticks[0]);
            shoulderMotor.resetPower(0);
        }
        if (elbowMotor.getMode() == RunMode.RUN_TO_POSITION && elbowMotor.getTargetPosition() == ticks[1] && !manualControlElbow && elbowTimer.seconds() > delays[1]) {
            elbowMotor.resetPower(power[1]);
        }
        if(shoulderMotor.getMode() == RunMode.RUN_TO_POSITION && shoulderMotor.getTargetPosition() == ticks[0] && !manualControlShoulder && shoulderTimer.seconds() > delays[0]){
        	System.out.println(shoulderTimer.seconds()+" "+delays[0]);
            shoulderMotor.resetPower(power[0]);
        }
    }
    public void runToPosition(int[] ticks, double[] power, double[] delays, boolean firstTime){
        if(firstTime) {
            elbowMotor.setMode(RunMode.RUN_TO_POSITION);
            elbowMotor.setTargetPosition(ticks[1]);
            shoulderMotor.setMode(RunMode.RUN_TO_POSITION);
            shoulderMotor.setTargetPosition(ticks[0]);

            elbowTimer.reset();
            shoulderTimer.reset();
        }

        if(elbowTimer.seconds() > delays[1]) {
            elbowMotor.resetPower(power[1]);
        }
        if(shoulderTimer.seconds() > delays[0]) {
            shoulderMotor.resetPower(power[0]);
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
