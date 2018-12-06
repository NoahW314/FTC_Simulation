package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim;

import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.HardwareMapSim;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadJoystick;

public class ArmMotorSim extends MotorSim{
    public ArmMotorSim(MotorSim motor) { super(motor); }

    public ArmMotorSim(String motorName, HardwareMapSim hwMap) { super(hwMap, motorName); }

    public void correctedSetPower(GamepadJoystick joystick, double powerCorrection, int[] correctionRange){
        if(joystick.isNone()) return;
        if(getCurrentPosition() >= correctionRange[0] && getCurrentPosition() <= correctionRange[1]) {
            setMode(RunMode.RUN_USING_ENCODER);
            setPower(joystick.getValue(gamepad) + powerCorrection);
        }
    }
}