package org.firstinspires.ftc.teamcode.teamcalamari.RobotAction;

import org.firstinspires.ftc.robotcore.external.function.Supplier;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.Motor;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotActionMotor extends RobotAction {
    protected Motor motor;
    protected DcMotor.RunMode mode;
    protected boolean reset;
    protected Supplier<Double> motorPowerSupplier;
    protected Supplier<Integer> motorPositionSupplier;
    public RobotActionMotor(Motor m, DcMotor.RunMode mode, Supplier<Double> supplier, boolean reset){
        motor = m;
        this.reset = reset;
        if(mode != DcMotor.RunMode.RUN_USING_ENCODER && mode != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            throw new IllegalStateException("Invalid Run Mode used in RobotActionMotor: "+mode);
        this.mode = mode;
        motorPowerSupplier = supplier;
    }
    public RobotActionMotor(Motor m, Supplier<Double> supplier){
        this(m, DcMotor.RunMode.RUN_USING_ENCODER, supplier, false);
    }
    public RobotActionMotor(Motor m, Supplier<Double> supplier, boolean reset){
        this(m, DcMotor.RunMode.RUN_USING_ENCODER, supplier, reset);
    }
    public RobotActionMotor(Motor m, Supplier<Double> powerSupplier, Supplier<Integer> poseSupplier, boolean reset){
        motor = m;
        this.reset = reset;
        mode = DcMotor.RunMode.RUN_TO_POSITION;
        motorPowerSupplier = powerSupplier;
        motorPositionSupplier = poseSupplier;
    }

    @Override
    public boolean start() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//paranoia
        motor.setMode(mode);
        return false;
    }

    @Override
    public boolean act(){
        if(mode == DcMotor.RunMode.RUN_TO_POSITION && motor.getMode() == mode){
            motor.setTargetPosition(motorPositionSupplier.get());
        }

        if(reset) motor.resetPower(motorPowerSupplier.get());
        else motor.setPower(motorPowerSupplier.get());
        return false;
    }
}
