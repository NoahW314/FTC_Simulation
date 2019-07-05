package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine;

import org.firstinspires.ftc.robotcore.external.function.Supplier;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.Motor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorRestrictedSupplier implements Supplier<Double> {
    private Motor motor;
    private DcMotorSimple.Direction restrictedDirection;

    public MotorRestrictedSupplier(Motor m, DcMotorSimple.Direction restrictedDirection){
        motor = m;
        this.restrictedDirection = restrictedDirection;
    }

    @Override
    public Double get(){
        if((restrictedDirection == DcMotorSimple.Direction.FORWARD && motor.getNextPower() > 0) ||
            restrictedDirection == DcMotorSimple.Direction.REVERSE && motor.getNextPower() < 0){
            return 0d;
        }
        return motor.getNextPower();
    }
}
