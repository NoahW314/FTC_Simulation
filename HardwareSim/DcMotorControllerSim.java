package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.UnspecifiedMotor;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class DcMotorControllerSim implements DcMotorController {

	@Override
	public Manufacturer getManufacturer() {
		return Manufacturer.Other;
	}

	@Override
	public String getDeviceName() {
		return "Motor Controller Simulation";
	}

	@Override
	public String getConnectionInfo() {
		return "Connection Info doesn't exist for a simulated Motor Controller";
	}

	@Override
	public int getVersion() {
		return 0;
	}

	@Override
	public void resetDeviceConfigurationForOpMode() {
	}

	@Override
	public void close() {
	}

	@Override
	public void setMotorType(int motor, MotorConfigurationType motorType) {
	}

	@Override
	public MotorConfigurationType getMotorType(int motor) {
		MotorConfigurationType type = new MotorConfigurationType();
		type.processAnnotation(UnspecifiedMotor.class.getDeclaredAnnotation(MotorType.class));
		return type;
	}

	@Override
	public void setMotorMode(int motor, RunMode mode) {
	}

	@Override
	public RunMode getMotorMode(int motor) {
		return RunMode.RUN_WITHOUT_ENCODER;
	}

	@Override
	public void setMotorPower(int motor, double power) {
	}

	@Override
	public double getMotorPower(int motor) {
		return 0;
	}

	@Override
	public boolean isBusy(int motor) {
		return false;
	}

	@Override
	public void setMotorZeroPowerBehavior(int motor, ZeroPowerBehavior zeroPowerBehavior) {
	}

	@Override
	public ZeroPowerBehavior getMotorZeroPowerBehavior(int motor) {
		return ZeroPowerBehavior.UNKNOWN;
	}

	@Override
	public boolean getMotorPowerFloat(int motor) {
		return false;
	}

	@Override
	public void setMotorTargetPosition(int motor, int position) {
	}

	@Override
	public int getMotorTargetPosition(int motor) {
		return 0;
	}

	@Override
	public int getMotorCurrentPosition(int motor) {
		return 0;
	}

	@Override
	public void resetDeviceConfigurationForOpMode(int motor) {
	}

}
