package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware;

public class ServoController implements com.qualcomm.robotcore.hardware.ServoController {

	@Override
	public Manufacturer getManufacturer() {
		return Manufacturer.Other;
	}

	@Override
	public String getDeviceName() {
		return "Servo Controller ulation";
	}

	@Override
	public String getConnectionInfo() {
		return "Connection Info doesn't exist for a ulated Servo controller";
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
	public void pwmEnable() {
	}

	@Override
	public void pwmDisable() {
	}

	@Override
	public PwmStatus getPwmStatus() {
		return PwmStatus.DISABLED;
	}

	@Override
	public void setServoPosition(int servo, double position) {
	}

	@Override
	public double getServoPosition(int servo) {
		return 0;
	}

}
