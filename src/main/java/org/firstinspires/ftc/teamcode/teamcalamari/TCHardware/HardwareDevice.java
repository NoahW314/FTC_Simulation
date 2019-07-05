package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware;


public interface HardwareDevice extends com.qualcomm.robotcore.hardware.HardwareDevice {
	void move();
	String log(String deviceName);
}
