package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Servos;

import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.ServoController;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.MotorSimple;

public class CRServo extends MotorSimple implements com.qualcomm.robotcore.hardware.CRServo {

	@Override
	public ServoController getController() {
		return new ServoController();
	}

	@Override
	public int getPortNumber() {
		return 0;
	}
	
}
