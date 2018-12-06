package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.ServosSim;

import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.ServoControllerSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.MotorSimpleSim;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;

public class CRServoSim extends MotorSimpleSim implements CRServo {

	@Override
	public ServoController getController() {
		return new ServoControllerSim();
	}

	@Override
	public int getPortNumber() {
		return 0;
	}
	
}
