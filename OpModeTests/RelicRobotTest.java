package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeTests;

import java.util.Iterator;
import java.util.Map.Entry;

import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.HardwareDeviceSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.RobotsSim.RelicRobotSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.RobotsSim.RelicRobotSim.RelicPrograms;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.OpModeSim;

public class RelicRobotTest extends OpModeSim {
	
	private RelicRobotSim falcon = new RelicRobotSim(RelicPrograms.DRIVE);

	@Override
	public void init() {
		falcon.init(hardwareMap);
	}

	private int n = 0;
	@Override
	public void loop() {
		n++;
		falcon.update(gamepad1, gamepad2);
		System.out.println(falcon.BlueMotor.getPower());
		Iterator<Entry<HardwareDeviceSim, String>> i = hardwareMap.deviceMap.entrySet().iterator();
		while(i.hasNext()) {
			Entry<HardwareDeviceSim, String> e = i.next();
			if(e.getValue().equals("BlueMotor")) {
				System.out.println(((MotorSim)e.getKey()).getPower());
			}
		}
	}

}
