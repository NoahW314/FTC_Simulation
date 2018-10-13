package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeTests;

import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.OpModeSim;

public class TestOpModeSim extends OpModeSim {

	@Override
	public void init() {
		System.out.println("Initialized");
	}

	@Override
	public void init_loop() {
		System.out.println("Init Looping...");
	}
	
	@Override
	public void start() {
		System.out.println("Started");
	}
	
	@Override
	public void loop() {
		System.out.println("Looping...");
	}

}
