package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeSim;

import java.util.Scanner;

import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeTests.VuforiaTests.RoverRuckusDetectionKnownAngle;

public class RunLinearOpMode {

	public static void main(String[] args) {
		LinearOpModeSim mode = new RoverRuckusDetectionKnownAngle();
		
		Scanner s = new Scanner(System.in);
		System.out.println("Press enter to initialize OpMode");
		s.nextLine();
		
		mode.setScanner(s);
		
		try {
			mode.runOpMode();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

}
