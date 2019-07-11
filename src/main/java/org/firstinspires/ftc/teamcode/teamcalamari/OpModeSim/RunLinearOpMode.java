package org.firstinspires.ftc.teamcode.teamcalamari.OpModeSim;

import java.util.Scanner;

import org.firstinspires.ftc.teamcode.teamcalamari.OpModeTests.Nav.OmniDriveAndTurnTrackingTest;

public class RunLinearOpMode {

	public static void main(String[] args) {
		LinearOpMode mode = new OmniDriveAndTurnTrackingTest();
		
		//mode.loopTime = 250;
		//mode.useTimer(false);
		
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
