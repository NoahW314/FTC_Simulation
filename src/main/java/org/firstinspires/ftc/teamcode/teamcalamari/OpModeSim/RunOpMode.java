package org.firstinspires.ftc.teamcode.teamcalamari.OpModeSim;

import java.io.IOException;
import java.util.Scanner;

import org.teamcalamari.OpModeTests.Rover.TeleOpCompSim;


public class RunOpMode {

	public static void main(String[] args) throws IOException, InterruptedException {
		//replace with the OpMode instance that you want to test with
		OpMode mode = new TeleOpCompSim();
		RunOpMode rom = new RunOpMode();
		initLoop il = rom.new initLoop(mode);
		Scanner sc = new Scanner(System.in);
		updateGamepads ug = rom.new updateGamepads(sc, mode);
	    
	    System.out.println("Press enter to initialize OpMode");
		sc.nextLine();
	    mode.init();
	    mode.telemetry.setHardwareMap(mode.hardwareMap);
	    Thread t = new Thread(il);
	    t.start();
	    
	    System.out.println("Press enter to start OpMode");
	    sc.nextLine();
	    //sc.close();
	    il.loop = false;
	    mode.start();
	    t.interrupt();
	    Thread tG = new Thread(ug);
	    tG.start();
	    
	    while(true) {
	    	mode.loop();
	    	mode.telemetry.setHardwareMap(mode.hardwareMap);
	    	mode.telemetry.update();
	    	Thread.sleep(100);
	    	if(mode.isOpModeStopRequested()) {
	    		ug.loop = false;
	    		sc.close();
	    		break;
	    	}
	    }
	}
	
	private class initLoop implements Runnable{
		volatile boolean loop = true;
		OpMode mode;
		public initLoop(OpMode mode) {
			this.mode = mode;
		}
		@Override
		public void run() {
			while(loop) {
				mode.init_loop();
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
				}
			}
		}
	}
	private class updateGamepads implements Runnable{
		volatile boolean loop = true;
		OpMode mode;
		Scanner s;
		public updateGamepads(Scanner sc, OpMode mode) {
			s = sc;
			this.mode = mode;
		}
		@Override
		public void run() {
			while(loop) {
				String str = s.nextLine();
				if(str.startsWith("1")) {
					str = str.substring(1, str.length());
					mode.gamepad1.update(str);
				}
				else{
					if(str.length() > 0) {
						str = str.substring(1, str.length());
					}
					mode.gamepad2.update(str);
				}
			}
		}
	}
}