package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadSim extends Gamepad {
	public GamepadSim(GamepadCallback callback) {
		super(callback);
	}
	public GamepadSim() {}
	
	public void update(String input) {
		//buttons
		
		//abxy
		a = input.contains("a");
		b = input.contains("b");
		x = input.contains("x");
		y = input.contains("y");
		
		//bumpers
		right_bumper = input.contains("p");
		left_bumper = input.contains("q");
		
		//dpad
		dpad_up = input.contains("o");
		dpad_down = input.contains("l");
		dpad_right = input.contains("k");
		dpad_left = input.contains(";");

		//mode, start, back
		guide = input.contains("m");
		start = input.contains("s");
		back = input.contains("`");
		
		
		//motion
		
		//triggers
		
		//left
		if(input.contains("1")) {left_trigger = 0;}
		if(input.contains("2")) {left_trigger = 1;}
		//right
		if(input.contains("9")) {right_trigger = 0;}
		if(input.contains("0")) {right_trigger = 1;}

		//joysticks
		
		//left
		//x
		if(input.contains("z")) {left_stick_x = -1;}
		if(input.contains("c")) {left_stick_x = 0;}
		if(input.contains("v")) {left_stick_x = 1;}
		//y
		if(input.contains("w")) {left_stick_y = -1;}
		if(input.contains("e")) {left_stick_y = 0;}
		if(input.contains("r")) {left_stick_y = 1;}
		//right
		//x
		if(input.contains("n")) {right_stick_x = -1;}
		if(input.contains("m")) {right_stick_x = 0;}
		if(input.contains(",")) {right_stick_x = 1;}
		//y
		if(input.contains("y")) {right_stick_y = -1;}
		if(input.contains("u")) {right_stick_y = 0;}
		if(input.contains("i")) {right_stick_y = 1;}

		
		callCallback();
	}
}
