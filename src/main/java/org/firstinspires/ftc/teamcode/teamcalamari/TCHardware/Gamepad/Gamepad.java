package org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad;

public class Gamepad extends com.qualcomm.robotcore.hardware.Gamepad {
	public Gamepad(GamepadCallback callback) {
		super(callback);
	}
	public Gamepad() {}
	
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
		dpad_right = input.contains(";");
		dpad_left = input.contains("k");

		//mode, start, back
		guide = input.contains("g");
		start = input.contains("d");
		back = input.contains("f");
		
		
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
		if(input.contains("t")) {right_stick_y = -1;}
		if(input.contains("u")) {right_stick_y = 0;}
		if(input.contains("i")) {right_stick_y = 1;}

		
		callCallback();
	}
	
	public static String letterToFullString(String letter) {
		switch(letter) {
			case "a":
			case "b":
			case "x":
			case "y":
				return letter.toUpperCase();
			
			case "p": return "RIGHT_BUMPER";
			case "q": return "LEFT_BUMPER";
			
			case "o": return "DPAD_UP";
			case "l": return "DPAD_OWN";
			case ";": return "DPAD_RIGHT";
			case "k": return "DPAD_LEFT";
			
			case "g": return "GUIDE";
			case "d": return "START";
			case "f": return "BACK";
			default: throw new IllegalArgumentException("Invalid Letter for a gamepad");
		}
	}
}
