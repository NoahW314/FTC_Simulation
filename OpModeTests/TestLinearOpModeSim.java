package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeTests;

import org.firstinspires.ftc.teamcode.teamcalamari.Math2;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.LinearOpModeSim;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TestLinearOpModeSim extends LinearOpModeSim {

	private MotorSim rightMotor;
	private MotorSim leftMotor;
	private Servo buttonPusher;
	
	private ElapsedTime runtime = new ElapsedTime();
	
	@Override
	public void runOpMode() throws InterruptedException {
		rightMotor = hardwareMap.dcMotor.get("rightMotor");
		leftMotor = hardwareMap.dcMotor.get("leftMotor");
		buttonPusher = hardwareMap.servo.get("buttonPusher");
		
		rightMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
		rightMotor.setMode(RunMode.RUN_TO_POSITION);
		leftMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
		leftMotor.setMode(RunMode.RUN_TO_POSITION);
		
		waitForStart();
		runtime.reset();
		
		rightMotor.setTargetPosition(1680);
		leftMotor.setTargetPosition(-1680);
		rightMotor.setPower(1);
		leftMotor.setPower(1);
		rightMotor.run();
		leftMotor.run();
		
		while(opModeIsActive() && rightMotor.getCurrentPosition() < 1680) {
			telemetry.addData("Position", " "+rightMotor.getCurrentPosition());
			telemetry.addData("Driving", " forward");
			telemetry.addData("Time", " "+Math2.round(runtime.seconds(), 0.1));
			telemetry.update();
		}
		telemetry.update();
		
		rightMotor.setPower(0);
		leftMotor.setPower(0);
		rightMotor.run();
		leftMotor.run();
		buttonPusher.setPosition(1);
		
		while(opModeIsActive()) {
			telemetry.addData("MotorType ", " "+rightMotor.getMotorType());
			telemetry.addData("Time", " "+Math2.round(runtime.seconds(), 0.1));
			telemetry.update();
			this.requestOpModeStop();
		}
	}
}
