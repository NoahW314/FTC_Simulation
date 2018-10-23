package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeTests;

import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeSim.LinearOpModeSim;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class MotorSimTest extends LinearOpModeSim {
	
	private MotorSim motor;

	@Override
	public void runOpMode() throws InterruptedException {
		motor = hardwareMap.dcMotor.get("motor");
		
		motor.setMode(RunMode.RUN_USING_ENCODER);
		motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
		
		MotorConfigurationType motorType = new MotorConfigurationType();
		motorType.processAnnotation(NeveRest40Gearmotor.class.getAnnotation(MotorType.class));
		
		motor.setMotorType(motorType);
		
		waitForStart();
		
		while(opModeIsActive()) {
			motor.setPower(1);
			motor.run();
			telemetry.addData("Position", motor.getCurrentPosition());
			telemetry.update();
			if(this.getRuntime() > 5) {
				this.requestOpModeStop();
			}
		}
	}

}
