package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.DriveTests;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.TankDriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.LinearOpModeSim;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

public class TankDriveTurnToTest extends LinearOpModeSim {

	private MotorSim LeftMotor;
	private MotorSim RightMotor;
	private TankDriveSim drive;
	
	private Angle turnToAngle = new Angle(Math.round(Math.random()*360));
	
	@Override
	public void runOpMode() throws InterruptedException {
		LeftMotor = hardwareMap.dcMotor.get("leftMotor");
		RightMotor = hardwareMap.dcMotor.get("rightMotor");
		
		LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        MotorConfigurationType motorType = new MotorConfigurationType();
        motorType.processAnnotation(NeveRest40Gearmotor.class.getDeclaredAnnotation(MotorType.class));
        
        LeftMotor.setMotorType(motorType);
        RightMotor.setMotorType(motorType);
		
		drive = new TankDriveSim(new MotorSim[] {LeftMotor, RightMotor});
		drive.updateEncoders();
        drive.inchesPerTick = Math.PI*6/LeftMotor.getMotorType().getTicksPerRev();
        drive.turnRadius = 8;
        drive.turnError = new Angle(2);
        drive.driveError = 2;
        
        drive.turnSpeed = 0.5;
        drive.driveSpeed = 0.5;
        drive.speed = 0.5;
		
		waitForStart();
		
		while(opModeIsActive()) {
        	drive.updateEncoderMotion();
        	Angle heading = drive.getEncoderHeading();
        	boolean there = drive.driveTo(drive.getEncoderDistance(), new VectorF(24, 0), heading);
        	drive.updateEncoders();
        	telemetry.addData("State", drive.driveToState);
        	telemetry.addData("Heading", heading.getDegree());
        	telemetry.addData("Position", drive.getEncoderDistance());
        	telemetry.addData("Turn Speed", drive.turnSpeed);
        	telemetry.addData("Drive Speed", drive.driveSpeed);
        	telemetry.addData("Speed", drive.speed);
        	telemetry.update();
        	if(there) {
        		this.requestOpModeStop();
        	}
		}
		
	}

}
