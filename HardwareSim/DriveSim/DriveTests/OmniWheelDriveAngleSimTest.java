package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.DriveSim.DriveTests;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.DriveSim.OmniWheelDriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeSim.LinearOpModeSim;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class OmniWheelDriveAngleSimTest extends LinearOpModeSim {
	
	private Angle angle = new Angle(135, AngleUnit.DEGREES);
	private double speed = 1;
    private double driveSpeed = 0.5;
	
	private MotorSim WhiteMotor;
    private MotorSim GreenMotor;
    private MotorSim BlueMotor;
    private MotorSim RedMotor;
    
    private OmniWheelDriveSim drive;

	@Override
	public void runOpMode() throws InterruptedException {
		WhiteMotor = hardwareMap.dcMotor.get("WhiteMotor");
        GreenMotor = hardwareMap.dcMotor.get("GreenMotor");
        BlueMotor = hardwareMap.dcMotor.get("BlueMotor");
        RedMotor = hardwareMap.dcMotor.get("RedMotor");

        //set the motors to run at a set speed
        BlueMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WhiteMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GreenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RedMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //set the motors to brake. Only required for the hub
        WhiteMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GreenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BlueMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        MotorConfigurationType motorType = new MotorConfigurationType();
        motorType.processAnnotation(NeveRest40Gearmotor.class.getDeclaredAnnotation(MotorType.class));
        
        WhiteMotor.setMotorType(motorType);
        GreenMotor.setMotorType(motorType);
        BlueMotor.setMotorType(motorType);
        RedMotor.setMotorType(motorType);
        
        drive = new OmniWheelDriveSim(new MotorSim[] {BlueMotor,WhiteMotor,RedMotor,GreenMotor}, angle, OpModeType.AUTO);
        drive.updateEncoders();
        drive.inchesPerTick = Math.PI*4/BlueMotor.getMotorType().getTicksPerRev();
        drive.turnError = new Angle(2);
        drive.driveError = 2;
        
        drive.speed = speed;
        drive.driveSpeed = driveSpeed;
        
        waitForStart();
        
        while(opModeIsActive()) {
        	drive.driveAtAngle(new Angle(0));
        	drive.drive();
        	telemetry.update();
        }
	}

}
