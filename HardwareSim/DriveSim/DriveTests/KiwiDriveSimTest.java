package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.DriveTests;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.DriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.KiwiDriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.OmniWheelDriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.OpModeSim;

public class KiwiDriveSimTest extends OpModeSim {

    //the angle of the front of the robot relative to the blue wheel
    private Angle angle = new Angle(90, AngleUnit.DEGREES);

    private MotorSim WhiteMotor;
    private MotorSim GreenMotor;
    private MotorSim BlueMotor;

    private KiwiDriveSim drive;

    @Override
    public void init() {
        WhiteMotor = hardwareMap.dcMotor.get("WhiteMotor");
        GreenMotor = hardwareMap.dcMotor.get("GreenMotor");
        BlueMotor = hardwareMap.dcMotor.get("BlueMotor");

        //set the motors to run at a set speed
        BlueMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WhiteMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GreenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set the motors to brake. Only required for the hub
        WhiteMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GreenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BlueMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new KiwiDriveSim(new MotorSim[]{BlueMotor,WhiteMotor,GreenMotor}, angle, OpModeType.AUTO);
        drive.driveSpeed = 1;
    }

    @Override
    public void loop() {
        drive.speed = 1;
        if(gamepad1.right_bumper) {
            drive.driveAtAngle(new Angle(0, AngleUnit.DEGREES));
        }
        else if(gamepad1.left_bumper) {
            drive.driveAtAngle(new Angle(120, AngleUnit.DEGREES));
        }

        if(gamepad1.right_trigger > 0) {
            drive.driveAtAngle(new Angle(240, AngleUnit.DEGREES));
        }
        if(gamepad1.left_trigger > 0) {
        	drive.driveAtAngle(new Angle(-30, AngleUnit.DEGREES));
        }

        drive.drive();
    }
}
