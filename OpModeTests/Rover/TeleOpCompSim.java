package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeTests.Rover;

import java.io.FileReader;
import java.io.IOException;

import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.RobotsSim.RoverRobotSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeSim.OpModeSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Tests.ResetStartingHeading;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Rover Comp", group="Comp")
public class TeleOpCompSim extends OpModeSim {

    private RoverRobotSim robot = new RoverRobotSim(RoverRobotSim.RoverCompPrograms.TELE_OP);

    @Override
    public void init() {
    	
        FileReader fr;
        try {
            //read the heading from the file and use it as the starting heading
            fr = new FileReader("F:/DataLogger/Heading Comp.txt");
            robot.startingHeading = new Angle(ResetStartingHeading.getHeading(fr));
            
            robot.autoMotorTicks = RoverRobotSim.processEncoderFile();
        }
        catch(IOException e) {
            //catch any errors and send them to the telemetry

            //don't automatically clear the data off the telemetry screen
            telemetry.setAutoClear(false);
            telemetry.addData("Error: ", e.getMessage());
            telemetry.update();
        }
        
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.update(gamepad1, gamepad2);
        
        telemetry.addData("Driver Oriented", robot.driverOriented);
        telemetry.addData("Lift position", robot.lift.getCurrentPosition());
        telemetry.addData("Shoulder position", robot.arm.shoulderMotor.getCurrentPosition());
        telemetry.addData("Elbow position", robot.arm.elbowMotor.getCurrentPosition());
        telemetry.addData("Heading", Math.round(robot.absoluteHeading.getDegree()));
        telemetry.addData("Starting Heading", Math.round(robot.startingHeading.getDegree()));
        telemetry.update();
    }
}
