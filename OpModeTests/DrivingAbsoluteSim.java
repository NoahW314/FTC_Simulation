package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeTests;

import java.io.FileReader;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.DataLogger;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.DriveSim.turnDirection;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.RobotsSim.RelicRobotSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.RobotsSim.RelicRobotSim.RelicPrograms;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.OpModeSim;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Drive Absolute", group="Drive testing")
public class DrivingAbsoluteSim extends OpModeSim {
	
	private RelicRobotSim falcon = new RelicRobotSim(RelicPrograms.DRIVEABS);
	/**heading that the robot started the program at relative to the zero defined by ResetStartingHeading*/
	private Angle startingHeading;
	/**current heading as reported by the imu, relative to the starting heading*/
	private Angle relativeHeading;
	/**current heading relative to the zero defined by ResetStartingHeading*/
	private Angle absoluteHeading;

	@Override
	public void init() {
		falcon.init(hardwareMap);
		
		FileReader fr;
		//read the heading from the file and use it as the starting heading
		//fr = new FileReader("/storage/emulated/0/DataLogger/Heading.txt");
		//startingHeading = new Angle(ResetStartingHeading.getHeading(fr));
		startingHeading = new Angle(0);
		
		telemetry.logHardware(false);
	}

	@Override
	public void loop() {
		//get the relative heading from the imu
		relativeHeading = new Angle(falcon.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
		//get the absolute heading by adding the starting heading and the relative heading
		absoluteHeading = Angle.add(startingHeading, relativeHeading);
		
		//store the joystick values in variables for easier access
        double glx = gamepad1.left_stick_x;
        double gly = gamepad1.left_stick_y;

        //only drive the robot if all the wheel motors are enabled
        //if(falcon.wheelMotorsEnabled()) {
        	falcon.drive.driveSpeed = Math.sqrt(Math.pow(glx, 2) + Math.pow(gly, 2));
        	falcon.drive.driveAtAngle(Angle.add(new Angle(Math.atan2(gly, glx), AngleUnit.RADIANS), absoluteHeading));
	        //if we are moving turn at half the speed
	        if (glx == 0 && gly == 0) {
	        	falcon.drive.speed = Math.abs(gamepad1.right_trigger-gamepad1.left_trigger)*falcon.speed;
	        	falcon.drive.turnSpeed = gamepad1.right_trigger;
	        	falcon.drive.turn(turnDirection.CW);
	        	falcon.drive.turnSpeed = gamepad1.left_trigger;
	        	falcon.drive.turn(turnDirection.CCW);
	        	falcon.drive.drive();
	        } else {
	        	falcon.drive.speed = Math.sqrt(Math.pow(glx, 2) + Math.pow(gly, 2))*falcon.speed;
	        	falcon.drive.turnSpeed = gamepad1.right_trigger/2;
	        	falcon.drive.turn(turnDirection.CW);
	        	falcon.drive.turnSpeed = gamepad1.left_trigger/2;
	        	falcon.drive.turn(turnDirection.CCW);
	        	falcon.drive.drive();
	        }
	        //}
	}
	
	@Override
	public void stop() {
		//log the current heading for use as the starting heading next time
		DataLogger logger = new DataLogger("Heading");
		logger.addField(absoluteHeading.getDegree());
		logger.newLine();
		logger.closeDataLogger();
	}

}
