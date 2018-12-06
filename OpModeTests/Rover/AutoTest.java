package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeTests.Rover;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.AutonomousNavigationSim.AllDirectionsNavigationSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.RobotsSim.RoverRobotSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeSim.LinearOpModeSim;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Final Simple Test", group="Auto Final")
public class AutoTest extends LinearOpModeSim {

    RoverRobotSim robot = new RoverRobotSim(RoverRobotSim.RoverCompPrograms.AUTO);

    private double speed = 0.5;
    private double driveSpeed = 0.8;

    private double stillTurnSpeed = 0.75;
    private double stillTurnTurnSpeed = 0.5;

    private double moveTurnSpeed = 0.5;

    private int targetNum = 2;
    private AllDirectionsNavigationSim nav = new AllDirectionsNavigationSim(targetNum, speed, driveSpeed, stillTurnSpeed, stillTurnTurnSpeed, moveTurnSpeed, this);

    private RobotAction detectSamples = new RobotAction(){
        @Override
        public boolean act(){
            return true;
        }
    };

    @Override
    public void runOpMode(){

        nav.targets.set(0, new VectorF(12, 0));
        nav.targets.set(1, new VectorF(51, 0));

        robot.init(hardwareMap);

        //navigation initialization
        nav.update(new VectorF(12, 0), new Angle(60, AngleUnit.DEGREES), true);
        nav.initialize(robot.drive);

        robot.imu.lastReturned = 0;
        
        waitForStart();

        while(opModeIsActive()){
            Angle heading = new Angle(robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES);
            robot.drive.updateEncoderMotion(Angle.add(heading, new Angle(60)));
            nav.updateHeading(heading, false);
            Position pose = robot.drive.getEncoderMotion().position;
            nav.updatePosition(new VectorF((float)pose.x, (float)pose.y), false);
            nav.run();
            telemetry.addData("Position Enc", robot.drive.getEncoderMotion().position);
            telemetry.addData("Position", nav.getPosition());
            telemetry.addData("Heading", nav.getHeading());
            telemetry.update();
        }
    }
}
