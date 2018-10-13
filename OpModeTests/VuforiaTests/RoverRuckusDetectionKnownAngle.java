package org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeTests.VuforiaTests;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.AutonomousNavigationSim.AutonomousNavigationSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.AutonomousNavigationSim.AutonomousNavigationSim.RobotStates;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.VuforiaSimTC;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.DriveSim.OmniWheelDriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.HardwareSim.SensorsSim.BNO055IMUSim;
import org.firstinspires.ftc.teamcode.teamcalamari.Simulation.OpModeSim.LinearOpModeSim;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Vuforia Detection Angle", group="Vuforia Detection")
public class RoverRuckusDetectionKnownAngle extends LinearOpModeSim {

	
	private VuforiaSimTC vuforia;
    private boolean targetVisible = false;
    private ElapsedTime timer = new ElapsedTime();
    private double detectTime = -1;

    private BNO055IMUSim imu;

    private MotorSim redMotor;
    private MotorSim blueMotor;
    private MotorSim greenMotor;
    private MotorSim whiteMotor;
    private OmniWheelDriveSim drive;

    private AutonomousNavigationSim nav = new AutonomousNavigationSim(2, 0.5, 0, 0.75, 1, this);

    @Override
    public void runOpMode() throws InterruptedException {
        //drivetrain initialization
        redMotor = hardwareMap.dcMotor.get("redMotor");
        blueMotor = hardwareMap.dcMotor.get("blueMotor");
        greenMotor = hardwareMap.dcMotor.get("greenMotor");
        whiteMotor = hardwareMap.dcMotor.get("whiteMotor");
        drive = new OmniWheelDriveSim(new MotorSim[]{redMotor, blueMotor, greenMotor, whiteMotor}, new Angle(45), OpModeType.AUTO);

        drive.turnError = new Angle(2);
        drive.driveError = 0.1;

        //Vuforia initialization
        VuforiaSimTC.Parameters parametersVuforia = new VuforiaSimTC.Parameters(hardwareMap);
        parametersVuforia.useExtendedTracking = false;
		parametersVuforia.cameraDirection = CameraDirection.BACK;

		try {
			vuforia = new VuforiaSimTC(parametersVuforia, "RoverRuckus");
		} catch (Exception e) {
			e.printStackTrace();
		}
        
        vuforia.activate();


        //IMU initialization
        BNO055IMUSim.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled      = true;
        parametersIMU.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMUSim.class, "imu");
        imu.initialize(parametersIMU);
        
        //navigation initialization
        nav.targets.set(0, new VectorF(0, 0));
        nav.targets.set(1, new VectorF(0, 0));

        nav.stationaryTurns.set(0, new Angle(0));
        
        nav.stickState = RobotStates.STATIONARY_TURN;
        nav.stickI = 0;

        imu.lastReturned = -15;
        imu.turning = true;
        nav.update(new VectorF(-1, -1), new Angle(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES));
        nav.initialize(drive);


        waitForStart();
        timer.reset();

        while(opModeIsActive()){
        	if(timer.seconds() > 2) {vuforia.trackables.get(0).isVisible = true;}
        	if(timer.seconds() > 5) {vuforia.trackables.get(1).isVisible = true;}
        	if(timer.seconds() > 10) {vuforia.trackables.get(2).isVisible = true;}
        	if(timer.seconds() > 15) {vuforia.trackables.get(3).isVisible = true;}
        	if(timer.seconds() > 15 && timer.seconds() < 16) {imu.lastReturned = -15;}
        	
        	Angle heading = new Angle(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES);
            nav.updateHeading(heading);
            nav.run();
            if(heading.getDegree() <= 1 && heading.getDegree() >= -1) {imu.turning = false;}
            else {imu.turning = true;}
            

            targetVisible = false;
            for (VuforiaSimTC.Trackable trackable : vuforia.trackables) {
                if (trackable.isVisible) {
                    if(detectTime == -1) {
                        detectTime = timer.seconds();
                    }
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                }
            }

            if(!targetVisible){
                telemetry.addData("Wow", " nice cloaking device.");
            }
            telemetry.addData("Detection Time", detectTime);
            telemetry.addData("Heading", nav.getHeading());
            telemetry.addData("State", nav.robotState);
            telemetry.update();
            
        }
    }
}
