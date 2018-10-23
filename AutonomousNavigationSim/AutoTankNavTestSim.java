package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.AutonomousNavigationSim;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.DriveSim.TankDriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.OpModeSim.LinearOpModeSim;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Tank Nav Test", group="Nav Tests")
public class AutoTankNavTestSim extends LinearOpModeSim{
	
    //the speed of the robot
    private double speed = 1;
    private double driveSpeed = 0.75;
    
    private double stillTurnSpeed = 1;
    private double stillTurnTurnSpeed = 0.75;
	
	//the motors, servos, sensors, servo values, and drivetrain
    private MotorSim LeftMotor;
    private MotorSim RightMotor;
    private MotorSim swiffer;
    
    private TankDriveSim drive;
    
    //navigation variables
    private int targetNum = 4;
    private AutonomousNavigationSim nav = new AutonomousNavigationSim(targetNum, speed, driveSpeed, stillTurnSpeed, stillTurnTurnSpeed, this);
    
    RobotAction log = new RobotAction() {
    	ElapsedTime time = new ElapsedTime();
    	public boolean start() {
    		time.reset();
    		return true;
    	}
    	public boolean act() {
    		if(time.seconds() > 1) {
    			return true;
    		}
    		else {
    			telemetry.addData("Drive Action", "Running");
    		}
    		return false;
    	}
    };
    
	@Override
	public void runOpMode() throws InterruptedException {
		nav.targets.set(0, new VectorF(0, 0));
	    nav.targets.set(1, new VectorF(24, 0));
	    nav.targets.set(2, new VectorF(24, 24));
	    nav.targets.set(3, new VectorF(0, 0));
	    
	    nav.stationaryTurns.set(0, new Angle(-45.0, AngleUnit.DEGREES));
	    nav.stationaryTurns.set(1, new Angle(125.0, AngleUnit.DEGREES));
	    
	    nav.actions.set(0, new RobotAction() {
	    	public boolean act() {
	    		swiffer.setPower(1);
	    		swiffer.run();
	    		return true;
	    	}
	    });
	    nav.actions.set(1, new RobotAction() {
	    	public boolean act() {
	    		swiffer.setPower(0);
	    		swiffer.run();
	    		return true;
	    	}
	    });
	    nav.actionFirst.set(1, true);
	    
	    nav.drivingActions.set(0, log);
	    		
		LeftMotor = hardwareMap.dcMotor.get("leftMotor");
        RightMotor = hardwareMap.dcMotor.get("rightMotor");
        swiffer = hardwareMap.dcMotor.get("swiffer");

        //set the motors to run at a set speed
        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        MotorConfigurationType motorType = new MotorConfigurationType();
        motorType.processAnnotation(NeveRest40Gearmotor.class.getDeclaredAnnotation(MotorType.class));
        
        LeftMotor.setMotorType(motorType);
        RightMotor.setMotorType(motorType);
        
        //drivetrain initialization
        drive = new TankDriveSim(new MotorSim[] {LeftMotor, RightMotor});
        drive.updateEncoders();
        drive.inchesPerTick = Math.PI*6/LeftMotor.getMotorType().getTicksPerRev();
        drive.turnRadius = 8;
        drive.turnError = new Angle(15);
        drive.driveError = 5;
        
        //navigation initialization
	    nav.update(new VectorF(0, 0), new Angle(0, AngleUnit.DEGREES));
        nav.initialize(drive);
        
        waitForStart();
        
        while(opModeIsActive()) {
        	drive.updateEncoderMotion();
        	nav.updateHeading(drive.getEncoderHeading());
        	nav.updatePosition(drive.getEncoderDistance());
        	nav.run();
        	drive.updateEncoders();
        	telemetry.addData("State", nav.robotState+" #"+nav.i);
        	telemetry.update();
        }
	}
}
