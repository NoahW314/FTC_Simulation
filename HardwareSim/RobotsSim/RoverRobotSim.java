package org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.RobotsSim;

import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeType;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotActionLinear;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.DataLoggerSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.AutonomousNavigationSim.AutonomousNavigationSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.GamepadSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.HardwareMapSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.DriveSim.DriveSim.turnDirection;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.DriveSim.KiwiDriveSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.DoubleJointedArmSystemSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.LiftMotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.MotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.SwifferMotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.MotorsSim.WheelMotorSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.SensorsSim.BNO055IMUSim;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.SensorsSim.DistanceSensorSimTC;
import org.firstinspires.ftc.teamcode.teamcalamari.FTC_Simulation.HardwareSim.ServosSim.CRServoSim;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadButton;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadJoystick;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadTrigger;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RoverRobotSim extends BaseRobotSim {


    public WheelMotorSim RedMotor;
    public WheelMotorSim YellowMotor;
    public WheelMotorSim GreenMotor;
    public KiwiDriveSim drive;
    
    public SwifferMotorSim collector;
    public LiftMotorSim lift;
    public DoubleJointedArmSystemSim arm;
    
    public int[] autoMotorTicks = new int[]{0,0,0};

    public DistanceSensorSimTC groundRange;
    public double inchesToGround = 3;

    public BNO055IMUSim imu;

    public double speed = 0.75;
    
    public HardwareMapSim hwMap;

    /**heading that the robot started the program at relative to the zero defined by ResetStartingHeading*/
    public Angle startingHeading;
    /**current heading as reported by the imu, relative to the starting heading*/
    public Angle relativeHeading;
    /**current heading relative to the zero defined by ResetStartingHeading*/
    public Angle absoluteHeading;
    
    public Angle phoneAngle = new Angle(-60);
    public Angle armAngle = new Angle(0);
    
    public boolean stopped = false;
    
    //collector controls
    public GamepadButton collectorStartInButton = GamepadButton.NONE;
    public GamepadButton collectorStartOutButton = GamepadButton.NONE;
    public GamepadButton collectorOffButton = GamepadButton.NONE;
    public GamepadButton collectorOneInButton = GamepadButton.NONE;
    public GamepadButton collectorTwoInButton = GamepadButton.NONE;
    public GamepadButton collectorOneOutButton = GamepadButton.NONE;
    public GamepadButton collectorTwoOutButton = GamepadButton.NONE;
    public GamepadButton collectorInButton = GamepadButton.NONE;
    public GamepadButton collectorOutButton = GamepadButton.NONE;
    public GamepadJoystick collectorInAndOut = GamepadJoystick.NONE;
    public GamepadTrigger collectorIn = GamepadTrigger.NONE;
    public GamepadTrigger collectorOut = GamepadTrigger.NONE;

    private double collectorOneInTime = 1;
    private double collectorTwoInTime = 2;
    private double collectorOneOutTime = 1;
    private double collectorTwoOutTime = 2;

    //lift controls
    public GamepadButton liftExtendFullyButton = GamepadButton.NONE;
    public GamepadButton liftRetractFullyButton = GamepadButton.NONE;
    public GamepadButton liftExtendButton = GamepadButton.NONE;
    public GamepadButton liftRetractButton = GamepadButton.NONE;
    public GamepadJoystick liftJoystick = GamepadJoystick.NONE;
    public GamepadTrigger liftUp = GamepadTrigger.NONE;
    public GamepadTrigger liftDown = GamepadTrigger.NONE;

    private int retractTicks = 0;
    private int extendTicks = 1440;

    //drivetrain controls
    public GamepadButton switchDriverOriented = GamepadButton.A;
    private boolean prevDriverOrientedState = false;
    public boolean driverOriented = true;

    //arm controls
    public GamepadButton armCraterExtension = GamepadButton.A;
    public GamepadButton armLanderExtension = GamepadButton.NONE;
    public GamepadButton armFoldUp = GamepadButton.X;
    public GamepadButton armDrivePosition = GamepadButton.Y;
    public GamepadJoystick armShoulderJoystickC = GamepadJoystick.RIGHT_STICK_X;
    public GamepadJoystick armElbowJoystickC = GamepadJoystick.RIGHT_STICK_Y;
    public GamepadJoystick armOverrideShoulder = GamepadJoystick.LEFT_STICK_X;
    public GamepadJoystick armOverrideElbow = GamepadJoystick.LEFT_STICK_Y;
    public GamepadButton armSetOverrideShoulder = GamepadButton.LEFT_BUMPER;
    public GamepadButton armSetOverrideElbow = GamepadButton.RIGHT_BUMPER;
    public GamepadButton armSetOverride = GamepadButton.START;

    //of form [shoulder, elbow]
    private int[] craterExtensionTicks = new int[]{100, 0};
    private int[] landerExtensionTicks = new int[]{50, 25};
    private int[] foldUpTicks = new int[]{0, 0};
    private int[] drivePositionTicks = new int[]{25, 72};

    public double armShoulderPowerCorrection = 0.3;
    public double armElbowPowerCorrection = 0;

    //lower should be first if being used
    public int[] armShoulderCorrectionRange = new int[]{90, 120};
    public int[] armElbowCorrectionRange = new int[]{0, -1};
    

    public RoverRobotSim(Program program){
        super(program);
    }

    @Override
    public void init(HardwareMapSim hwMap){
        this.hwMap = hwMap;


        collector = new SwifferMotorSim("collector", hwMap, CRServoSim.class, DcMotorSimple.Direction.FORWARD);

        lift = new LiftMotorSim(hwMap, "lift", DcMotorSimple.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setLimits(retractTicks, extendTicks);
        lift.startingTicks = autoMotorTicks[0];

        arm = new DoubleJointedArmSystemSim("shoulder", "elbow", hwMap);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorConfigurationType motorType = new MotorConfigurationType();
        motorType.processAnnotation(RevRoboticsCoreHexMotor.class.getDeclaredAnnotation(MotorType.class));
        arm.shoulderMotor.setMotorType(motorType);
        arm.elbowMotor.setMotorType(motorType);
        arm.shoulderMotor.startingTicks = autoMotorTicks[1];
        arm.elbowMotor.startingTicks = autoMotorTicks[2];

        //groundRange = new DistanceSensorSimTC(hwMap.get(ModernRoboticsI2cRangeSensor.class, "ground"));


        //get the motors
        RedMotor = new WheelMotorSim(hwMap, "RedMotor");
        YellowMotor = new WheelMotorSim(hwMap, "YellowMotor");
        GreenMotor = new WheelMotorSim(hwMap, "GreenMotor");

        //set the motors to brake. only required for the hub
        RedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        YellowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GreenMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reset the encoders before running
        RedMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        YellowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        GreenMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set the motors to run at a set speed
        RedMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        YellowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GreenMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initialize the drivetrain
        drive = new KiwiDriveSim(new MotorSim[]{RedMotor, GreenMotor, YellowMotor}, new Angle(0), program.getOpModeType());
        drive.updateEncoders();
        drive.inchesPerTick = Math.PI*4/1120;
        drive.turnError = new Angle(2);
        drive.driveError = 1;
        

        //imu initialization
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled      = true;
        parametersIMU.loggingTag          = "IMU";

        imu = hwMap.get(BNO055IMUSim.class, "imu");
        imu.initialize(parametersIMU);
        
        super.init(hwMap);
    }

    public boolean wheelMotorsEnabled() {
        return enabledDevices.get("RedMotor") && enabledDevices.get("GreenMotor") && enabledDevices.get("YellowMotor");
    }
    public boolean armEnabled() {
    	return enabledDevices.get("shoulder") && enabledDevices.get("elbow");
    }

    @Override
    public void update(GamepadSim gamepad1, GamepadSim gamepad2) {
    	
    	if(GamepadButton.B.getValue(gamepad1)) {
    		drive.headAngle = new Angle(120);
    	}
    	
        //store the joystick values in variables for easier access
        double glx = gamepad1.left_stick_x;
        double gly = gamepad1.left_stick_y;

        //only drive the robot if all the wheel motors are enabled
        if(wheelMotorsEnabled()) {
        	
        	if(!switchDriverOriented.isNone()){
                if(switchDriverOriented.getValue(gamepad1) && !prevDriverOrientedState){
                    driverOriented = !driverOriented;
                }
                prevDriverOrientedState = switchDriverOriented.getValue(gamepad1);
            }
        	
        	Angle driveHeading;
            Angle joystickAngle = new Angle(Math.atan2(gly, glx), AngleUnit.RADIANS);

            if ((program.equals(RoverTestPrograms.DRIVE_ABS) || program.equals(RoverTestPrograms.DRIVE_ABS_CORRECTIVE)
                    || program.equals(RoverCompPrograms.TELE_OP)) && driverOriented) {
                //get the relative heading from the imu
                relativeHeading = new Angle(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
                //get the absolute heading by adding the starting heading and the relative heading
                absoluteHeading = Angle.add(startingHeading, relativeHeading);

                driveHeading = Angle.add(absoluteHeading, joystickAngle);
            }
            else {
                //a little trick to prevent possible side effects from directly setting the drive heading to the joystick angle
                driveHeading = Angle.add(joystickAngle, new Angle(0));
            }

            drive.driveSpeed = Math.sqrt(Math.pow(glx, 2) + Math.pow(gly, 2));
            drive.driveAtAngle(driveHeading);
            //if we are moving turn at half the speed
            if (glx == 0 && gly == 0) {
                drive.speed = Math.abs(gamepad1.right_trigger - gamepad1.left_trigger) * speed;
                drive.turnSpeed = gamepad1.right_trigger;
                drive.turn(turnDirection.CW);
                drive.turnSpeed = gamepad1.left_trigger;
                drive.turn(turnDirection.CCW);
                drive.drive();
            } else {
                drive.speed = Math.sqrt(Math.pow(glx, 2) + Math.pow(gly, 2)) * speed;
                drive.turnSpeed = gamepad1.right_trigger / 2;
                drive.turn(turnDirection.CW);
                drive.turnSpeed = gamepad1.left_trigger / 2;
                drive.turn(turnDirection.CCW);
                drive.drive();
            }
        }
        
        if(enabledDevices.get("collector")){
	        collector.setGamepad(gamepad2);
	
	        collector.turnOnIn(collectorStartInButton);
	        collector.turnOnOut(collectorStartOutButton);
	        collector.turnOff(collectorOffButton);
	        collector.runIn(collectorInButton);
	        collector.runOut(collectorOutButton);
	        collector.runInTime(collectorOneInButton, collectorOneInTime);
	        collector.runInTime(collectorTwoInButton, collectorTwoInTime);
	        collector.runOutTime(collectorOneOutButton, collectorOneOutTime);
	        collector.runOutTime(collectorTwoOutButton, collectorTwoOutTime);
	        collector.runIn(collectorIn);
	        collector.runOut(collectorOut);
	        collector.run(collectorInAndOut);
	
	        collector.run();
	    }
	    if(enabledDevices.get("lift")){
	        lift.setGamepad(gamepad2);
	
	        lift.retractFully(liftRetractFullyButton);
	        lift.extendFully(liftExtendFullyButton);
	        lift.manualOverrideRetract(liftRetractButton);
	        lift.manualOverrideExtend(liftExtendButton);
	        lift.manualOverrideRetract(liftDown);
	        lift.manualOverrideExtend(liftUp);
	        lift.manualOverride(liftJoystick);
	        
	        lift.run();
	    }
	    if(armEnabled()){
            arm.setGamepad(gamepad2);

            arm.setManualOverride(armSetOverride);
            arm.setManualOverrideElbow(armSetOverrideElbow);
            arm.setManualOverrideShoulder(armSetOverrideShoulder);
            arm.manualOverride(armOverrideShoulder, armOverrideElbow);

            if(arm.shoulderMotor.getCurrentPower() != 0) {
                arm.shoulderMotor.correctedSetPower(armShoulderJoystickC, armShoulderPowerCorrection, armShoulderCorrectionRange);
            }
            if(arm.elbowMotor.getCurrentPower() != 0) {
                arm.elbowMotor.correctedSetPower(armElbowJoystickC, armElbowPowerCorrection, armElbowCorrectionRange);
            }

            boolean manualControlShoulder = arm.shoulderMotor.getCurrentPower() != 0 || arm.manualOverrideShoulder;
            boolean manualControlElbow = arm.elbowMotor.getCurrentPower() != 0 || arm.manualOverrideElbow;
            
            arm.runToPosition(armCraterExtension, craterExtensionTicks, new double[]{0.5, 0.5}, 
            		manualControlShoulder, manualControlElbow);
            arm.runToPosition(armDrivePosition, drivePositionTicks, new double[]{0.5, 0.5}, 
            		manualControlShoulder, manualControlElbow);
            arm.runToPosition(armFoldUp, foldUpTicks, new double[]{0.5, 0.5}, 
            		manualControlShoulder, manualControlElbow);
            arm.runToPosition(armLanderExtension, landerExtensionTicks, new double[]{0.5, 0.5}, 
            		manualControlShoulder, manualControlElbow);

            arm.run();
        }
    }
    
    public void turnOnFlash(){System.out.println("Flash!");}
    public void turnOffFlash() {System.out.println("Unflash!");}
    
    public AutonomousNavigationSim nav;
    public int goldLocation;
    public String goldLocationString = "";
    public ArrayList<Integer> masses = new ArrayList<Integer>();
    public RobotAction detectSamples = new RobotAction(){
        final int frameNum = 1;
        int frameCount = 0;
        //SampleResult[] results = new SampleResult[frameNum];
        ElapsedTime timer = new ElapsedTime();
        
        @Override
        public boolean start(){
            turnOnFlash();
            timer.reset();
            return true;
        }

        @Override
        public boolean act(){
        	if(timer.seconds() > 1) {
	            /*SampleResult currResult = FtcRobotControllerActivity.scanSamples();
	            results[frameCount] = currResult;*/
	            frameCount++;
	
	            if(frameCount >= frameNum){
	            	//SampleResult sampleOrientation = SampleResult.combineResults(results);
                    goldLocation = 2;//sampleOrientation.getSampleNum(SampleResult.SampleType.GOLD);
                    //masses.add(sampleOrientation.masses[0][1][0]);
                    //masses.add(sampleOrientation.masses[1][1][0]);
                    masses.add((int) Math.round(Math.random()*10));
                    masses.add((int) Math.round(Math.random()*100));

                    goldLocationString = goldLocationToString(goldLocation);
                    nav.targets.set(2, goldLocationToTarget(goldLocation));
                    nav.targets.set(3, new VectorF(39, nav.targets.get(2).get(1)));

	                turnOffFlash();
	                return true;
	            }
        	}
            return false;
        }

        private String goldLocationToString(int i){
            switch(i){
                case 0:
                    return "LEFT";
                case 1:
                    return "MIDDLE";
                case 2:
                    return "RIGHT";
                default:
                    return "ERROR!";
            }
        }
        private VectorF goldLocationToTarget(int i){
            return new VectorF(51, 14.5f*(1-i));
        }
    };
    
    //test of lowering the robot to the ground
    public RobotAction lowerRobotTest = new RobotAction(){

        ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean start(){
            timer.reset();
            return true;
        }

        @Override
        public boolean act(){
            lift.retractFully(1);
            if(groundRange.getData(DistanceUnit.INCH) <= inchesToGround){
                lift.stop();
                return true;
            }
            lift.run();
            return false;
        }
    };
    //lowers the robot to the ground
    public RobotAction lowerRobot = new RobotAction(){
        @Override
        public boolean act(){
            return true;
        }
    };
    //drops the team marker in the depot
    public RobotAction dropMarker = new RobotAction(){
        @Override
        public boolean act(){
            return true;
        }
    };
    //extends the arm so that the collector is over the depot
    public RobotAction extendArm = new RobotAction(){
        @Override
        public boolean act(){
            return true;
        }
    };
    //collapse the arm so it doesn't get in the way of another robot trying to place their team marker
    public RobotAction collapseArm = new RobotAction(){
        @Override
        public boolean act(){
            return true;
        }
    };
    //places the team marker in the depot
    public RobotAction placeTeamMarker = new RobotActionLinear(extendArm, dropMarker, collapseArm);
    
    
    @Override
    public void stop(){
    	if(!stopped) {
	        drive.stop();
	        lift.stop();
	        collector.stop();
	        arm.stop();
	
	        
	        DataLoggerSim loggerH = new DataLoggerSim("Heading Comp");
	        Angle heading = new Angle(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, AngleUnit.DEGREES);
	
	        loggerH.addField(heading.getDegree());
	        loggerH.newLine();
	        loggerH.closeDataLogger();
	        
	        
	        DataLoggerSim loggerM = new DataLoggerSim("Encoder Comp");
	        
	        /*int liftTicks = lift.getCurrentPosition();
	        int shoulderTicks = arm.shoulderMotor.getCurrentPosition();
	        int elbowTicks = arm.elbowMotor.getCurrentPosition();*/
	        int liftTicks = (int) Math.round(Math.random()*1000);
	        int shoulderTicks = (int) Math.round(Math.random()*300);
	        int elbowTicks = (int) Math.round(-Math.random()*300);
	        System.out.println(liftTicks+" "+shoulderTicks+" "+elbowTicks);
	        loggerM.addField(liftTicks);
	        loggerM.newLine();
	
	        loggerM.addField(shoulderTicks);
	        loggerM.newLine();
	        
	        loggerM.addField(elbowTicks);
	        loggerM.newLine();
	
	        loggerM.closeDataLogger();
	        
	        stopped = true;
    	}
    }

    public static int[] processEncoderFile() throws IOException{
        int[] ticks = new int[3];
        FileReader fr = new FileReader("F:/DataLogger/Encoder Comp.txt");
        String str = "";
        int i;

        while((i=fr.read()) != -1) {
            str+=(char)i;
        }
        fr.close();
        String[] lines = str.split("\n");

        ticks[0] = Integer.parseInt(lines[0]);
        ticks[1] = Integer.parseInt(lines[1]);
        ticks[2] = Integer.parseInt(lines[2]);
        
        return ticks;
    }
    public enum RoverCompPrograms implements Program{
        TELE_OP{@Override public OpModeType getOpModeType() { return OpModeType.TELE; }},
        DEPOT{@Override public OpModeType getOpModeType(){return OpModeType.AUTO;}},
        CRATER{@Override public OpModeType getOpModeType(){return OpModeType.AUTO;}},
    }

    public enum RoverTestPrograms implements Program{
        DRIVE{@Override public OpModeType getOpModeType() { return OpModeType.TELE; }},
        DRIVE_ABS{@Override public OpModeType getOpModeType() { return OpModeType.TELE; }},
        DRIVE_ABS_CORRECTIVE{@Override public OpModeType getOpModeType(){ return OpModeType.TELE; }},
        LOWER{@Override public OpModeType getOpModeType(){ return OpModeType.TELE; }}
    }
}
