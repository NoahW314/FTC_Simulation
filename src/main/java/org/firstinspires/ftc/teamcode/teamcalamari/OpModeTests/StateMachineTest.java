package org.firstinspires.ftc.teamcode.teamcalamari.OpModeTests;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.function.Supplier;
import org.firstinspires.ftc.teamcode.teamcalamari.Range;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeSim.OpMode;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction.RobotActionMotor;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.MotorRestrictedSupplier;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.StateMachine;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.StateMachine.State;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.StateMachine.StateTransition;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.StateMachineUtils;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.TimeEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadButtonClickedEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadButtonEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadEvent;
import static org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadEvent.GamepadNumber.G1;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Sensor.TouchSensorEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.States.MotorState;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadButton;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadJoystick;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadMotion;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.Motor;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors.TouchSensorSimpleTC;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="State Machine Test")
public class StateMachineTest extends OpMode {

    private Motor m;
    private TouchSensorSimpleTC forwardLimit;
    private GamepadButton sensorLimitButton = GamepadButton.LEFT_BUMPER;

    private GamepadMotion overrideControl = GamepadJoystick.RIGHT_STICK_Y;
    private GamepadMotion joystickControl = GamepadJoystick.LEFT_STICK_Y;
    private GamepadButtonClickedEvent switchJoystickControl = GamepadButton.RIGHT_BUMPER.getClickedEvent(G1, true);
    private GamepadButtonEvent runForward = GamepadButton.B.getEvent(G1);
    private GamepadButtonEvent runBackward = GamepadButton.X.getEvent(G1);
    private GamepadButtonEvent stopButton = GamepadButton.A.getEvent(G1);
    private GamepadButtonEvent runForwardTime = GamepadButton.Y.getEvent(G1);

    private StateMachine stateMachine;
    private StateMachine stateMachine2;

    private ElapsedTime forwardTimer = new ElapsedTime();
    
    private StateTransition toStop = new StateTransition(stopButton, "stop");
    private StateTransition toForward = new StateTransition(runForward, "forward");
    private StateTransition toBackward = new StateTransition(runBackward, "backward");
    private StateTransition toOverride = new StateTransition(overrideControl.getDetectedEvent(G1), overrideControl.getString());
    private StateTransition fromOverride = new StateTransition(overrideControl.getNotDetectedEvent(G1), "stop");
    private StateTransition toJoystickControl = new StateTransition(switchJoystickControl, "left_stick_y");
    private StateTransition fromJoystickControl = new StateTransition(switchJoystickControl, "stop");
    private StateTransition toForwardTime = new StateTransition(runForwardTime, "forward_time");
    private StateTransition fromForwardTime = new StateTransition(new TimeEvent("forward_time", new Range<Double>(2d, true)), "stop");

    @Override
    public void init() {
        m = new Motor("motor", hardwareMap);
        forwardLimit = new TouchSensorSimpleTC(hardwareMap.touchSensor.get("forwardLimit"));
        forwardLimit.linkWithGamepad(gamepad1, sensorLimitButton);

        State joystickControlling = new MotorState(joystickControl.getString(), m, new Supplier<Double>(){
            @Override
            public Double get() {
                return Double.valueOf(joystickControl.getValue(gamepad1));
            }
        }, fromJoystickControl, toOverride);
        State joystickOverride = new MotorState(overrideControl.getString(), new RobotActionMotor(m, new Supplier<Double>() {
            @Override
            public Double get() {
                return Double.valueOf(overrideControl.getValue(gamepad1));
            }
        }, true), fromOverride);
        State forwardFull = new MotorState("forward", m, StateMachineUtils.getSupplier(1d),
                toBackward, toStop, toOverride, toJoystickControl);
        State backwardFull = new MotorState("backward", m, StateMachineUtils.getSupplier(-1d),
                toForward, toStop, toOverride, toJoystickControl);
        State stop = new MotorState("stop", m, StateMachineUtils.getSupplier(0d),
                toForward, toBackward, toOverride, toJoystickControl, toForwardTime);
        State forwardTime = new MotorState("forward_time", m, StateMachineUtils.getSupplier(1d),
        		new Runnable() {public void run() {
        			System.out.println("Reset Timer");
        			forwardTimer.reset();
        			}},
                fromForwardTime, toOverride);

        stateMachine = new StateMachine(joystickControlling, joystickOverride, forwardFull, backwardFull, stop, forwardTime);


        StateTransition fromRestricted = new StateTransition(new TouchSensorEvent("forwardLimit", false), "normal");
        StateTransition toRestricted = new StateTransition(new TouchSensorEvent("forwardLimit", true), "restrictedForward");
        State restrictedForward = new MotorState("restrictedForward",
                new RobotActionMotor(m, new MotorRestrictedSupplier(m, DcMotorSimple.Direction.FORWARD), true),
                fromRestricted);
        State normal = new MotorState("normal", null, toRestricted);

        stateMachine2 = new StateMachine(normal, restrictedForward);
    }

    @Override
    public void start(){
        forwardTimer.reset();
        stateMachine.start("stop");
        stateMachine2.start("normal");
    }

    @Override
    public void loop() {
        List<GamepadEvent<?, ?>> gamepadEvents = StateMachineUtils.generateGamepadEvents(gamepad1, G1);
        for(GamepadEvent<?, ?> e : gamepadEvents) {
            stateMachine.handleEvent(e);
        }
        stateMachine.handleEvent(new TimeEvent("forward_time", forwardTimer.seconds()));
        stateMachine.run();

        stateMachine2.handleEvent(new TouchSensorEvent("forwardLimit", forwardLimit));
        stateMachine2.run();

        m.run();
        
        ((GamepadButtonClickedEvent)toJoystickControl.getEvent()).assignPrevValue(gamepad1);
        ((GamepadButtonClickedEvent)fromJoystickControl.getEvent()).assignPrevValue(gamepad1);
        
        
        //telemetry.addData("Right Bumper", GamepadButton.RIGHT_BUMPER.getValue(gamepad1));
        //telemetry.addData("Y", GamepadButton.Y.getValue(gamepad1));
        //telemetry.addData("Time", forwardTimer.seconds());
        telemetry.addData("State", stateMachine.getCurrentState());
        telemetry.addData("State 2", stateMachine2.getCurrentState());
    }
}
