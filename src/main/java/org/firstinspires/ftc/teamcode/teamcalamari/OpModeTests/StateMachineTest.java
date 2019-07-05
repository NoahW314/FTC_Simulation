package org.firstinspires.ftc.teamcode.teamcalamari.OpModeTests;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.function.Supplier;
import org.firstinspires.ftc.teamcode.teamcalamari.OpModeSim.OpMode;
import org.firstinspires.ftc.teamcode.teamcalamari.RobotAction.RobotActionMotor;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.MotorRestrictedSupplier;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.StateMachine;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.StateMachine.State;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.StateMachine.StateTransition;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.StateMachineUtils;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.TimeEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadButtonEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Gamepad.GamepadEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Sensor.TouchSensorEvent;
import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.States.MotorState;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadButton;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadJoystick;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Gamepad.GamepadMotion;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Motors.Motor;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors.TouchSensorTC;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="State Machine Test")
public class StateMachineTest extends OpMode {

    private Motor m;
    private TouchSensorTC forwardLimit;

    private GamepadMotion overrideControl = GamepadJoystick.RIGHT_STICK_Y;
    private GamepadMotion joystickControl = GamepadJoystick.LEFT_STICK_Y;
    private GamepadButtonEvent switchJoystickControl = GamepadButton.RIGHT_BUMPER.getEvent();
    private GamepadButtonEvent runForward = GamepadButton.B.getEvent();
    private GamepadButtonEvent runBackward = GamepadButton.X.getEvent();
    private GamepadButtonEvent stopButton = GamepadButton.A.getEvent();
    private GamepadButtonEvent runForwardTime = GamepadButton.Y.getEvent();

    private StateMachine stateMachine;
    private StateMachine stateMachine2;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        m = new Motor("motor", hardwareMap);
        forwardLimit = new TouchSensorTC(hardwareMap.touchSensor.get("forwardLimit"));

        StateTransition toStop = new StateTransition(stopButton, "stop");
        StateTransition toForward = new StateTransition(runForward, "forward");
        StateTransition toBackward = new StateTransition(runBackward, "backward");
        StateTransition toOverride = new StateTransition(overrideControl.getDetectedEvent(), overrideControl.getString());
        StateTransition fromOverride = new StateTransition(overrideControl.getNotDetectedEvent(), "stop");
        StateTransition toJoystickControl = new StateTransition(switchJoystickControl, "left_stick_y");
        StateTransition fromJoystickControl = new StateTransition(switchJoystickControl, "stop");
        StateTransition toForwardTime = new StateTransition(runForwardTime, "forward_time");
        StateTransition fromForwardTime = new StateTransition(new TimeEvent(2), "stop");

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
        timer.reset();
        stateMachine.start("stop");
        stateMachine2.start("normal");
    }

    @Override
    public void loop() {
        List<GamepadEvent> gamepadEvents = StateMachineUtils.generateGamepadEvents(gamepad1);
        for(GamepadEvent e : gamepadEvents) {
            stateMachine.handleEvent(e);
        }
        stateMachine.handleEvent(new TimeEvent(timer.seconds()));
        stateMachine.run();

        stateMachine2.handleEvent(new TouchSensorEvent("forwardLimit", forwardLimit));
        stateMachine2.run();

        m.run();
    }
}
