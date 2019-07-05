package org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Sensor;

import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.teamcalamari.StateMachine.Events.Event;
import org.firstinspires.ftc.teamcode.teamcalamari.TCHardware.Sensors.TouchSensorTC;

public class TouchSensorEvent extends Event<Boolean> {
    private TouchSensorTC sensor;
    private Boolean value;

    public TouchSensorEvent(String name, TouchSensorTC sensor){
        super(name);
        this.sensor = sensor;
    }
    public TouchSensorEvent(String name, TouchSensor sensor){
        this(name, new TouchSensorTC(sensor));
    }
    public TouchSensorEvent(String name, Boolean value){
        super(name);
        this.value = value;
    }

    @Override
    public Boolean getValue() {
        if(sensor != null) return sensor.sensor.isPressed();
        return value;
    }

    @Override
    public Event<Boolean> copy() {
        if(sensor != null) return new TouchSensorEvent(name, sensor);
        return new TouchSensorEvent(name, value);
    }
}
