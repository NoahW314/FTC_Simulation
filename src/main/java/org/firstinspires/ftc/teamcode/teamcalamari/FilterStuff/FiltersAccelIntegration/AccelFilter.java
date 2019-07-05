package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FiltersAccelIntegration;

import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.meanIntegrate;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.plus;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public abstract class AccelFilter extends BasicAccelFilter {
	
	@Override
	public void update(Acceleration linearAcceleration) {
		currValue = linearAcceleration;
		values.add(linearAcceleration);
		Acceleration acceleration = run();
		filtValues.add(acceleration);
		
		if (prevAccel != null && prevAccel.acquisitionTime != 0){
	        Velocity deltaVelocity = meanIntegrate(acceleration, prevAccel);
	        velocity = plus(velocity, deltaVelocity);
		}
		if (prevVelocity != null &&  prevVelocity.acquisitionTime != 0){
	        Position deltaPosition = meanIntegrate(velocity, prevVelocity);
	        position = plus(position, deltaPosition);
        }
		
		prevAccel = acceleration;
		prevVelocity = velocity;
	}
	public abstract Acceleration run();
}
