package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FiltersAccelIntegration;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class NoFilter extends AccelFilter {

	public void setVelocity(Velocity velo) {
		this.velocity = velo;
	}
	
	@Override
	public Acceleration run() {
		return currValue;
	}

}
