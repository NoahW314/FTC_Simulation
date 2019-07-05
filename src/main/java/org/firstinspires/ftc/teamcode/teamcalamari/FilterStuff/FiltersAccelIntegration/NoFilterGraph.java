package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FiltersAccelIntegration;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

public class NoFilterGraph extends AccelFilter {

	@Override
	public Acceleration run() {
		currValue.yAccel+=5;
		currValue.zAccel+=10;
		return currValue;
	}

}
