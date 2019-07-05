package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters;

import java.util.ArrayList;

public class NoFilter implements Filter<Double[]> {

	@Override
	public Double[] run(ArrayList<Double[]> values, int measNum) {
		Double[] currValues = values.get(measNum);
		return new Double[] {currValues[1], currValues[2], currValues[3]};
	}

}
