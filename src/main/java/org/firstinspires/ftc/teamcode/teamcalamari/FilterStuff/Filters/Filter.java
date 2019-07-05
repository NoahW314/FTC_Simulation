package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters;

import java.util.ArrayList;

public interface Filter<D> {
	public D run(ArrayList<Double[]> values, int measNum);
}