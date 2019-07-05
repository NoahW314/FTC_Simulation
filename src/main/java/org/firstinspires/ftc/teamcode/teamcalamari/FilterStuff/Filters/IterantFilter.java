package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters;

public abstract class IterantFilter<D> implements Filter<D> {
	public int[] iterations = new int[3];
	public void setIterations(int times) {
		for(int i = 0; i < 3; i++) {
			iterations[i] = times;
		}
	}
	public void setIterations(int[] times) {
		iterations = times;
	}
}
