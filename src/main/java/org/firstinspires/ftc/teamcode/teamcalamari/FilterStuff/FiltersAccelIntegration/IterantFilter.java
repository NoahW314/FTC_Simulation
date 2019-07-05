package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FiltersAccelIntegration;

public abstract class IterantFilter extends BasicAccelFilter {
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
