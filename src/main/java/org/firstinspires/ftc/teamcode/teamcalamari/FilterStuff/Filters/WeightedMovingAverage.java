package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters;

import java.util.ArrayList;

public class WeightedMovingAverage implements Filter<Double[]> {
	
	int[] sampleSize;
	Double[] filtValues;
	
	public WeightedMovingAverage(int[] ss) {
		sampleSize = new int[ss.length];
		filtValues = new Double[ss.length];
		for(int i = 0; i < ss.length; i++) {
			sampleSize[i] = ss[i];
		}
	}

	@Override
	public Double[] run(ArrayList<Double[]> values, int measNum) {
		for(int i = 0; i < sampleSize.length; i++) {
			filtValues[i] = 0d;
			
			int max = Math.max(0, measNum-sampleSize[i]+1);
			for(int j = max; j < measNum+1; j++){
				filtValues[i]+=(j+1-max)*(values.get(j)[i+1]);
			}
			filtValues[i]/=((Math.pow(Math.min(sampleSize[i], measNum+1), 2)+Math.min(sampleSize[i], measNum+1))/2);
		}
		return filtValues;
	}

}
