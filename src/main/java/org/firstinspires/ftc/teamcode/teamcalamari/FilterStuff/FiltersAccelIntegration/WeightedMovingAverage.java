package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FiltersAccelIntegration;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

public class WeightedMovingAverage extends AccelFilter {
	
	int[] sampleSize;
	double[] filtValues;
	
	public WeightedMovingAverage(int[] ss) {
		sampleSize = new int[3];
		filtValues = new double[3];
		for(int i = 0; i < 3; i++) {
			sampleSize[i] = ss[i];
		}
	}

	@Override
	public Acceleration run() {
		int measNum = values.size()-1;
		for(int i = 0; i < 3; i++) {
			filtValues[i] = 0.0;
			
			int max = Math.max(0, measNum-sampleSize[i]+1);
			for(int j = max; j < measNum+1; j++){
				filtValues[i]+=(j+1-max)*(AccelAxisFromInt(values.get(j), i));
			}
			filtValues[i]/=((Math.pow(Math.min(sampleSize[i], measNum+1), 2)+Math.min(sampleSize[i], measNum+1))/2);
		}
		return DoubleArrayToAccel(filtValues, currValue.acquisitionTime);
	}

}
