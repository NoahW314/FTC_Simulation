package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FilterTestsAccel;

import java.io.File;
import java.io.IOException;

import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FilePath;
import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FiltersTest;
import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters.WeightedMovingAverage;
import org.jfree.ui.RefineryUtilities;

public class WeightedMovingAverageTest extends FiltersTest<WeightedMovingAverage> {

	private static final long serialVersionUID = 3335273766621691777L;
	
	public static void main(String[] args) throws IOException {
		//still 1,1,1
		//1 nav 1,1,1
		int[] sampleSize = new int[] {5, 5, 5};
		File folder = new File(FilePath.folder);
		for(File file : folder.listFiles()) {
			if(file.isDirectory()) break;
			for(int i = 0; i < 2; i++) {
				WeightedMovingAverageTest chart = new WeightedMovingAverageTest("SMA Test", "SMA", 
						new WeightedMovingAverage(sampleSize), FilePath.folder+"\\"+file.getName(), i);
				
				chart.pack();
				RefineryUtilities.centerFrameOnScreen(chart);
				chart.setVisible(true);
			}
		}
	}

	public WeightedMovingAverageTest(String title, String chartTitle, WeightedMovingAverage filt, String filePath, int i) throws IOException {
		super(title, chartTitle, filt, filePath, i);
	}

}
