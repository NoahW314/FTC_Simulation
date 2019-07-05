package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FilterTestsAccel;

import java.io.File;
import java.io.IOException;

import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FilePath;
import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FiltersTest;
import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters.ExponentialMovingAverage;
import org.jfree.ui.RefineryUtilities;

public class ExponentialMovingAverageTest extends FiltersTest<ExponentialMovingAverage> {
	
	private static final long serialVersionUID = 4962163293350402068L;

	public ExponentialMovingAverageTest(String title, String chartTitle, ExponentialMovingAverage filt, String filePath, int i) throws IOException {
		super(title, chartTitle, filt, filePath, i);
	}

	public static void main(String[] args) throws IOException {
		//still 1,1,1
		//1 nav 1,0,1 
		double[] smoothingFactor = new double[] {1, 0, 1};
		File folder = new File(FilePath.folder);
		for(File file : folder.listFiles()) {
			if(file.isDirectory()) break;
			for(int i = 0; i < 2; i++) {
				ExponentialMovingAverageTest chart = new ExponentialMovingAverageTest("EMA Test", "EMA", 
						new ExponentialMovingAverage(smoothingFactor), FilePath.folder+"\\"+file.getName(), i);
				
				chart.pack();
				RefineryUtilities.centerFrameOnScreen(chart);
				chart.setVisible(true);
			}
		}
	}
}
