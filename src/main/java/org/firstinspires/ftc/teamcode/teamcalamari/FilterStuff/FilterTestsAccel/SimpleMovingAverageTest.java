package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FilterTestsAccel;

import java.io.File;
import java.io.IOException;

import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FilePath;
import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FiltersTest;
import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters.SimpleMovingAverage;
import org.jfree.ui.RefineryUtilities;

public class SimpleMovingAverageTest extends FiltersTest<SimpleMovingAverage> {

	private static final long serialVersionUID = -172886545774556561L;

	public static void main(String[] args) throws IOException {
		//still 1,1,1
		//1 nav 1,1,1
		int[] sampleSize = new int[] {5, 5, 5};
		File folder = new File(FilePath.folder);
		for(File file : folder.listFiles()) {
			if(file.isDirectory()) break;
			for(int i = 0; i < 2; i++) {
				SimpleMovingAverageTest chart = new SimpleMovingAverageTest("SMA Test", "SMA", 
						new SimpleMovingAverage(sampleSize), FilePath.folder+"\\"+file.getName(), i);
				
				chart.pack();
				RefineryUtilities.centerFrameOnScreen(chart);
				chart.setVisible(true);
			}
		}
	}
	
	public SimpleMovingAverageTest(String title, String chartTitle, SimpleMovingAverage filter, String filePath, int i) throws IOException {
		super(title, chartTitle, filter, filePath, i);
	}

}
