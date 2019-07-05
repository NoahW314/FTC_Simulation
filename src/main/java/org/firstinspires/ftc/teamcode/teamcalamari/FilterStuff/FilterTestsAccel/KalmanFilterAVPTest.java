package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FilterTestsAccel;

import java.io.File;
import java.io.IOException;

import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FilePath;
import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FiltersTest;
import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters.KalmanFilterAVP;
import org.jfree.ui.RefineryUtilities;

public class KalmanFilterAVPTest extends FiltersTest<KalmanFilterAVP> {

	public static void main(String[] args) throws IOException {
		//space holder for time  (after all time and space are related so why not use one for the other ;-)
		double t = 0;
		File folder = new File(FilePath.folder);
		for(File file : folder.listFiles()) {
			if(file.isDirectory()) break;
			for(int i = 0; i < 2; i++) {
				KalmanFilterAVPTest chart = new KalmanFilterAVPTest("Kalman Test", "Kalman", 
						new KalmanFilterAVP(new double[][] {{1, 0, 0}, {t, 1, 0}, {0.5*Math.pow(t, 2), t, 1}},
								new double[][] {{1, 0, 0}},
								//still 0,0,0, 0,0,0, 0,0,0
								//1 nav 0,0,0, 0,0,0, 0,0,0
								new double[][] {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
								//still 0.0001/12
								//1 nav 0.0001/12
								new double[] {0.0001/12},
								new double[][] {{0}, {0}, {0}}), FilePath.folder+"\\"+file.getName(), i);
				
				chart.pack();
				RefineryUtilities.centerFrameOnScreen(chart);
				chart.setVisible(true);
			}
		}
	}
	
	private static final long serialVersionUID = 8225195695128160787L;

	public KalmanFilterAVPTest(String title, String chartTitle, KalmanFilterAVP filt, String filePath, int i) throws IOException {
		super(title, chartTitle, filt, filePath, i);
	}

}
