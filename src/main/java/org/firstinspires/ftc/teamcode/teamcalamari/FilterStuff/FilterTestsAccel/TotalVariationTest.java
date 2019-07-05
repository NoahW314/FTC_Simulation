package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FilterTestsAccel;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FilePath;
import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FiltersTestTemplate;
import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters.TotalVariationFilter;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.RefineryUtilities;

public class TotalVariationTest extends FiltersTestTemplate<TotalVariationFilter> {
	
	public static void main(String[] args) throws IOException {
		//still 50,50,50
		double[] smoothing = new double[] {50,50,50};
		File folder = new File(FilePath.folder);
		for(File file : folder.listFiles()) {
			if(file.isDirectory()) break;
			for(int i = 0; i < 2; i++) {
				TotalVariationTest chart = new TotalVariationTest("TV Test", "TV", 
						new TotalVariationFilter(smoothing), FilePath.folder+"\\"+file.getName(), i);
				
				chart.pack();
				RefineryUtilities.centerFrameOnScreen(chart);
				chart.setVisible(true);
			}
		}
	}

	private static final long serialVersionUID = 4436600638624381763L;

	public TotalVariationTest(String title, String chartTitle, TotalVariationFilter filt, String filePath, int i) throws IOException {
		super(title, chartTitle, filt, filePath, i);
	}
	
	@Override
	public XYDataset[] createDataset(String filePath) throws IOException {
		final XYSeries filteredX = new XYSeries("FX");
		final XYSeries filteredY = new XYSeries("FY");
		final XYSeries filteredZ = new XYSeries("FZ");
		final XYSeries x = new XYSeries("X");
		final XYSeries y = new XYSeries("Y");
		final XYSeries z = new XYSeries("Z");
		
		ArrayList<Double[]> acceleration = parseFile(filePath);
		
		double time = 0;
		double pTime = acceleration.get(0)[0]/(Math.pow(10, 9));
		double startTime = pTime;
		for(int i = 0; i < acceleration.size(); i++) {
			time = acceleration.get(i)[0]/(Math.pow(10, 9));
			x.add(time-startTime, acceleration.get(i)[1]);
			y.add(time-startTime, acceleration.get(i)[2]);
			z.add(time-startTime, acceleration.get(i)[3]);
		}
		
		time = 0;
		int times = 1;
		filter.setIterations(times);
		ArrayList<Double> filtx = filter.run(acceleration, acceleration.size(), 0);
		ArrayList<Double> filty = filter.run(acceleration, acceleration.size(), 1);
		ArrayList<Double> filtz = filter.run(acceleration, acceleration.size(), 2);
		for(int i = 0; i < acceleration.size(); i++) {
			time = acceleration.get(i)[0]/(Math.pow(10, 9));
			filteredX.add(time-startTime, filtx.get(i));
			filteredY.add(time-startTime, filty.get(i));
			filteredZ.add(time-startTime, filtz.get(i));
			filteredAccelerations.add(new Double[] {time-pTime, filtx.get(i), filty.get(i), filtz.get(i)});
			pTime = time;
		}
		
		final XYSeriesCollection[] datasets = new XYSeriesCollection[] {new XYSeriesCollection(), new XYSeriesCollection(), new XYSeriesCollection()};
		datasets[0].addSeries(x);  
		datasets[0].addSeries(filteredX);
		datasets[1].addSeries(y);
		datasets[1].addSeries(filteredY);
		datasets[2].addSeries(z);
		datasets[2].addSeries(filteredZ);
		
	    return datasets;
	}

}
