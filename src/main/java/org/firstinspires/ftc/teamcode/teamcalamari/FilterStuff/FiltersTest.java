package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff;

import java.io.IOException;
import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters.Filter;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public abstract class FiltersTest<F extends Filter<Double[]>> extends FiltersTestTemplate<F> {
	
	private static final long serialVersionUID = -8985121128668181591L;

	/*public static void main(String[] args) throws IOException {
      FiltersTest chart = new FiltersTest(
         "Graph Test" ,
         "Graph");

      chart.pack();
      RefineryUtilities.centerFrameOnScreen(chart);
      chart.setVisible(true);
   }*/

	public FiltersTest(String title, String chartTitle, F filt, String filePath, int i) throws IOException {
		super(title, chartTitle, filt, filePath, i);
	}
	
	public XYDataset[] createDataset(String filePath) throws IOException {
		final XYSeries filteredX = new XYSeries("FX");
		final XYSeries filteredY = new XYSeries("FY");
		final XYSeries filteredZ = new XYSeries("FZ");
		final XYSeries x = new XYSeries("X");
		final XYSeries y = new XYSeries("Y");
		final XYSeries z = new XYSeries("Z");
		final XYSeries heading = new XYSeries("Heading");
		
		ArrayList<Double[]> acceleration = parseFile(filePath);
		
		double time = 0;
		double pTime = acceleration.get(0)[0]/(Math.pow(10, 9));
		double startTime = pTime;
		for(int i = 0; i < acceleration.size(); i++) {
			time = acceleration.get(i)[0]/(Math.pow(10, 9));
			Double[] f = filter.run(acceleration, i);
			double fX = f[0];
			double fY = f[1];
			double fZ = f[2];
			filteredAccelerations.add(new Double[]{time-pTime, fX, fY, fZ});
			filteredX.add(time-startTime, fX);
			filteredY.add(time-startTime, fY);
			filteredZ.add(time-startTime, fZ);
			x.add(time-startTime, acceleration.get(i)[1]);
			y.add(time-startTime, acceleration.get(i)[2]);
			z.add(time-startTime, acceleration.get(i)[3]);
			heading.add(time-startTime, acceleration.get(i)[4]);
			pTime = time;
		}
		
		final XYSeriesCollection[] datasets = new XYSeriesCollection[] {new XYSeriesCollection(), new XYSeriesCollection(), new XYSeriesCollection(), new XYSeriesCollection()};
		datasets[0].addSeries(x);
		datasets[0].addSeries(filteredX);
		datasets[1].addSeries(y);
		datasets[1].addSeries(filteredY);
		datasets[2].addSeries(z);
		datasets[2].addSeries(filteredZ);
		datasets[3].addSeries(heading);
		
	    return datasets;
	}
}
