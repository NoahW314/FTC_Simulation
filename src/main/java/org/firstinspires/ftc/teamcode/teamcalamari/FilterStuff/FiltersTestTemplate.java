package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff;

import java.awt.Color;
import java.awt.Dimension;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters.Filter;
import org.firstinspires.ftc.teamcode.teamcalamari.Tests.CompareAutos.Accelerometer.Accelerometer;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.ui.ApplicationFrame;

public abstract class FiltersTestTemplate<F extends Filter<?>> extends ApplicationFrame {
	
	public F filter;
	public ArrayList<Double[]> filteredAccelerations = new ArrayList<Double[]>();
	public double[] position;
	
	private static final long serialVersionUID = 2315131740376413437L;
	public FiltersTestTemplate(String title, String chartTitle, F filt, String filePath, int i) throws IOException {
		super(title);
		filter = filt;
		XYDataset[] datasets = createDataset(filePath);
		JFreeChart chart = ChartFactory.createXYLineChart(
			chartTitle+" "+i,
			"Time","Acceleration",
			datasets[i],
			PlotOrientation.VERTICAL,
			true,true,false);
      
		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new Dimension(1250, 900));
		final XYPlot plot = chart.getXYPlot();
		plot.setBackgroundPaint(Color.WHITE);
		plot.setDomainGridlinePaint(Color.BLACK);
		plot.setRangeGridlinePaint(Color.BLACK);
  
		XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();
		renderer.setSeriesPaint(0, Color.PINK);
		renderer.setSeriesPaint(1, Color.RED);
		/*renderer.setSeriesPaint(2, Color.GRAY);
		renderer.setSeriesPaint(3, Color.BLACK);
		renderer.setSeriesPaint(4, Color.CYAN);
		renderer.setSeriesPaint(5, Color.BLUE);*/
		renderer.setSeriesShapesVisible(0, false);
		renderer.setSeriesShapesVisible(1, false);
		/*renderer.setSeriesShapesVisible(2, false);
		renderer.setSeriesShapesVisible(3, false);
		renderer.setSeriesShapesVisible(4, false);
		renderer.setSeriesShapesVisible(5, false);*/
		plot.setRenderer(renderer); 
		setContentPane(chartPanel);
		
		
		if(i == 0) {
			position = calculatePosition(filteredAccelerations);
			double[] positionInches = metersToInches(position);
			Position pose = Accelerometer.correctPosition(new Position(DistanceUnit.INCH, positionInches[0], positionInches[1], positionInches[2], 0));
			System.out.println(pose.toString()+" "+title);
		}
	}
	
	private double[] metersToInches(double[] position) {
		return new double[] {metersToInches(position[0]), metersToInches(position[1]), metersToInches(position[2])};
	}
	private double metersToInches(double position) {return position*100/2.54;}
	
	private double[] calculatePosition(ArrayList<Double[]> filtAccel) {
		double[] currAccel = new double[3];
		double[] prevAccel = new double[3];
		double[] currVelo = new double[] {0,0,0};
		double[] prevVelo = new double[] {0,0,0};
		double[] position = new double[] {0,0,0};
		
		for(int i = 0; i < filtAccel.size()-1; i++) {
			for(int j = 0; j < 3; j++) {
				double time = filtAccel.get(i+1)[0];
				
				prevAccel[j] = filtAccel.get(i)[j+1];
				currAccel[j] = filtAccel.get(i+1)[j+1];
				
				currVelo[j]+=((prevAccel[j]+currAccel[j])*time/2);
				
				position[j]+=((prevVelo[j]+currVelo[j])*time/2);
				
				prevVelo[j] = currVelo[j];
			}
		}
		return position;
	}
	
	
	protected abstract XYDataset[] createDataset(String filePath) throws IOException;


	protected ArrayList<Double[]> parseFile(String filePath) throws IOException {
		FileReader fr = new FileReader(filePath);
		String text = readAllLines(fr);
		ArrayList<String> lines = new ArrayList<String>(Arrays.asList(text.split("\n")));
		lines.remove(0);
		
		ArrayList<Double[]> accelerations = new ArrayList<Double[]>();
		
		for(String str : lines){
			String[] measurements = str.split(",");
			accelerations.add(stringsToDoubles(measurements));
		}
		
		return accelerations;
	}
	private Double[] stringsToDoubles(String[] strings) {
		Double[] doubles = new Double[strings.length];
		for(int j = 0; j < strings.length; j++) {
			doubles[j] = Double.parseDouble(strings[j]);
		}
		return doubles;
	}
	private static String readAllLines(FileReader fr) throws IOException {
		String str = "";
		int i;
		while((i=fr.read()) != -1) {
			str+=(char)i;
		}
		return str;
	}
}
