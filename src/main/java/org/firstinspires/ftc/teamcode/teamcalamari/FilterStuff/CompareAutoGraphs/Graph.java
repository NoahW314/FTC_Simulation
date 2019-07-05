package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.CompareAutoGraphs;

import java.awt.Color;
import java.awt.Dimension;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

import org.firstinspires.ftc.teamcode.teamcalamari.Angle;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.ApplicationFrame;
import org.jfree.ui.RefineryUtilities;

public class Graph extends ApplicationFrame {

	private static final long serialVersionUID = -6678282680836893049L;
	
	public static void main(String[] args) throws IOException {
		File folder = new File(DataPath.folder);
		for(File file : folder.listFiles()) {
			if(file.isDirectory()) break;
			Graph graph = new Graph(file.getName(), DataPath.folder+"\\"+file.getName(), 1, 2);
			graph.pack();
			RefineryUtilities.centerFrameOnScreen(graph);
			graph.setVisible(true);
		}
	}

	public Graph(String title, String filePath, int...is) throws IOException {
		super(title);
		JFreeChart chart = ChartFactory.createXYLineChart("Compare Auto", "Time", "Dist",
				createDataset(filePath, is), PlotOrientation.VERTICAL, true, true, false);
		
		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new Dimension(1250, 900));
		final XYPlot plot = chart.getXYPlot();
		plot.setBackgroundPaint(Color.WHITE);
		plot.setDomainGridlinePaint(Color.BLACK);
		plot.setRangeGridlinePaint(Color.BLACK);
		
		XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();
		renderer.setSeriesPaint(0, Color.RED); //X
		renderer.setSeriesPaint(1, Color.BLUE); //Y
		renderer.setSeriesPaint(2, Color.PINK); //X2
		renderer.setSeriesPaint(3, Color.CYAN); //Y2
		renderer.setSeriesShapesVisible(0, false);
		renderer.setSeriesShapesVisible(1, false);
		renderer.setSeriesShapesVisible(2, false);
		renderer.setSeriesShapesVisible(3, false);
		
		plot.setRenderer(renderer); 
		setContentPane(chartPanel);
	}
	
	public static String intToAxis(int i) {
		if(i == 0) return "X";
		if(i == 1) return "Y";
		if(i == 2) return "Z";
		return "A";
	}
	
	public static XYDataset createDataset(String filePath, int...is) throws IOException {
		final XYSeries X = new XYSeries("X");
		final XYSeries Y = new XYSeries("Y");
		
		final XYSeries eX = new XYSeries("Enc X");
		final XYSeries eY = new XYSeries("Enc Y");
		
		ArrayList<Double[]> data = parseFile(filePath);
		
		double[] lastEnc = new double[3];
		double[] encDiff = new double[3];
		double xD = 0;
		double yD = 0;
		double x = 0;
		double y = 0;
		double inchesPerTick = Math.PI*4/1120;

		for(int i = 0; i < data.size(); i++) {
			X.add(i, data.get(i)[is[0]]);
			Y.add(i, data.get(i)[is[1]]);
			
			//X.add(data.get(i)[0], data.get(i)[1]);
			//Y.add(data.get(i)[0], new Double(1.09*data.get(i)[0]));
			
			/*if(data.get(i)[is[0]] != Double.POSITIVE_INFINITY && data.get(i)[is[0]] != Double.NEGATIVE_INFINITY && data.get(i)[is[0]] != 52.0) {
				X.add(i, 52-ultraFunc(52-data.get(i)[is[0]]));
			}
			else {
				X.add(i, 52-ultraFunc(52-data.get(i-1)[is[0]]));
			}*/
			
			//Y.add(i, ultraFunc(data.get(i)[is[1]]+28)-28);
			
			/*if(data.get(i)[is[1]] != Double.POSITIVE_INFINITY && data.get(i)[is[1]] != Double.NEGATIVE_INFINITY) {Y.add(i, data.get(i)[is[1]]);}
			else {Y.add(i, (data.get(i-1)[is[1]]+data.get(i+1)[is[1]])/2);}*/
			
			/*for(int j = 0; j < 3; j++) {
				encDiff[j] = data.get(i)[3+j]-lastEnc[j];
				lastEnc[j] = data.get(i)[3+j];
			}
			
			xD = (encDiff[1]-encDiff[2])/Math.sqrt(3)*inchesPerTick;
	        yD = (encDiff[0]-(encDiff[2]+encDiff[1])/2)*2/3*inchesPerTick;
	        
	        double driveDistance = Math.sqrt(Math.pow(xD, 2)+Math.pow(yD, 2));
	        double theta = new Angle(data.get(i)[0]).getRadian()+Math.atan2(yD, xD);
	        
	        x+=driveDistance*Math.cos(theta);
	        y+=driveDistance*Math.sin(theta);
	        
	        eX.add(i, x);
	        eY.add(i, y);*/
			
		}
		System.out.println("X: "+x);
		System.out.println("Y: "+y);
		
		final XYSeriesCollection dataset = new XYSeriesCollection();
		dataset.addSeries(X);
		dataset.addSeries(Y);
		//dataset.addSeries(eX);
		//dataset.addSeries(eY);
		return dataset;
	}
	
	public static double ultraFunc(double data) {
		return 1.0919*data-0.5322;
	}
	
	protected static ArrayList<Double[]> parseFile(String filePath) throws IOException {
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
	private static Double[] stringsToDoubles(String[] strings) {
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
