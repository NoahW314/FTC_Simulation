package org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FilterTestsAccel;

import java.io.File;
import java.io.IOException;

import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FilePath;
import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.FiltersTest;
import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.CompareAutoGraphs.DataPath;
import org.firstinspires.ftc.teamcode.teamcalamari.FilterStuff.Filters.NoFilter;
import org.jfree.ui.RefineryUtilities;

public class NoFilterTest extends FiltersTest<NoFilter> {
	
	public static void main(String[] args) throws IOException {
		File folder = new File(FilePath.folder);
		for(File file : folder.listFiles()) {
			if(file.isDirectory()) break;
			for(int i = 0; i < 2; i++) {
				NoFilterTest chart = new NoFilterTest(file.getName(),"No Filter", new NoFilter(), FilePath.folder+"\\"+file.getName(), i);
				
				chart.pack();
				RefineryUtilities.centerFrameOnScreen(chart);
				chart.setVisible(true);
			}
			/*NoFilterTest chart = new NoFilterTest("Filter Test", "No Filter", new NoFilter(), FilePath.folder+"\\"+file.getName(), 3);
			
			chart.pack();
			RefineryUtilities.centerFrameOnScreen(chart);
			chart.setVisible(true);*/
		}
   }
	
	private static final long serialVersionUID = 2780949740655076103L;

	public NoFilterTest(String title, String chartTitle, NoFilter filt, String filePath, int i) throws IOException {
		super(title, chartTitle, filt, filePath, i);
	}

}
