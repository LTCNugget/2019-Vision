import java.util.stream.Collectors;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.networktables.NetworkTableInstance;

public class CargoRunnable implements Runnable {

	static CargoPipeline pipeline = new CargoPipeline();

	Mat mat;

	public CargoRunnable(Mat mat) {
		this.mat = mat;
	}

	@Override
	public void run() {
		pipeline.process(this.mat);
		Circle[] circles = pipeline.filterContoursOutput().parallelStream().map((mat) -> {
			Circle circle = new Circle();
			float[] radius = new float[]{};
			MatOfPoint2f mat2f = new MatOfPoint2f();
			mat.convertTo(mat2f, CvType.CV_32F);
			Imgproc.minEnclosingCircle(mat2f, circle.center, radius);
			circle.radius = radius[0];
			return circle;
		}).collect(Collectors.toList()).toArray(new Circle[]{});

		double[] cargoX = new double[circles.length];
		double[] cargoY = new double[circles.length];
		double[] cargoR = new double[circles.length];

		for(int i = 0; i < circles.length; i++) {
			cargoX[i] = circles[i].center.x;
			cargoY[i] = circles[i].center.y;
			cargoR[i] = circles[i].radius;
		}
		NetworkTableInstance.getDefault().getTable("vision/cargo").getEntry("x").setDoubleArray(cargoX);
		NetworkTableInstance.getDefault().getTable("vision/cargo").getEntry("y").setDoubleArray(cargoY);
		NetworkTableInstance.getDefault().getTable("vision/cargo").getEntry("r").setDoubleArray(cargoR);
	}

}