import java.util.ArrayList;
import java.util.List;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TargetRunnable implements Runnable {

	static NetworkTable targetsTable = NetworkTableInstance.getDefault().getTable("vision/targets");

	static TargetPipeline pipeline = new TargetPipeline();
	static RotatedRect leftRect, rightRect;

	Mat mat;

	public TargetRunnable(Mat mat) {
		this.mat = mat;
	}

	@Override
	public void run() {
		pipeline.process(this.mat);
		RotatedRect[] rotatedRects = findTargets(pipeline.filterContoursOutput());
		if(rotatedRects.length == 2) {
			leftRect  = rotatedRects[0];
			rightRect = rotatedRects[1];
			if(rotatedRects[0] != rotatedRects[1] && diff(rotatedRects[0].angle, -75.5) < 10 && diff(rotatedRects[1].angle, -14.5) < 10) {
				putTargets(rotatedRects[0], rotatedRects[1]);
			}
		} else {
			resetTargetEntries();
		}
	}

	public static void drawMinAreaRect(Mat mat, RotatedRect rect, Scalar color) {
		if(rect == null) return;
		Point[] vertices = new Point[4];
		rect.points(vertices);
		for(int j = 0; j < 4; j++) {
			Imgproc.line(mat, vertices[j], vertices[(j + 1) % 4], color, 2);
		}
	}

	public static double distance(double width, double height) {
		return (distanceW(width) + distanceH(height)) / 2;
	}

	public static double distanceW(double width) {
		double focalLength = 393.903;
		double realWidth = 3.313;
		return (focalLength * realWidth) / width;
	}

	public static double distanceH(double height) {
		double focalLength = 370.815;
		double realHeight = 5.825;
		return (focalLength * realHeight) / height;
	}

	public static double diff(double a, double b) {
		return Math.abs(Math.abs(a) - Math.abs(b));
	}

	public static double distanceToCenter(Point a) {
		return Math.sqrt(Math.pow(a.x - 208, 2) + Math.pow(a.y - 120, 2));
	}

	public static void resetTargetEntries() {
		NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
		targetsTable.getEntry("contour_left").setDoubleArray(new double[6]);
		targetsTable.getEntry("contour_right").setDoubleArray(new double[6]);
	}

	public static RotatedRect[] findTargets(List<MatOfPoint> contours) {
		if(contours.size() == 0) return new RotatedRect[0];
		List<RotatedRect> rotatedBoxes = new ArrayList<RotatedRect>();
		for(MatOfPoint mat : contours) {
			MatOfPoint2f mat2f = new MatOfPoint2f();
			mat.convertTo(mat2f, CvType.CV_32F);
			rotatedBoxes.add(Imgproc.minAreaRect(mat2f));
		}

		rotatedBoxes.sort((a, b) -> { return Double.compare(distanceToCenter(a.center), distanceToCenter(b.center)); });

		int iLeft = 0, iRight = 0;
		for(int i = 0; i < rotatedBoxes.size(); i++) {
			if(diff(rotatedBoxes.get(i).angle, -75.5) < diff(rotatedBoxes.get(iLeft).angle, -75.5)) {
				iLeft = i;
			} else if(diff(rotatedBoxes.get(i).angle, -14.5) < diff(rotatedBoxes.get(iRight).angle, -14.5)) {
				iRight = i;
			}
		}

		return new RotatedRect[]{ rotatedBoxes.get(iLeft), rotatedBoxes.get(iRight) };
	}

	public static void putTargets(RotatedRect lTarget, RotatedRect rTarget) {
		Rect leftBox = lTarget.boundingRect();
		targetsTable.getEntry("contour_left").setDoubleArray(new double[]{
			leftBox.x, leftBox.y, leftBox.width, leftBox.height, distance(leftBox.width, leftBox.height), lTarget.angle
		});
		Rect rightBox = rTarget.boundingRect();
		targetsTable.getEntry("contour_right").setDoubleArray(new double[]{
			rightBox.x, rightBox.y, rightBox.width, rightBox.height, distance(rightBox.width, rightBox.height), rTarget.angle
		});
	}

}