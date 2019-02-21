import org.opencv.core.Point;

public class Circle {

	public Point center;
	public double radius;

	public Circle() {}

	public Circle(Point center, double radius) {
		this.center = center;
		this.radius = radius;
	}

}