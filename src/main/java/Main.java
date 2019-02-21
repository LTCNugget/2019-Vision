import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;

public final class Main {
	private static String configFile = "/boot/frc.json";

	@SuppressWarnings("MemberName")
	public static class CameraConfig {
		public String name;
		public String path;
		public JsonObject config;
		public JsonElement streamConfig;
	}

	public static int team;
	public static boolean server;
	public static List<CameraConfig> cameraConfigs = new ArrayList<>();

	public static int source;
	public static final int hatchCamera = 1, cargoCamera = 0;
	public static Mat lastMat;

	public static boolean running = false;
	public static RotatedRect leftRect, rightRect;

	private Main() {
	}

	/**
	 * Report parse error.
	 */
	public static void parseError(String str) {
		System.err.println("config error in '" + configFile + "': " + str);
	}

	/**
	 * Read single camera configuration.
	 */
	public static boolean readCameraConfig(JsonObject config) {
		CameraConfig cam = new CameraConfig();

		// name
		JsonElement nameElement = config.get("name");
		if(nameElement == null) {
			parseError("could not read camera name");
			return false;
		}
		cam.name = nameElement.getAsString();

		// path
		JsonElement pathElement = config.get("path");
		if(pathElement == null) {
			parseError("camera '" + cam.name + "': could not read path");
			return false;
		}
		cam.path = pathElement.getAsString();

		// stream properties
		cam.streamConfig = config.get("stream");

		cam.config = config;

		cameraConfigs.add(cam);
		return true;
	}

	/**
	 * Read configuration file.
	 */
	@SuppressWarnings("PMD.CyclomaticComplexity")
	public static boolean readConfig() {
		// parse file
		JsonElement top;
		try {
			top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
		} catch(IOException ex) {
			System.err.println("could not open '" + configFile + "': " + ex);
			return false;
		}

		// top level must be an object
		if(!top.isJsonObject()) {
			parseError("must be JSON object");
			return false;
		}
		JsonObject obj = top.getAsJsonObject();

		// team number
		team = 4509;

		// ntmode (optional)
		if(obj.has("ntmode")) {
			String str = obj.get("ntmode").getAsString();
			if("client".equalsIgnoreCase(str)) {
				server = false;
			} else if("server".equalsIgnoreCase(str)) {
				server = true;
			} else {
				parseError("could not understand ntmode value '" + str + "'");
			}
		}

		// cameras
		JsonElement camerasElement = obj.get("cameras");
		if(camerasElement == null) {
			parseError("could not read cameras");
			return false;
		}
		JsonArray cameras = camerasElement.getAsJsonArray();
		for(JsonElement camera : cameras) {
			if(!readCameraConfig(camera.getAsJsonObject())) {
				return false;
			}
		}

		return true;
	}

	/**
	 * Start running the camera.
	 */
	public static VideoSource startCamera(CameraConfig config) {
		System.out.println("Starting camera '" + config.name + "' on " + config.path);
		CameraServer inst = CameraServer.getInstance();
		UsbCamera camera = new UsbCamera(config.name, config.path);
		MjpegServer server = inst.startAutomaticCapture(camera);

		Gson gson = new GsonBuilder().create();

		camera.setConfigJson(gson.toJson(config.config));
		camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

		if(config.streamConfig != null) {
			server.setConfigJson(gson.toJson(config.streamConfig));
		}

		return camera;
	}

	/**
	 * Main.
	 */
	public static void main(String... args) {
		if(args.length > 0) {
			configFile = args[0];
		}

		// read configuration
		if(!readConfig()) {
			return;
		}

		// start NetworkTables
		NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
		if(server) {
			System.out.println("Setting up NetworkTables server");
			ntinst.startServer();
		} else {
			System.out.println("Setting up NetworkTables client for team " + team);
			ntinst.startClientTeam(team);
		}

		// start cameras
		List<VideoSource> cameras = new ArrayList<>();
		for(CameraConfig cameraConfig : cameraConfigs) {
			cameras.add(startCamera(cameraConfig));
		}

		if(cameras.size() >= 2) {
			source = (int)NetworkTableInstance.getDefault().getTable("vision").getEntry("source").getDouble(0) % cameras.size();
			ntinst.getTable("vision").addEntryListener("source", (table, key, entry, value, flags) -> {
				source = (int)value.getDouble() % cameras.size();
			}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

			Thread grabMatThread = new Thread(() -> {
				lastMat = new Mat();
				while(running) {
					CameraServer.getInstance().getVideo(cameras.get(source)).grabFrame(lastMat);
				}
			});

			Thread hatchThread = new Thread(() -> {
				HatchPipeline hatchPipeline = new HatchPipeline();
				while(running) {
					hatchPipeline.process(lastMat);
					RotatedRect[] rotatedRects = findHatchTargets(hatchPipeline.filterContoursOutput());
					if(rotatedRects.length == 2) {
						leftRect  = rotatedRects[0];
						rightRect = rotatedRects[1];
						if(rotatedRects[0] != rotatedRects[1] && diff(rotatedRects[0].angle, -75.5) < 10 && diff(rotatedRects[1].angle, -14.5) < 10) {
							putHatchTargets(rotatedRects[0], rotatedRects[1]);
						}
					} else {
						resetHatchEntries();
					}
				}
			});

			Thread cargoThread = new Thread(() -> {
				CargoPipeline cargoPipeline = new CargoPipeline();
				while(running) {
					cargoPipeline.process(lastMat);
					Circle[] circles = cargoPipeline.filterContoursOutput().parallelStream().map((mat) -> {
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
					ntinst.getTable("vision/cargo").getEntry("x").setDoubleArray(cargoX);
					ntinst.getTable("vision/cargo").getEntry("y").setDoubleArray(cargoY);
					ntinst.getTable("vision/cargo").getEntry("r").setDoubleArray(cargoR);
				}
			});

			running = true;
			grabMatThread.start();
			hatchThread.start();
			cargoThread.start();
		}

		// loop forever
		for(;;) {
			try {
				Thread.sleep(10000);
			} catch (InterruptedException ex) {
				running = false;
				return;
			}
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

	public static void resetHatchEntries() {
		NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
		ntinst.getTable("vision/targets").getEntry("contour_left").setDoubleArray(new double[6]);
		ntinst.getTable("vision/targets").getEntry("contour_right").setDoubleArray(new double[6]);
	}

	public static RotatedRect[] findHatchTargets(List<MatOfPoint> contours) {
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

	public static void putHatchTargets(RotatedRect lTarget, RotatedRect rTarget) {
		Rect leftBox = lTarget.boundingRect();
		NetworkTableInstance.getDefault().getTable("vision/hatch-targets").getEntry("contour_left").setDoubleArray(new double[]{
			leftBox.x, leftBox.y, leftBox.width, leftBox.height, distance(leftBox.width, leftBox.height), lTarget.angle
		});
		Rect rightBox = rTarget.boundingRect();
		NetworkTableInstance.getDefault().getTable("vision/hatch-targets").getEntry("contour_right").setDoubleArray(new double[]{
			rightBox.x, rightBox.y, rightBox.width, rightBox.height, distance(rightBox.width, rightBox.height), rTarget.angle
		});
	}

}