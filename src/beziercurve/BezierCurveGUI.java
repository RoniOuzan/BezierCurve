package beziercurve;

import beziercurve.pid.ProfiledPIDController;
import beziercurve.pid.TrapezoidProfile;
import gui.Frame;
import gui.types.draw.DrawCentered;
import gui.types.field.ZeroCenter;
import math.geometry.Dimension2d;
import math.geometry.Pose2d;
import math.geometry.Rotation2d;
import math.geometry.Translation2d;

import java.awt.*;

public class BezierCurveGUI extends Frame implements ZeroCenter, DrawCentered {
    private static final double MAX_VALUE = 10.5;
    private static final int PIXELS_IN_UNIT = (int) (480 / MAX_VALUE);

    private static final double FPS = 20;
    private static final double ROBOT_WIDTH = 0.91;
    private static final double TOLERANCE = 0.3;

    private final BezierCurve bezierCurve;
    private final Robot robot;

    private final ProfiledPIDController pidController;

    public BezierCurveGUI() {
        super("Bezier Curve", new Dimension2d(960, 960), PIXELS_IN_UNIT);

        this.bezierCurve = new BezierCurve(
                new Translation2d(-9, -3),
                new Translation2d(-5, 6),
                new Translation2d(-2, 1),
                new Translation2d(-6, -6),
                new Translation2d(3, 3),
                new Translation2d(7, -1)
        );
        this.robot = new Robot(new Pose2d(new Translation2d(-3, -4), Rotation2d.fromDegrees(0)),
                new Robot.Constants(4, 1 / FPS));

        this.pidController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(10, 10));

        this.draw();
        this.start();
    }

    public void draw() {
        this.drawGrid();
        this.repaint();

        for (double t = this.bezierCurve.getDifferentBetweenTs(); t < 1; t += this.bezierCurve.getDifferentBetweenTs()) {
            this.drawRobotPose(this.bezierCurve.getLocation(t));
        }

        for (Translation2d waypoint : this.bezierCurve.getWaypoints()) {
            this.drawWaypoint(waypoint);
        }

        this.repaint();
    }

    public void moveRobot() {
        BezierCurve.State state = this.bezierCurve.getClosestPoint(this.robot.getPosition());
        Pose2d robot = this.robot.getPosition();
        double radius = ROBOT_WIDTH / 2;

        Translation2d[] translation2ds = new Translation2d[5];
        for (int i = 0; i < translation2ds.length - 1; i++) {
            double radians = Math.toRadians(45 + (90 * i)) + robot.getRotation().getRadians();
            translation2ds[i] = robot.getTranslation().plus(new Translation2d(radius * Math.cos(radians), radius * Math.sin(radians)));
        }
        translation2ds[4] = robot.getTranslation().plus(
                new Translation2d(
                        1.5 * radius * Math.cos(robot.getRotation().getRadians()),
                        1.5 * radius * Math.sin(robot.getRotation().getRadians())));

        this.fillPolygon(Color.YELLOW, translation2ds);

        if (this.robot.getPosition().getTranslation().getDistance(this.bezierCurve.getFinalPoint()) > TOLERANCE)
            this.robot.drive(this.bezierCurve.getVelocity(state, this.robot.getPosition()));
    }

    public double calculateVelocity(double t) {
        double distance = this.robot.getPosition().getTranslation().getDistance(this.bezierCurve.getFinalPoint());
        double velocity = this.pidController.calculate(distance, 0);
        System.out.println(velocity);
        return velocity;
    }

    public void start() {
        double t = 0;
        int direction = 1;

        while (true) {
            this.clearFrame();
            this.draw();
            this.moveRobot();
            this.repaint();

            t += this.bezierCurve.getDifferentBetweenTs() * direction;
            if (t > 1) {
                t = 1;
                direction = -1;
            } else if (t < 0) {
                t = 0;
                direction = 1;
            }
            try {
                Thread.sleep((long) (1000 / FPS));
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private void drawRobotPose(Translation2d pose) {
        this.fillPoint(pose.getX(), pose.getY(), convertPixelsToUnits(2), Color.RED);
    }

    private void drawWaypoint(Translation2d waypoint) {
        this.fillPoint(waypoint.getX(), waypoint.getY(), convertPixelsToUnits(6), Color.BLUE);
    }

    private void drawGrid() {
        for (int i = (int) -Math.ceil(MAX_VALUE); i <= Math.floor(MAX_VALUE); i++) {
            this.drawThinLine(i, this.getDimensionWithUnits().getY() / -2, i, this.getDimensionWithUnits().getY() / 2, Color.BLACK);
        }

        for (int i = (int) -Math.ceil(MAX_VALUE); i <= Math.floor(MAX_VALUE); i++) {
            this.drawThinLine(this.getDimensionWithUnits().getX() / -2, i, this.getDimensionWithUnits().getX() / 2, i, Color.BLACK);
        }

        this.drawLine(0, this.getDimensionWithUnits().getY() / -2, 0, this.getDimensionWithUnits().getY() / 2, convertPixelsToUnits(5), Color.BLACK);
        this.drawLine(this.getDimensionWithUnits().getX() / -2, 0, this.getDimensionWithUnits().getX() / 2, 0, convertPixelsToUnits(5), Color.BLACK);

        double textSize = convertPixelsToUnits(20);
        for (int i = 0; i <= MAX_VALUE; i++) {
            this.write(0, i - (textSize / 2), " " + i, textSize, Color.BLACK);
            if (i != 0)
                this.write(0, -i - (textSize / 2), -i + "", textSize, Color.BLACK);
        }
        for (int i = 0; i <= MAX_VALUE; i++) {
            this.write(i - (textSize / 2), -textSize, " " + i, textSize, Color.BLACK);
            if (i != 0)
                this.write(-i - (textSize / 2), -textSize, -i + "", textSize, Color.BLACK);
        }
    }
}
