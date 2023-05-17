package beziercurve;

import beziercurve.pid.ProfiledPIDController;
import beziercurve.pid.TrapezoidProfile;
import gui.Frame;
import gui.types.draw.DrawCentered;
import gui.types.field.ZeroCenter;
import math.MathUtil;
import math.geometry.Dimension2d;
import math.geometry.Pose2d;
import math.geometry.Rotation2d;
import math.geometry.Translation2d;

import javax.swing.*;
import java.awt.*;

public class BezierCurveGUI extends Frame implements ZeroCenter, DrawCentered {
    private static final double MAX_VALUE = 8;
    private static final Dimension2d DIMENSION = new Dimension2d(1920, 930);
    private static final int PIXELS_IN_UNIT = (int) (DIMENSION.getX() / MAX_VALUE) / 2;

    private static final double FPS = 20;
    private static final double ROBOT_WIDTH = 0.91;
    private static final double TOLERANCE = 0.2;

    private final BezierCurve bezierCurve;
    private final Robot robot;

    private double currentT = 0;

    private final ProfiledPIDController pidController;

    public BezierCurveGUI() {
        super("Bezier Curve", DIMENSION, PIXELS_IN_UNIT);
//        this.setExtendedState(JFrame.MAXIMIZED_BOTH);

        this.bezierCurve = new BezierCurve(new BezierCurve.Constants(4.5, 4.5, 0.5),
                new Translation2d(-9, -3),
                new Translation2d(-5, 6),
                new Translation2d(-2, 1),
                new Translation2d(-6, -6),
                new Translation2d(3, 3),
                new Translation2d(7, -1)
        );
        this.robot = new Robot(new Pose2d(this.bezierCurve.getStartPoint(), Rotation2d.fromDegrees(0)),
                new Robot.Constants(5, 1 / FPS));

        this.pidController = new ProfiledPIDController(2, 0, 0,
                new TrapezoidProfile.Constraints(0.4, 0.4));
        this.pidController.reset(
                this.bezierCurve.getDistance(0, this.bezierCurve.getClosestPoint(this.robot.getPosition()).t()),
                0);

        this.draw();
        this.start();
    }

    public void draw() {
        this.drawGrid();

        for (double t = this.bezierCurve.getDifferentBetweenTs(); t < 1; t += this.bezierCurve.getDifferentBetweenTs()) {
            this.drawRobotPose(this.bezierCurve.getLocation(t));
        }

        for (Translation2d waypoint : this.bezierCurve.getWaypoints()) {
            this.drawWaypoint(waypoint);
        }
    }

    public void moveRobot() {
        BezierCurve.State state = this.bezierCurve.getClosestPoint(this.robot.getPosition());
        currentT = state.t();
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

        if (this.robot.getPosition().getTranslation().getDistance(this.bezierCurve.getFinalPoint()) > TOLERANCE) {
            this.robot.drive(this.bezierCurve.getVelocity(state, this.robot.getPosition(), this.calculateVelocity()));
            Translation2d setpoint = this.robot.getPosition().getTranslation()
                    .plus(new Translation2d(this.pidController.getSetpoint().position, this.robot.getVelocity().getTranslation().getAngle()));
            this.fillPoint(setpoint.getX(), setpoint.getY(), convertPixelsToUnits(5), Color.GREEN);

            double curvatureRadius = this.bezierCurve.getCurvatureRadius(state.t());
            Translation2d curvature = state.pose().getTranslation()
                    .plus(new Translation2d(
                            curvatureRadius,
                            this.bezierCurve.getAngle(state.t()).plus(Rotation2d.fromDegrees(curvatureRadius > 0 ? -90 : 90))
                    ));
            this.drawPoint(curvature.getX(), curvature.getY(), Math.abs(curvatureRadius), Color.BLUE);
        }
    }

    public double calculateVelocity() {
        return this.pidController.calculate(
                this.bezierCurve.getDistance(0, currentT), this.bezierCurve.getDistance(0, 1));
    }

    public void writeValues() {
        String[] texts = {
                "T: " + MathUtil.limitDot(this.currentT, 4),
                "Pose: (" + MathUtil.limitDot(this.robot.getPosition().getTranslation().getX(), 3) + ", "
                        + MathUtil.limitDot(this.robot.getPosition().getTranslation().getY(), 3) + ")",
                "Angle: " + MathUtil.limitDot(this.robot.getPosition().getRotation().getDegrees(), 3) + " Deg",
                "Vector: (" + MathUtil.limitDot(this.robot.getVelocity().getTranslation().getX(), 3) + ", "
                        + MathUtil.limitDot(this.robot.getVelocity().getTranslation().getY(), 3) + ")",
                "Velocity: " + MathUtil.limitDot(this.robot.getVelocity().getTranslation().getNorm(), 3) + "m/s",
                "Angular Velocity: " + MathUtil.limitDot(this.robot.getVelocity().getRotation().getDegrees(), 3) + " Deg",
                "Distance: " + MathUtil.limitDot(this.bezierCurve.getDistance(0, currentT), 3),
                "Curvature Radius: " + MathUtil.limitDot(this.bezierCurve.getCurvatureRadius(currentT), 3)

        };

        double size = convertPixelsToUnits(20);
        double space = convertPixelsToUnits(10);
        for (int i = 0; i < texts.length; i++) {
            this.write(-MAX_VALUE, MAX_VALUE - ((size + space) * (i + 0.5)), texts[i], size, Color.BLACK);
        }
    }

    public void start() {
        double t = 0;
        int direction = 1;

        while (true) {
            this.clearFrame();
            this.draw();
            this.moveRobot();
            this.writeValues();
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
        for (double i = (int) -Math.floor(MAX_VALUE); i <= MAX_VALUE; i += 0.25) {
            this.drawThinLine(i, this.getDimensionWithUnits().getY() / -2, i, this.getDimensionWithUnits().getY() / 2,
                    Math.floor(i * 10) / 10d % 1 == 0 ? new Color(60, 60, 60) : new Color(230, 230, 230));
        }

        for (double i = (int) -Math.floor(MAX_VALUE); i <= MAX_VALUE; i += 0.25) {
            this.drawThinLine(this.getDimensionWithUnits().getX() / -2, i, this.getDimensionWithUnits().getX() / 2, i,
                    Math.floor(i * 10) / 10d % 1 == 0 ? new Color(60, 60, 60) : new Color(230, 230, 230));
        }

        this.drawLine(0, this.getDimensionWithUnits().getY() / -2, 0, this.getDimensionWithUnits().getY() / 2, convertPixelsToUnits(5), Color.BLACK);
        this.drawLine(this.getDimensionWithUnits().getX() / -2, 0, this.getDimensionWithUnits().getX() / 2, 0, convertPixelsToUnits(5), Color.BLACK);

        double textSize = convertPixelsToUnits(20);
        for (int i = 0; i <= MAX_VALUE; i += MAX_VALUE >= 15 ? 5 : 1) {
            this.write(0, i - (textSize / 2), " " + i, textSize, Color.BLACK);
            if (i != 0)
                this.write(0, -i - (textSize / 2), -i + "", textSize, Color.BLACK);
        }
        for (int i = 0; i <= MAX_VALUE; i += MAX_VALUE >= 15 ? 5 : 1) {
            this.write(i - (textSize / 2), -textSize, " " + i, textSize, Color.BLACK);
            if (i != 0)
                this.write(-i - (textSize / 2), -textSize, -i + "", textSize, Color.BLACK);
        }
    }
}
