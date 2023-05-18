package beziercurve;

import math.geometry.Pose2d;
import math.geometry.Rotation2d;
import math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

@SuppressWarnings(value = "unused")
public class BezierCurve {
    private static final double DX = 0.0001;

    private final List<Translation2d> waypoints;
    private final double differentBetweenTs;

    private final Constants constants;

    public BezierCurve(Constants constants, List<Translation2d> waypoints) {
        this.constants = constants;
        this.waypoints = waypoints;
        this.differentBetweenTs = 0.01 / waypoints.size();
    }

    public BezierCurve(Constants constants, Translation2d... waypoints) {
        this(constants, new ArrayList<>(Arrays.asList(waypoints)));
    }

    public State getClosestPoint(Pose2d robotPose) {
        double minT = 0;
        double minDistance = Double.MAX_VALUE;
        Pose2d output = this.getPosition(0);
        for (double t = this.differentBetweenTs; t <= 1; t += this.differentBetweenTs) {
            Pose2d pose = this.getPosition(t);
            double distance = robotPose.getTranslation().getDistance(pose.getTranslation());

            if (distance < minDistance) {
                minDistance = distance;
                output = pose;
                minT = t;
            }
        }

        return new State(output, minT);
    }

    public Pose2d getVelocity(State state, Pose2d robot, double velocity) {
        Translation2d vector = new Translation2d(1 - constants.errorCorrectorPower, this.getAngle(state.t))
                .plus(state.pose.getTranslation().minus(robot.getTranslation()).times(constants.errorCorrectorPower));

        double curvature = 1 / Math.abs(this.getCurvatureRadius(state.t));
        velocity = Math.min(constants.maxVel - Math.min(curvature, 3.5), velocity);

        return new Pose2d(new Translation2d(velocity, vector.getAngle()), this.getAngle(state.t));
    }

    public Pose2d getVelocity(State state, Pose2d robot) {
        return this.getVelocity(state, robot, (robot.getTranslation().getDistance(getFinalPoint())) * 3);
    }

    public double getX(double t) {
        List<Double> pointsX = waypoints.stream().map(Translation2d::getX).toList();
        while (pointsX.size() > 1) {
            List<Double> newX = new ArrayList<>();
            for (int i = 0; i < pointsX.size() - 1; i++) {
                newX.add(pointsX.get(i) + (t * (pointsX.get(i + 1) - pointsX.get(i))));
            }
            pointsX = newX;
        }
        return pointsX.get(0);
    }

    public Translation2d[] getBezierPoint(double t, int times) {
        List<Double> pointsX = waypoints.stream().map(Translation2d::getX).toList();
        List<Double> pointsY = waypoints.stream().map(Translation2d::getY).toList();
        for (int j = 0; j < times; j++) {
            List<Double> newX = new ArrayList<>();
            List<Double> newY = new ArrayList<>();
            for (int i = 0; i < pointsX.size() - 1; i++) {
                newX.add(pointsX.get(i) + (t * (pointsX.get(i + 1) - pointsX.get(i))));
                newY.add(pointsY.get(i) + (t * (pointsY.get(i + 1) - pointsY.get(i))));
            }
            pointsX = newX;
            pointsY = newY;
        }
        Translation2d[] points = new Translation2d[pointsX.size()];
        for (int i = 0; i < points.length; i++) {
            points[i] = new Translation2d(pointsX.get(i), pointsY.get(i));
        }
        return points;
    }

    public double getY(double t) {
        List<Double> pointsY = waypoints.stream().map(Translation2d::getY).toList();
        while (pointsY.size() > 1) {
            List<Double> newY = new ArrayList<>();
            for (int i = 0; i < pointsY.size() - 1; i++) {
                newY.add(pointsY.get(i) + (t * (pointsY.get(i + 1) - pointsY.get(i))));
            }
            pointsY = newY;
        }
        return pointsY.get(0);
    }

    public Translation2d getLocation(double t) {
        return new Translation2d(getX(t), getY(t));
    }

    public Pose2d getPosition(double t) {
        return new Pose2d(this.getLocation(t), this.getAngle(t));
    }

    public double getXDerivative(double t) {
        return this.calculateDerivative(t, this::getX);
    }

    public double getYDerivative(double t) {
        return this.calculateDerivative(t, this::getY);
    }

    public double getXSecondDerivative(double t) {
        return this.calculateDerivative(t, this::getXDerivative);
    }

    public double getYSecondDerivative(double t) {
        return this.calculateDerivative(t, this::getYDerivative);
    }

    private double calculateDerivative(double x, Function<Double, Double> function) {
        return (function.apply(x + DX) - function.apply(x - DX)) / (2 * DX);
    }

    public Rotation2d getAngle(double t) {
        Translation2d angle = this.getLocation(t + DX).minus(this.getLocation(t - DX));
        return new Rotation2d(angle.getX(), angle.getY());
    }

    public double getDistance(double t1, double t2) {
        double sum = 0;
        for (double t = t1; t < t2; t += DX) {
            sum += (Math.hypot(this.getXDerivative(t - DX), this.getYDerivative(t - DX)) + Math.hypot(this.getXDerivative(t + DX), this.getYDerivative(t + DX))) * (0.5 * DX);
        }
        return sum;
    }

    public double getDistance(double t) {
        return getDistance(0, t);
    }

    public double getCurvatureRadius(double t) {
        double d1x = this.getXDerivative(t);
        double d1y = this.getYDerivative(t);
        double d2x = this.getXSecondDerivative(t);
        double d2y = this.getYSecondDerivative(t);

        return Math.pow((d1x * d1x) + (d1y * d1y), 1.5) / ((d1x * d2y) - (d1y * d2x));
    }

    public Translation2d getFinalPoint() {
        return this.waypoints.get(this.waypoints.size() - 1);
    }

    public Translation2d getStartPoint() {
        return this.waypoints.get(0);
    }

    public List<Translation2d> getWaypoints() {
        return waypoints;
    }

    public void setWaypoint(int index, Translation2d waypoint) {
        this.waypoints.set(index, waypoint);
    }

    public double getDifferentBetweenTs() {
        return differentBetweenTs;
    }

    public double getPathLength() {
        return this.getDistance(0, 1);
    }

    public record State(Pose2d pose, double t) {}

    public record Constants(double maxVel, double maxAccel, double errorCorrectorPower) {}
}
