package beziercurve;

import math.geometry.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class BezierCurve {
    private final List<Translation2d> waypoints;
    private final double differentBetweenTs;

    private boolean hasBeenInLast;

    public BezierCurve(List<Translation2d> waypoints) {
        this.waypoints = waypoints;
        this.differentBetweenTs = 0.02 / waypoints.size();

        this.hasBeenInLast = false;
    }

    public BezierCurve(Translation2d... waypoints) {
        this(List.of(waypoints));
    }

    public State getClosestPoint(Pose2d robotPose) {
        double minT = 0;
        double minDistance = Double.MAX_VALUE;
        Pose2d output = this.getPosition(0);
        for (double t = this.differentBetweenTs; t <= 1; t += this.differentBetweenTs) {
            Pose2d pose = this.getPosition(t);
            double distance = robotPose.getTranslation().getDistance(pose.getTranslation());
//            if (Math.abs(distance) <= 0.001)
//                return pose;

            if (distance < minDistance) {
                minDistance = distance;
                output = pose;
                minT = t;
            }
        }

        if (Math.abs(minT - 1) <= this.differentBetweenTs) {
            hasBeenInLast = true;
        }

        return new State(output, minT);
    }

    public Pose2d getVelocity(State state, Pose2d robot, double velocity) {
        Translation2d vector = new Translation2d(velocity, 0).rotateBy(this.getAngle(state.t))
                .plus(state.pose.getTranslation().minus(robot.getTranslation()).times(3));
        return new Pose2d(vector, Rotation2d.fromDegrees(0));
    }

    public Pose2d getVelocity(State state, Pose2d robot) {
        return this.getVelocity(state, robot, (robot.getTranslation().getDistance(getFinalPoint())) * 2);
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

    private double getSlopeX(double t) {
        return (this.getX(t + 0.0001) - this.getX(t - 0.0001)) / 0.0002;
    }

    private double getSlopeY(double t) {
        return (this.getY(t + 0.0001) - this.getY(t - 0.0001)) / 0.0002;
    }

    public Rotation2d getAngle(double t) {
        Translation2d angle = this.getLocation(t + 0.001).minus(this.getLocation(t - 0.001));
        return new Rotation2d(angle.getX(), angle.getY());
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

    public double getDifferentBetweenTs() {
        return differentBetweenTs;
    }

    public static class State {
        public final Pose2d pose;
        public final double t;

        public State(Pose2d pose, double t) {
            this.pose = pose;
            this.t = t;
        }
    }
}
