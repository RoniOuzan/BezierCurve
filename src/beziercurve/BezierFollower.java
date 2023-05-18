package beziercurve;

import beziercurve.pid.ProfiledPIDController;
import beziercurve.pid.TrapezoidProfile;
import math.geometry.Pose2d;
import math.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class BezierFollower {
    private final BezierCurve bezierCurve;
    private final Robot robot;

    private final ProfiledPIDController pidController;

    private BezierCurve.State state = new BezierCurve.State(new Pose2d(), -1);

    private boolean isRunning = true;

    public BezierFollower(BezierCurve bezierCurve, Robot robot, Constants constants) {
        this.bezierCurve = bezierCurve;
        this.robot = robot;

        this.pidController = new ProfiledPIDController(constants.kP, constants.kI, constants.kD, constants.pidConstants);
    }

    public void start() {
        this.pidController.reset(
                this.bezierCurve.getDistance(0, this.bezierCurve.getClosestPoint(this.robot.getPosition()).t()),
                0);
    }

    public void update() {
        this.state = this.getClosestState();

        if (this.isRunning) {
            this.robot.drive(this.bezierCurve.getVelocity(state, this.robot.getPosition(), this.calculateVelocity()));
        }
    }

    public double calculateVelocity() {
        return this.pidController.calculate(this.bezierCurve.getDistance(this.state.t()), this.bezierCurve.getPathLength());
    }

    public void setRunning(boolean running) {
        this.isRunning = running;
    }

    public boolean isRunning() {
        return isRunning;
    }

    private BezierCurve.State getClosestState() {
        List<BezierCurve.State> points = new ArrayList<>();
        for (double t = 0; t <= 1; t += this.bezierCurve.getDifferentBetweenTs()) {
            if (this.state.t() < 0 || Math.abs(this.state.t() - t) <= 0.3)
                points.add(new BezierCurve.State(this.bezierCurve.getPosition(t), t));
        }
        points.sort(Comparator.comparing(s -> this.robot.getPosition().getTranslation().getDistance(s.pose().getTranslation())));
        return points.get(0);
    }

    public ProfiledPIDController getPidController() {
        return pidController;
    }

    public BezierCurve.State getState() {
        return state;
    }

    public void reset() {
        this.state = new BezierCurve.State(new Pose2d(), 0);
        this.robot.setPosition(new Pose2d(this.bezierCurve.getStartPoint(), Rotation2d.fromDegrees(0)));
        this.start();
    }

    public record Constants(double kP, double kI, double kD, TrapezoidProfile.Constraints pidConstants) {}
}
