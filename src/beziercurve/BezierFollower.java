package beziercurve;

import beziercurve.pid.PIDPreset;
import beziercurve.pid.ProfiledPIDController;
import beziercurve.pid.TrapezoidProfile;
import math.geometry.Pose2d;
import math.geometry.Rotation2d;
import math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class BezierFollower {
    private final BezierCurve bezierCurve;
    private final Robot robot;

    private final Constants constants;

    private final ProfiledPIDController pidController;
    private final ProfiledPIDController omegaController;

    private BezierCurve.State state = new BezierCurve.State(new Pose2d(), -1);

    private boolean isRunning = true;

    private long lastUpdate = 0;

    public BezierFollower(BezierCurve bezierCurve, Robot robot, Constants constants) {
        this.bezierCurve = bezierCurve;
        this.robot = robot;

        this.constants = constants;

        this.pidController = new ProfiledPIDController(constants.drivePreset.getkP(), constants.drivePreset.getkI(), constants.drivePreset.getkD(),
                new TrapezoidProfile.Constraints(constants.drivePreset.getMaxVel(), constants.drivePreset.getMaxAccel()));
        this.omegaController = new ProfiledPIDController(constants.omegaPreset.getkP(), constants.omegaPreset.getkI(), constants.omegaPreset.getkD(),
                new TrapezoidProfile.Constraints(constants.omegaPreset.getMaxVel(), constants.omegaPreset.getMaxAccel()));
        this.omegaController.enableContinuousInput(0, 360);
    }

    public void start() {
        this.pidController.reset(
                this.bezierCurve.getDistance(0, this.bezierCurve.getClosestPoint(this.robot.getPosition()).t()),
                0);
        this.omegaController.reset(this.robot.getPosition().getRotation().getDegrees(), 0);
    }

    public void update() {
        this.state = this.getClosestState();

        if (this.isRunning) {
            Pose2d velocity = this.bezierCurve.getVelocity(state, this.robot.getPosition(), this.calculateVelocity(), this.calculateOmega());

            double lastUpdate = (System.currentTimeMillis() - this.lastUpdate) / 1000d;
            double velocityDiff = velocity.getTranslation().getNorm() - this.robot.getVelocity().getTranslation().getNorm();
            if (Math.abs(velocityDiff) / lastUpdate >= this.bezierCurve.getConstants().maxAccel()) {
                velocity = new Pose2d(
                        new Translation2d(
                            this.robot.getVelocity().getTranslation().getNorm() +
                                    Math.copySign(this.bezierCurve.getConstants().maxAccel() * lastUpdate, velocityDiff),
                            velocity.getTranslation().getAngle()),
                        velocity.getRotation());
            }

            this.robot.drive(velocity);
        }

        this.lastUpdate = System.currentTimeMillis();
    }

    public double calculateVelocity() {
        return this.pidController.calculate(this.bezierCurve.getDistance(this.state.t()), this.bezierCurve.getPathLength());
    }

    public double calculateOmega() {
        return this.omegaController.calculate(this.robot.getPosition().getRotation().getDegrees(), this.constants.endHeading);
    }

    public void setRunning(boolean running) {
        this.isRunning = running;
    }

    public boolean isRunning() {
        return isRunning;
    }

    private BezierCurve.State getClosestState() {
        List<BezierCurve.State> points = new ArrayList<>();
        for (double t = 0; t <= 1; t = getNextT(t, this.bezierCurve.getDifferentBetweenTs())) {
            if (this.state.t() < 0 || Math.abs(this.state.t() - t) <= 0.3)
                points.add(new BezierCurve.State(this.bezierCurve.getPosition(t), t));
        }
        points.sort(Comparator.comparing(s -> this.robot.getPosition().getTranslation().getDistance(s.pose().getTranslation())));
        return points.get(0);
    }

    private double getNextT(double t, double diff) {
        double value = t + diff;
        if (t < 1 && value > 1)
            return 1;
        return value;
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
        this.robot.drive(new Pose2d());
        this.robot.setAngle(this.constants.startHeading);
        this.start();
    }

    public record Constants(double startHeading, double endHeading,
                            PIDPreset drivePreset, PIDPreset omegaPreset) {}
}
