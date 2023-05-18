package beziercurve;

import math.geometry.*;

public class Robot {
    private Pose2d position;
    private Pose2d velocity;

    private final Constants constants;

    public Robot(Pose2d position, Constants constants) {
        this.position = position;
        this.velocity = new Pose2d();
        this.constants = constants;
    }

    public void drive(Pose2d velocity) {
        if (velocity.getTranslation().getNorm() > constants.maxVel) {
            velocity = new Pose2d(new Translation2d(constants.maxVel, velocity.getTranslation().getAngle()), velocity.getRotation());
        }

        this.position = new Pose2d(
                        this.position.getTranslation().plus(velocity.getTranslation().times(constants.period)),
                        velocity.getRotation()); // this.position.getRotation().rotateBy(Rotation2d.fromDegrees(velocity.getRotation().getDegrees() * constants.period))
        this.velocity = velocity;
    }

    public Pose2d getPosition() {
        return position;
    }

    public void setPosition(Pose2d position) {
        this.position = position;
    }

    public Pose2d getVelocity() {
        return velocity;
    }

    public record Constants(double maxVel, double period) {}
}
