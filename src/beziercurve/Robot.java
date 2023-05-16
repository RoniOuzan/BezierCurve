package beziercurve;

import math.geometry.*;

public class Robot {
    private Pose2d position;

    private final Constants constants;

    public Robot(Pose2d position, Constants constants) {
        this.position = position;
        this.constants = constants;
    }

    public void drive(Pose2d velocity) {
        if (velocity.getTranslation().getNorm() > constants.maxVel) {
            velocity = new Pose2d(new Translation2d(constants.maxVel, velocity.getTranslation().getAngle()), velocity.getRotation());
        }

        this.position = new Pose2d(
                        this.position.getTranslation().plus(velocity.getTranslation().times(constants.period)),
                        this.position.getRotation().rotateBy(Rotation2d.fromDegrees(velocity.getRotation().getDegrees() * constants.period)));
    }

    public Pose2d getPosition() {
        return position;
    }

    public static class Constants {
        public final double maxVel;
        public final double period;

        public Constants(double maxVel, double period) {
            this.maxVel = maxVel;
            this.period = period;
        }
    }
}
