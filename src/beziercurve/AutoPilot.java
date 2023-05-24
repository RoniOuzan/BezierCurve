package beziercurve;

import math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class AutoPilot {
    public static List<Translation2d> hub1 = Arrays.asList(
            new Translation2d(8.727, 1.780),
            new Translation2d(8.505, -0.029),
            new Translation2d(6.053, -4.454),
            new Translation2d(7.921, -4.279)
    );
    public static List<Translation2d> hub9 = Arrays.asList(
            new Translation2d(7.767, 1.199),
            new Translation2d(9.605, -1.234),
            new Translation2d(6.053, -4.454),
            new Translation2d(7.921, -4.279)
    );

    public static List<Translation2d> generateWaypoints(Robot robot, int hub) {
        double percent = (hub - 1) / 8d;
        List<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(robot.getPosition().getTranslation());

        for (int i = 0; i < hub1.size(); i++) {
            waypoints.add(new Translation2d((hub1.get(i).getX() - hub9.get(i).getX()) * percent, (hub1.get(i).getY() - hub9.get(i).getY()) * percent));
        }

        return waypoints;
    }
}
