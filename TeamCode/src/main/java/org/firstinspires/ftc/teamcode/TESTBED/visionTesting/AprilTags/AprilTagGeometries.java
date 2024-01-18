package org.firstinspires.ftc.teamcode.TESTBED.visionTesting.AprilTags;

public class AprilTagGeometries {
    public static double[] XYTTtoRBY(double x, double y, double botTheta, double tagTheta){
        double range = Math.sqrt(x * x + y * y);
        double bearing = Math.toDegrees(Math.atan2(-x,y));
        double yaw = botTheta - tagTheta;
        return new double[]{range, bearing, yaw};
    }

    public static double[] RBYtoXYT(double range, double bearing, double yaw, double tagTheta){
        double x = -range*Math.sin(Math.toRadians(bearing));
        double y = range*Math.cos(Math.toRadians(bearing));
        double t = yaw + tagTheta;
        return new double[]{x, y, t};
    }
}
