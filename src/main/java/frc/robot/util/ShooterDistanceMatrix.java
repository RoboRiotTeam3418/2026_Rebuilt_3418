package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterDistanceMatrix {
    private static InterpolatingDoubleTreeMap matrix = new InterpolatingDoubleTreeMap();

    public static void InitializeMatrix() {
        matrix.put(3.9, 45.0);
        matrix.put(2.4, 180.0);
        matrix.put(3.5, 60.0);
        matrix.put(4.3, 75.0);
        matrix.put(3.9, 90.0);
        matrix.put(3.0, 105.0);
        matrix.put(3.0, 120.0);
        matrix.put(2.7, 135.0);
        matrix.put(2.6, 150.0);
        matrix.put(2.1, 165.0);
    }

    public static double get(double val) {
        return matrix.get(val);
    }
}