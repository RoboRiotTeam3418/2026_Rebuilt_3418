package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterDistanceMatrix {
    private static InterpolatingDoubleTreeMap matrix = new InterpolatingDoubleTreeMap();

    public static void InitializeMatrix() {
        matrix.put(350.35, 0.6); // Untested
        matrix.put(365.82, 0.7); // ~6ft at 0.7 speed (as of shooter prototype #1)
    }

    public static double get(double val) {
        return matrix.get(val);
    }
}