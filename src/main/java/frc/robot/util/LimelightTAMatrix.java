package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class LimelightTAMatrix {
    private static InterpolatingDoubleTreeMap cameraMap = new InterpolatingDoubleTreeMap();

    /**
     * Initalizes the matrices for use. Call this on robot start.
     */
    public static void InitializeMatrix() {
        cameraMap.put(72.0, 19.25); // TA - Centimeters
        cameraMap.put(29.5, 25.0); 
        cameraMap.put(8.95, 50.0);
        cameraMap.put(4.5, 75.0);
        cameraMap.put(2.55, 100.0);
        cameraMap.put(1.7, 125.0);
        cameraMap.put(1.18, 150.0);
    }

    /**
     * Accuratly estimates (Within 5cm of actual distance) distance from april-tag based on ta (From camera)
     * @param ta The limelight ta value
     * @return The distance in centimeters
     */
    public static double get(double ta) {
        return cameraMap.get(ta);
    }


    /*
    Usage:

    if (LimelightHelpers.getTV("limelight")) {
        ta = LimelightTAMatrix.get(LimelightHelpers.getTA("limelight")); 
    }
    
    */
}
