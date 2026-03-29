package frc.robot.subsystems;

public class ShooterCalcV2 {
    private final double INCH_TO_METER = 0.0254;
    private final double[][] shooterData = {
        {10, 0},
        {45, (12 * 1) * INCH_TO_METER},//These are caution  NOT ACCURATE just a guess
        {45, (12 * 2) * INCH_TO_METER},//These are caution NOT ACCURATE just a guess
        {45, (12 * 3) * INCH_TO_METER},
        {47, (12 * 4) * INCH_TO_METER},
        {47, (12 * 5) * INCH_TO_METER},
        {49, (12 * 6) * INCH_TO_METER},
        {49, (12 * 7) * INCH_TO_METER},
        {54, (12 * 8) * INCH_TO_METER},
        {54, (12 * 9) * INCH_TO_METER},
        {57, (12 * 10) * INCH_TO_METER},
        {57, (12 * 11) * INCH_TO_METER}, //These are caution  NOT ACCURATE just a guess
        {57.5, (12 * 12) * INCH_TO_METER},
        {80, (12 * 15) * INCH_TO_METER},
    };

    public double getRPSForDistance(double targetDistanceMeters) {

        for (int i = 0; i < shooterData.length - 1; i++) {

            double rps1 = shooterData[i][0];
            double dist1 = shooterData[i][1];
            double rps2 = shooterData[i + 1][0];
            double dist2 = shooterData[i + 1][1];

            if (targetDistanceMeters >= dist1 && targetDistanceMeters <= dist2) {
                double ratio = (targetDistanceMeters - dist1) / (dist2 - dist1);
                return rps1 + ratio * (rps2 - rps1);
            }
        }
        return shooterData[shooterData.length - 1][0];
    }
}