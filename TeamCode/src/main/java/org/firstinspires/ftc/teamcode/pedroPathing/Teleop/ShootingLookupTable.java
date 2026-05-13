package org.firstinspires.ftc.teamcode.pedroPathing.Teleop;

import java.util.ArrayList;
import java.util.List;

/**
 * Reference lookup table for assisted shooting.
 * Populate these entries from field measurement tests.
 */
public class ShootingLookupTable {
    public static class LookupEntry {
        public final double distanceInches;
        public final double rpm;
        public final double hoodPosition;

        public LookupEntry(double distanceInches, double rpm, double hoodPosition) {
            this.distanceInches = distanceInches;
            this.rpm = rpm;
            this.hoodPosition = hoodPosition;
        }
    }

    public static class ShotSolution {
        public final double rpm;
        public final double hoodPosition;

        public ShotSolution(double rpm, double hoodPosition) {
            this.rpm = rpm;
            this.hoodPosition = hoodPosition;
        }
    }

    private final List<LookupEntry> entries = new ArrayList<>();

    public ShootingLookupTable() {
        // Placeholder values. Replace with tested measurements.
        entries.add(new LookupEntry(24.0, 2600.0, 0.20));
        entries.add(new LookupEntry(36.0, 2900.0, 0.35));
        entries.add(new LookupEntry(48.0, 3200.0, 0.50));
        entries.add(new LookupEntry(60.0, 3500.0, 0.65));
    }

    public ShotSolution getNearest(double distanceInches) {
        if (entries.isEmpty()) {
            return new ShotSolution(3000.0, 0.35);
        }

        LookupEntry best = entries.get(0);
        double bestErr = Math.abs(distanceInches - best.distanceInches);
        for (int i = 1; i < entries.size(); i++) {
            LookupEntry e = entries.get(i);
            double err = Math.abs(distanceInches - e.distanceInches);
            if (err < bestErr) {
                best = e;
                bestErr = err;
            }
        }

        return new ShotSolution(best.rpm, best.hoodPosition);
    }
}
