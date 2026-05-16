package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs.ParamsConfig;

import java.util.ArrayList;
import java.util.List;

/**
 * Red alliance – Long side auton: 1 row pickup + 2 corner cycles, then park.
 *
 * Field origin: Pedro Pathing (0, 0) at the near-red corner.
 * Robot faces away from the field wall (intake forward / shooter + camera backward).
 *
 * Identical to {@link Red_1R_Long} except {@code cornerCycles = 2}.
 */
@Autonomous(name = "Red 2R_Long", group = "RedAuton")
public class Red_2R_Long extends AutonBase {
    @Override
    protected AutonConfig getConfig() {

        Path_2R_Long path = new Path_2R_Long();
        return path.getConfig();
    }
}
