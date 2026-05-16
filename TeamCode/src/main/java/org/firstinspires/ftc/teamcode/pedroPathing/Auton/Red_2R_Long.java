package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs.Alliance;

/**
 * Red alliance – Long side auton: 1 row pickup + 2 corner cycles, then park.
 *
 * Field origin: Pedro Pathing (0, 0) at the near-red corner.
 * Robot faces away from the field wall (intake forward / shooter + camera backward).
 */
@Autonomous(name = "Red 2R_Long", group = "RedAuton")
public class Red_2R_Long extends AutonBase {
    @Override
    protected AutonConfig getConfig() {

        Path_2R_Long path = new Path_2R_Long();
        return path.getConfig(Alliance.RED);
    }
}
