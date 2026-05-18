package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs.ParamsConfig.Alliance;

/**
 * Red alliance – Close side auton: preload, 2 row pickups, then park.
 */
@Autonomous(name = "Red 2R_Close", group = "RedAuton")
public class Red_2R_Close extends AutonBase {
    @Override
    protected AutonConfig getConfig() {
        Path_2R_Close path = new Path_2R_Close();
        return path.getConfig(Alliance.RED);
    }
}