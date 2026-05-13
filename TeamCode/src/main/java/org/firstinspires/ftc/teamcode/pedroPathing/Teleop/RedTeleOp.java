package org.firstinspires.ftc.teamcode.pedroPathing.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Red TeleOp", group = "TeleOp")
public class RedTeleOp extends CommonTeleOp {
    @Override
    protected TeleOpTuningConfig.AllianceTeleOpConfig getAllianceConfig() {
        return TeleOpTuningConfig.RED;
    }
}
