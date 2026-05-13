package org.firstinspires.ftc.teamcode.pedroPathing.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue TeleOp", group = "TeleOp")
public class BlueTeleOp extends CommonTeleOp {
    @Override
    protected TeleOpTuningConfig.AllianceTeleOpConfig getAllianceConfig() {
        return TeleOpTuningConfig.BLUE;
    }
}
