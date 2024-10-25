package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    public DcMotor lift;

    public void init(HardwareMap map) {
        lift = map.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveUp() {
        lift.setPower(-0.3);
    }
    public void moveDown() {
        lift.setPower(0.3);
    }

    public void stop() {
        lift.setPower(0.0);
    }
}