package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@TeleOp
public class LiftTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift();
        lift.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {

            if(gamepad1.right_bumper) {
                lift.moveUp();
            }
            else if(gamepad1.left_bumper) {
                lift.moveDown();
            }
            else {
                lift.stop();
            }
        }
    }
}
