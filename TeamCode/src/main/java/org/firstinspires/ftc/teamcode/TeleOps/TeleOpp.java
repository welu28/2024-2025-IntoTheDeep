package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp
public class TeleOpp extends LinearOpMode {

    private Drivetrain drive = new Drivetrain();

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            drive.move(0.5,0,0);
            sleep(250);

//            telemetry.addData("Power", power);
//            telemetry.addData("Strafe", strafe);
//            telemetry.addData("Turn", turn);
//            telemetry.update();
        }
    }
}