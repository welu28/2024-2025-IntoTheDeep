package org.firstinspires.ftc.teamcode.Autons;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class PIDF extends LinearOpMode {
    private PIDController controller;

    // PID coefficients that can be tuned from the FTC Dashboarde
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    public static int target = 200;

    private final double ticks_in_degree = 8192 / 360.0;

    private DcMotor frontRight;
    private DcMotor frontLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(P, I, D);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            controller.setPID(P, I, D);

            int currentPosition = frontRight.getCurrentPosition();

            double pid = controller.calculate(currentPosition, target);

            double power = pid + F; // fixed for linear slide

            frontRight.setPower(0);
            frontLeft.setPower(0);

            telemetry.addData("Position", currentPosition);
            telemetry.addData("Target", target);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}
