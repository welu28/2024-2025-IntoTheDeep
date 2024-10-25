/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Autons;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.GoBildaPinpointDriver;

import java.util.Locale;
@Config

@TeleOp(name="goBILDA® PinPoint Odometry Example", group="Linear OpMode")
//@Disabled

public class pidOdom extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
//    public static double x_pos = 0;
    public static double y_pos = 0;
    public static double setpoint = 500;


    private Drivetrain drivetrain;


    @Override
    public void runOpMode() {
        PIDController controller = new PIDController(P, I, D);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();


        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.addData("X offset", odo.getXOffset());
        dashboardTelemetry.addData("Y offset", odo.getYOffset());
        dashboardTelemetry.addData("Device Version Number:", odo.getDeviceVersion());
        dashboardTelemetry.addData("Device Scalar", odo.getYawScalar());
        dashboardTelemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();
        controller.setSetPoint(setpoint);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            odo.update();

            if (gamepad1.a){
                odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.b){
                odo.recalibrateIMU(); //recalibrates the IMU without resetting position
            }

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            Pose2D pos = odo.getPosition();
            double cur_y = pos.getY(DistanceUnit.MM);
            double pid = controller.calculate(cur_y, setpoint); 
            drivetrain.move(pid, 0, 0);
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            dashboardTelemetry.addData("Position", data);

            Pose2D vel = odo.getVelocity();
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            dashboardTelemetry.addData("Velocity", velocity);

//            dashboardTelemetry.addData("Status", odo.getDeviceStatus());

//            dashboardTelemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

//            dashboardTelemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            dashboardTelemetry.addData("Current Position (ticks)", y_pos);
//            dashboardTelemetry.addData("Target (ticks)", targetTicks);
            dashboardTelemetry.addData("Power", pid);
            dashboardTelemetry.addData("Setpoint", setpoint);

            dashboardTelemetry.update();

        }
    }}
