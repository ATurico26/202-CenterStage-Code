package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="ArmDown", group="Iterative Opmode")
public class ArmDown extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        Gamepad driver = gamepad1, operator = gamepad2;


        ElapsedTime mRuntime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double LastTime = mRuntime.time();


        double ClawOffset = robot.ClawOffset;
        double VFBPower = 0;
        boolean HuskyLensActive = true;
        boolean HuskyLensToggle = false;


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            // Virtual Four Bar

            //Initial Set Power
            if (gamepad2.right_stick_y < 0) VFBPower = (-0.5 * Math.abs(Math.pow(gamepad2.right_stick_y, 3)));
            else if (gamepad2.right_stick_y >= 0) VFBPower = (0.5 * Math.abs(Math.pow(gamepad2.right_stick_y, 3)));
            else VFBPower = 0;

            robot.VFBRight.setPower(VFBPower);
            robot.VFBLeft.setPower(VFBPower);



            // Drone launcher
            if(gamepad1.a) robot.DroneLauncher.setPosition(0);
            else if (gamepad1.b) {
                robot.DroneLauncher.setPosition(0.5);
                // 0.5 second wait to allow mechanism to fire - might hold any last inputs
                sleep(500);
            }


            // Claw
            if(gamepad2.y) robot.Claw.setPosition(0 + ClawOffset);
            else if (gamepad2.x) robot.Claw.setPosition(0.35 + ClawOffset);



            telemetry.addData("FPS:", Math.round((1 / (mRuntime.time() - LastTime)) * 1000));
            telemetry.addData("MSPerFrame:", (mRuntime.time() - LastTime));
            LastTime = mRuntime.time();
            telemetry.addData("VFB Pos:", robot.VFBLeft.getCurrentPosition());
            telemetry.addData("VFB Vel:", robot.VFBLeft.getVelocity());
            telemetry.addData("Claw:", robot.Claw.getPosition());

            telemetry.addLine("");

            // Toggles husky lens
            if (gamepad1.x && !HuskyLensToggle) {
                HuskyLensActive = !HuskyLensActive;
                HuskyLensToggle = true;
            } else if (!gamepad1.x) HuskyLensToggle = false;

            // HuskyLens Telemetry (if enabled)
            if (HuskyLensActive) {
                HuskyLens.Block[] block = robot.Camera.blocks();
                telemetry.addData("HuskyLens block count:", block.length);
                for (HuskyLens.Block value : block) {
                    telemetry.addLine("ID:" + (value.id) + " X/Y:" + (value.x) + ", " + (value.y) + " h:" + (value.height) + " w:" + (value.width) + " origin:" + (value.left) + ", " + (value.top));
                }
            } else telemetry.addLine("HuskyLens disabled");

            telemetry.update();


        }


    }

}