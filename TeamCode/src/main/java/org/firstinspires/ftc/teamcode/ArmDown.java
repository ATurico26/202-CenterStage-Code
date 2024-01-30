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
        int HuskyIDCheck = 0;
        boolean HuskyIDSwitch = false;
        boolean HuskyAutonomousCheck = false;
        double[] teamObjectLocation = robot.findTeamObject(0);

        telemetry.addLine("Both colors");
        if (teamObjectLocation[0] == 0) telemetry.addLine("Location: Left");
        else if (teamObjectLocation[0] == 1) telemetry.addLine("Location: Middle");
        else if (teamObjectLocation[0] == 2) telemetry.addLine("Location: Right");
        telemetry.addData("Confidence: ", teamObjectLocation[1]);
        telemetry.update();


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
            else if (gamepad2.x) robot.Claw.setPosition(0.15 + ClawOffset);



            telemetry.addData("FPS:", Math.round((1 / (mRuntime.time() - LastTime)) * 1000));
            telemetry.addData("MSPerFrame:", (mRuntime.time() - LastTime));
            LastTime = mRuntime.time();
            telemetry.addData("VFB Pos:", robot.VFBLeft.getCurrentPosition());
            telemetry.addData("VFB Vel:", robot.VFBLeft.getVelocity());
            telemetry.addData("Claw:", robot.Claw.getPosition());

            telemetry.addLine("");

            // checks position of object like it would in autonomous
            if (gamepad1.y && !HuskyAutonomousCheck) {
                teamObjectLocation = robot.findTeamObject(HuskyIDCheck);
                HuskyAutonomousCheck = true;
            } else if (!gamepad1.y) HuskyAutonomousCheck = false;

            // Toggles husky lens
            if (gamepad1.x && !HuskyIDSwitch) {
                HuskyIDCheck++;
                if (HuskyIDCheck > 2) HuskyIDCheck = 0;
                HuskyIDSwitch = true;
            } else if (!gamepad1.x) HuskyIDSwitch = false;

            // HuskyLens Telemetry (if enabled)
            HuskyLens.Block[] block = robot.Camera.blocks();
            telemetry.addData("HuskyLens block count:", block.length);
            for (HuskyLens.Block value : block) {
                telemetry.addLine("ID:" + (value.id) + " X/Y:" + (value.x) + ", " + (value.y) + " h:" + (value.height) + " w:" + (value.width) + " origin:" + (value.left) + ", " + (value.top));
            }

            telemetry.addLine(" ");
            if (HuskyIDCheck == 0) telemetry.addLine("Both colors");
            else if (HuskyIDCheck == 1) telemetry.addLine("Red");
            else if (HuskyIDCheck == 2) telemetry.addLine("Blue");

            if (teamObjectLocation[0] == 0) telemetry.addLine("Location: Left");
            else if (teamObjectLocation[0] == 1) telemetry.addLine("Location: Middle");
            else if (teamObjectLocation[0] == 2) telemetry.addLine("Location: Right");
            telemetry.addData("Confidence: ", teamObjectLocation[1]);

            telemetry.update();


        }


    }

}