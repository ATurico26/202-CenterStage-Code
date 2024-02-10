package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.advanced.OpVariableStorage;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="TeleOp", group="Iterative Opmode")
public class BasicTeleOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        Gamepad driver = gamepad1, operator = gamepad2;


        ElapsedTime mRuntime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double LastTime = mRuntime.time();


        double VFBPower = 0;
        double VFBOffset = OpVariableStorage.VFBPosition;
        double RelativeRotation = 0; // Rotation relative to starting rotation
        boolean VFBLimiters = true;
        boolean VFBSetting = false;
        double AbsoluteX = 0;
        double AbsoluteY = 0;
        boolean AbsoluteSetting = false;
        double TurnControl = 0;
        double AbsDrivingDirection = OpVariableStorage.rotationChange;
        double FrameRate = 0;

        // temporary TurnControl PID
        //double PIDVar = 0; // 0 = kp, 1 = ki, 2 = kd
        //double PIDChangeIncrement = 0.01;


        // RoadRunner
        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();
        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        //Starting Position
        myLocalizer.setPoseEstimate(OpVariableStorage.currentPose);

        telemetry.addData("Last VFB Pos:", OpVariableStorage.VFBPosition);
        telemetry.addData("StartingPose:", OpVariableStorage.currentPose);
        if (AbsDrivingDirection == -0.5) telemetry.addLine("Starting on BLUE");
        else telemetry.addLine("Starting on RED");
        telemetry.update();


        waitForStart();



        while (opModeIsActive() && !isStopRequested()) {

            // Get Current Pose from Road Runner
            myLocalizer.update();
            Pose2d myPose = myLocalizer.getPoseEstimate();


            RelativeRotation = ((-1 * myPose.getHeading()) / Math.PI) + ((Math.signum(myPose.getHeading() - (3 * Math.PI / 2))) + Math.abs(Math.signum(myPose.getHeading() - (3 * Math.PI / 2)))) + AbsDrivingDirection;

            if (Math.abs(gamepad1.right_stick_x) < 0.05 && Math.abs(gamepad1.right_stick_y) < 0.05) {
                AbsoluteX = Math.cos(Math.PI * RelativeRotation) * (-0.4 * Math.abs(gamepad2.left_stick_x) * gamepad2.left_stick_x) +
                        Math.sin(Math.PI * RelativeRotation) * (-0.4 * Math.abs(gamepad2.left_stick_y) * gamepad2.left_stick_y);
                AbsoluteY = -1 * Math.sin(Math.PI * RelativeRotation) * (-0.4 * Math.abs(gamepad2.left_stick_x) * gamepad2.left_stick_x) +
                        Math.cos(Math.PI * RelativeRotation) * (-0.4 * Math.abs(gamepad2.left_stick_y) * gamepad2.left_stick_y);
            } else {
                AbsoluteX = Math.cos(Math.PI * RelativeRotation) * (-1 * Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x) +
                        Math.sin(Math.PI * RelativeRotation) * (-1 * Math.abs(gamepad1.right_stick_y) * gamepad1.right_stick_y);
                AbsoluteY = -1 * Math.sin(Math.PI * RelativeRotation) * (-1 * Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x) +
                        Math.cos(Math.PI * RelativeRotation) * (-1 * Math.abs(gamepad1.right_stick_y) * gamepad1.right_stick_y);
            }

            // Auto rotate to face VFB towards the board
            if (gamepad1.right_stick_button) {
                TurnControl = -1 * robot.PosPID(robot.angleDifference(Math.toDegrees(myPose.getHeading()), 180), 0);
            } else TurnControl = Math.pow(gamepad1.left_stick_x, 3); // Math.abs(gamepad1.left_stick_x) * gamepad1.left_stick_x

            robot.driveWithControllers(AbsoluteX, AbsoluteY, TurnControl * (1 - 0.4 * gamepad1.left_trigger), 1 - 0.6 * gamepad1.left_trigger);


            // temporary TurnControl PID
            //if (gamepad1.left_bumper) PIDChangeIncrement = 0.01;
            //else PIDChangeIncrement = 0.0001;

            // Reset Absolute Driving encoders
            if (gamepad1.dpad_right && !AbsoluteSetting) { // Align absolute driving to red side
                //PIDVar = PIDVar + 1;
                //if (PIDVar > 2) PIDVar = 0;
                AbsDrivingDirection = 0.5;
                AbsoluteSetting = true;
            } else if (gamepad1.dpad_left && !AbsoluteSetting) { // Align absolute driving to blue side
                //PIDVar = PIDVar - 1;
                //if (PIDVar < 0) PIDVar = 2;
                AbsDrivingDirection = -0.5;
                AbsoluteSetting = true;
            } else if (gamepad1.dpad_up && !AbsoluteSetting) { // emergency set current heading to 0
                //if (PIDVar == 0) robot.VFBKp = (robot.VFBKp + PIDChangeIncrement);
                //else if (PIDVar == 1) robot.VFBKi = (robot.VFBKi + PIDChangeIncrement);
                //else if (PIDVar == 2) robot.VFBKd = (robot.VFBKd + PIDChangeIncrement);

                myLocalizer.setPoseEstimate(new Pose2d(myPose.getX(), myPose.getY(), Math.toRadians(180)));
                //if (gamepad1.left_trigger > 0.2)
                AbsoluteSetting = true;
            } if (gamepad1.dpad_down) {
                //if (PIDVar == 0 && robot.VFBKp > 0) robot.VFBKp = (robot.VFBKp - PIDChangeIncrement);
                //else if (PIDVar == 1 && robot.VFBKi > 0) robot.VFBKi = (robot.VFBKi - PIDChangeIncrement);
                //else if (PIDVar == 2 && robot.VFBKd > 0) robot.VFBKd = (robot.VFBKd - PIDChangeIncrement);

                //if (gamepad1.left_trigger > 0.2)
                AbsoluteSetting = true;
            } else if (!gamepad1.dpad_up && !gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad1.dpad_down) AbsoluteSetting = false;


            // Virtual Four Bar

            VFBLimiters = !gamepad2.left_bumper;
            if (VFBLimiters) {
                if (gamepad2.dpad_right) { // use PID for hanging
                    VFBPower = robot.VFBPID(-780, robot.VFBLeft.getCurrentPosition());

                } else {
                    robot.VFBPIDtimer.reset(); // prevent PID from freaking out when not active


                    //Set power normally with limiters
                    if (gamepad2.right_stick_y < 0 && (robot.VFBLeft.getCurrentPosition() + VFBOffset) > -4000) VFBPower = (-1 * Math.abs(Math.pow(gamepad2.right_stick_y, 3)));
                    else if (gamepad2.right_stick_y >= 0 && (robot.VFBLeft.getCurrentPosition() + VFBOffset) < 0) VFBPower = (1 * Math.abs(Math.pow(gamepad2.right_stick_y, 3)));
                    else VFBPower = 0;
                }
                //Stop power in direction that will break claw
                if (robot.Claw.getPosition() >= 0.1 + robot.ClawOffset && ((robot.VFBLeft.getCurrentPosition() + VFBOffset) <= -600 && (robot.VFBLeft.getCurrentPosition() + VFBOffset) >= -1800) && VFBPower < 0) VFBPower = 0;
                else if (robot.Claw.getPosition() >= 0.1 + robot.ClawOffset && ((robot.VFBLeft.getCurrentPosition() + VFBOffset) <= -1800 && (robot.VFBLeft.getCurrentPosition() + VFBOffset) >= -3100) && VFBPower > 0) VFBPower = 0;


            } else {
                robot.VFBPIDtimer.reset(); // prevent PID from freaking out when not active

                // VFB power without limiters
                if (gamepad2.right_stick_y < 0) VFBPower = (-1 * Math.abs(Math.pow(gamepad2.right_stick_y, 3)));
                else if (gamepad2.right_stick_y > 0) VFBPower = (1 * Math.abs(Math.pow(gamepad2.right_stick_y, 3)));
                else VFBPower = 0;
            }

            robot.VFBRight.setPower(VFBPower);
            robot.VFBLeft.setPower(VFBPower);

            // Reset VFB Position
            if (gamepad2.dpad_up && !VFBSetting) { // Reset Current VFB Position to 0
                VFBOffset = -1 * robot.VFBLeft.getCurrentPosition();
                VFBSetting = true;
            } else if (!gamepad2.dpad_up) VFBSetting = false;



            // Intake
            if (gamepad2.right_trigger > 0.05) robot.Intake.setPower(1 * gamepad2.right_trigger);
            else if (gamepad2.left_trigger > 0.05) robot.Intake.setPower(-1 * gamepad2.left_trigger);
            else if (gamepad1.right_trigger > 0.05) robot.Intake.setPower(1 * gamepad1.right_trigger);
            else robot.Intake.setPower(0);


            // Drone launcher
            if (gamepad1.a) robot.DroneLauncher.setPosition(0);
            else if (gamepad1.b) {
                robot.DroneLauncher.setPosition(0.5);
                // 0.5 second wait to allow mechanism to fire - might hold any last inputs
                sleep(500);
            }


            // Claw
            if (gamepad2.y) robot.Claw.setPosition(0 + robot.ClawOffset);
            else if (gamepad2.x) robot.Claw.setPosition(0.2 + robot.ClawOffset);


            FrameRate = Math.round((1 / (mRuntime.time() - LastTime)) * 1000);

            telemetry.addData("FPS:", FrameRate);
            telemetry.addData("MSPerFrame:", (mRuntime.time() - LastTime));
            LastTime = mRuntime.time();
            telemetry.addData("X:", myPose.getX());
            telemetry.addData("Y:", myPose.getY());
            telemetry.addData("Heading:", Math.toDegrees(myPose.getHeading()));
            telemetry.addData("Relative Rotation:", (RelativeRotation * 180));
            telemetry.addData("Claw:", robot.Claw.getPosition());
            telemetry.addData("VFB Pos:", (robot.VFBLeft.getCurrentPosition() + VFBOffset));
            if (!VFBLimiters) telemetry.addLine("VFB Limiters disabled");
            //telemetry.addLine(" ");
            //telemetry.addData("VFBPower:", VFBPower);
            //if (PIDVar == 0) telemetry.addLine("Setting: Kp");
            //else if (PIDVar == 1) telemetry.addLine("Setting: Ki");
            //else if (PIDVar == 2) telemetry.addLine("Setting: Kd");
            //telemetry.addData("Kp: ", robot.VFBKp);
            //telemetry.addData("Ki: ", robot.VFBKi);
            //telemetry.addData("Kd: ", robot.VFBKd);
            telemetry.update();
        }
    }
}
