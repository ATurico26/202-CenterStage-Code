package org.firstinspires.ftc.teamcode.Autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.OpVariableStorage;

@Autonomous(name = "BlueFarPixelPlace", group = "Iterative Opmode")
public class BlueFarPixelPlace extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        // Build roadrunner trajectories

        Pose2d startPose = new Pose2d(-35.34, 61.26, Math.toRadians(90 + 180));

        drive.setPoseEstimate(startPose);

        Trajectory PushPixelToRight = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-32 - 24, 38))
                .build();
        Trajectory PushPixelToMiddle = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-18.4 - 24, 34))
                .build();
        Trajectory PushPixelToLeft = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-15 - 24, 47))
                .splineTo(new Vector2d(-12 - 24, 31), Math.toRadians(135 + 180))
                .build();

        Trajectory GoToParkingSpotRight = drive.trajectoryBuilder(PushPixelToRight.end(), true)
                .splineToConstantHeading(new Vector2d(-35, 50), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-35, 18), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-30, 9, Math.toRadians(180)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, 9), Math.toRadians(0))
                .build();
        Trajectory GoToParkingSpotMiddle = drive.trajectoryBuilder(PushPixelToMiddle.end(), true)
                .strafeTo(new Vector2d(-53, 45))
                .splineToConstantHeading(new Vector2d(-55, 30), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-35, 9, Math.toRadians(180)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, 9), Math.toRadians(0))
                .build();
        Trajectory GoToParkingSpotLeft = drive.trajectoryBuilder(PushPixelToLeft.end(), true)
                .strafeTo(new Vector2d(-45, 40))
                .splineToConstantHeading(new Vector2d(-45, 30), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-30, 9, Math.toRadians(180)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, 9), Math.toRadians(0))
                .build();

        Trajectory MoveToRightBoard = drive.trajectoryBuilder(GoToParkingSpotRight.end(), true)
                .lineToConstantHeading(new Vector2d(47.5, 31))
                .build();
        Trajectory MoveToMiddleBoard = drive.trajectoryBuilder(GoToParkingSpotMiddle.end(), true)
                .lineToConstantHeading(new Vector2d(47.5, 38.5))
                .build();
        Trajectory MoveToLeftBoard = drive.trajectoryBuilder(GoToParkingSpotLeft.end(), true)
                .lineToConstantHeading(new Vector2d(48.5, 46.5))
                .build();

        telemetry.addLine("Finished Building Trajectories");
        double[] testHuskyLens = robot.findTeamObjectPixels(new int[]{2});
        if (testHuskyLens[0] == 0) telemetry.addLine("Location: Left");
        else if (testHuskyLens[0] == 1) telemetry.addLine("Location: Middle");
        else if (testHuskyLens[0] == 2) telemetry.addLine("Location: Right");
        telemetry.addData("Confidence: ", testHuskyLens[1]);
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        ElapsedTime AutonTimer = new ElapsedTime();

        //Close claw
        robot.Claw.setPosition(0 + robot.ClawOffset);


        // Find where the team object is, move, and place pixel

        double[] objectLocation = robot.findTeamObjectPixels(new int[]{2});

        if (Math.round(objectLocation[0]) == 0) {
            telemetry.addLine("Object at left");
            telemetry.addData("Confidence: ", objectLocation[1]);
            telemetry.update();

            drive.followTrajectory(PushPixelToLeft);
            sleep(500);
            //sleep(500);
            drive.followTrajectory(GoToParkingSpotLeft);

            while (AutonTimer.seconds() <= 21) sleep(100);
            drive.followTrajectory(MoveToLeftBoard);
            robot.dropPixelOnBackboard();


        }
        else if (Math.round(objectLocation[0]) == 2) {
            telemetry.addLine("Object at right");
            telemetry.addData("Confidence: ", objectLocation[1]);
            telemetry.update();

            drive.followTrajectory(PushPixelToRight);
            sleep(500);
            //sleep(500);
            drive.followTrajectory(GoToParkingSpotRight);

            while (AutonTimer.seconds() <= 21) sleep(100);
            drive.followTrajectory(MoveToRightBoard);
            robot.dropPixelOnBackboard();


        }
        else {
            telemetry.addLine("Object at middle");
            telemetry.addData("Confidence: ", objectLocation[1]);
            telemetry.update();

            drive.followTrajectory(PushPixelToMiddle);
            sleep(500);
            //sleep(500);
            drive.followTrajectory(GoToParkingSpotMiddle);

            while (AutonTimer.seconds() <= 21) sleep(100);
            drive.followTrajectory(MoveToMiddleBoard);
            robot.dropPixelOnBackboard();


        }

        OpVariableStorage.currentPose = drive.getPoseEstimate();
        OpVariableStorage.rotationChange = -0.5;
        OpVariableStorage.VFBPosition = robot.VFBLeft.getCurrentPosition();

        telemetry.addLine("End of Autonomous");
        telemetry.update();

    }

}
