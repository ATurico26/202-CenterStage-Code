package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.OpVariableStorage;

@Autonomous(name = "RedFarLine", group = "Iterative Opmode")
public class RedFar extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        // Build roadrunner trajectories

        Pose2d startPose = new Pose2d(-35.5, -61.44, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory PushPixelToRight = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-39 - 24, -33.3))
                .build();
        Trajectory PushPixelToMiddle = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-18.4 - 24, -34.4))
                .build();
        Trajectory PushPixelToLeft = drive.trajectoryBuilder(startPose)
                .forward(15)
                .splineTo(new Vector2d(-11.3 - 24, -32.3), Math.toRadians(45))
                .build();

        Trajectory GoToParkingSpotRight = drive.trajectoryBuilder(PushPixelToRight.end(), true)
                .strafeTo(new Vector2d(-55, -55))
                .splineToConstantHeading(new Vector2d(-60, -30), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-30, -9, Math.toRadians(180)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(55, -9), Math.toRadians(0))
                .build();
        Trajectory GoToParkingSpotMiddle = drive.trajectoryBuilder(PushPixelToMiddle.end(), true)
                .strafeTo(new Vector2d(-50, -50))
                .splineToConstantHeading(new Vector2d(-55, -30), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-30, -9, Math.toRadians(180)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(55, -9), Math.toRadians(0))
                .build();
        Trajectory GoToParkingSpotLeft = drive.trajectoryBuilder(PushPixelToLeft.end(), true)
                .strafeTo(new Vector2d(-45, 55))
                .splineToConstantHeading(new Vector2d(-50, -30), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-30, -9, Math.toRadians(180)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(55, -9), Math.toRadians(0))
                .build();

        telemetry.addLine("Finished Building Trajectories");
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        //Close claw
        robot.Claw.setPosition(0 + robot.ClawOffset);


        // Find where the team object is, move, and place pixel

        double[] objectLocation = robot.findTeamObjectPixels(new int[]{1});

        if (Math.round(objectLocation[0]) == 2) {
            telemetry.addLine("Object at right");
            telemetry.addData("Confidence: ", objectLocation[1]);
            telemetry.update();

            drive.followTrajectory(PushPixelToLeft);
            sleep(500);
            //sleep(500);
            drive.followTrajectory(GoToParkingSpotLeft);


        }
        else if (Math.round(objectLocation[0]) == 0) {
            telemetry.addLine("Object at left");
            telemetry.addData("Confidence: ", objectLocation[1]);
            telemetry.update();

            drive.followTrajectory(PushPixelToRight);
            sleep(500);
            //sleep(500);
            drive.followTrajectory(GoToParkingSpotRight);


        }
        else {
            telemetry.addLine("Object at middle");
            telemetry.addData("Confidence: ", objectLocation[1]);
            telemetry.update();

            drive.followTrajectory(PushPixelToMiddle);
            sleep(500);
            //sleep(500);
            drive.followTrajectory(GoToParkingSpotMiddle);


        }

        OpVariableStorage.currentPose = drive.getPoseEstimate();
        OpVariableStorage.rotationChange = 0.5;
        //OpVariableStorage.VFBPosition = robot.VFBLeft.getCurrentPosition();

        telemetry.addLine("End of Autonomous");
        telemetry.update();

    }

}
