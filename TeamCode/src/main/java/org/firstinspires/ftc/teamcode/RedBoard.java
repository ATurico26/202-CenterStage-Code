package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

@Autonomous(name = "RedBoard", group = "Iterative Opmode")
public class RedBoard extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        // Build roadrunner trajectories

        Pose2d startPose = new Pose2d(12, -61.44, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory PushPixelToRight = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35, -39.3))
                .build();
        Trajectory PushPixelToMiddle = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(18.4, -32.4))
                .build();
        Trajectory PushPixelToLeft = drive.trajectoryBuilder(startPose)
                .forward(15)
                .splineTo(new Vector2d(10.3, -32.3), Math.toRadians(135))
                .build();

        Trajectory MoveToRightBoard = drive.trajectoryBuilder(PushPixelToRight.end(), true)
                .back(10)
                .splineTo(new Vector2d(49, -41.5), Math.toRadians(0))
                .build();
        Trajectory MoveToMiddleBoard = drive.trajectoryBuilder(PushPixelToMiddle.end(), true)
                .back(10)
                .splineTo(new Vector2d(49, -35.1), Math.toRadians(0))
                .build();
        Trajectory MoveToLeftBoard = drive.trajectoryBuilder(PushPixelToLeft.end(), true)
                .back(10)
                .splineTo(new Vector2d(49, -28.5), Math.toRadians(0))
                .build();

        Trajectory GoToParkingSpotRight = drive.trajectoryBuilder(MoveToRightBoard.end())
                .lineToConstantHeading(new Vector2d(48, -62))
                .build();
        Trajectory GoToParkingSpotMiddle = drive.trajectoryBuilder(MoveToMiddleBoard.end())
                .lineToConstantHeading(new Vector2d(48, -62))
                .build();
        Trajectory GoToParkingSpotLeft = drive.trajectoryBuilder(MoveToLeftBoard.end())
                .lineToConstantHeading(new Vector2d(48, -62))
                .build();


        waitForStart();

        //Close claw
        robot.Claw.setPosition(0);

        sleep(250);


        // Find where the team object is, move, and place pixel

        double[] objectLocation = robot.teamObjectPosition(11, 10);

        if (Math.round(objectLocation[0]) == 0) {
            telemetry.addLine("Object at left");
            telemetry.addData("Confidence:", objectLocation[1]);
            telemetry.update();

            //sleep(250);

            drive.followTrajectory(PushPixelToLeft);
            sleep(500);
            drive.followTrajectory(MoveToLeftBoard);
            sleep(500);
            drive.followTrajectory(GoToParkingSpotLeft);


        }
        else if (Math.round(objectLocation[0]) == 2) {
            telemetry.addLine("Object at right");
            telemetry.addData("Confidence:", objectLocation[1]);
            telemetry.update();

            //sleep(250);

            drive.followTrajectory(PushPixelToRight);
            sleep(500);
            drive.followTrajectory(MoveToRightBoard);
            sleep(500);
            drive.followTrajectory(GoToParkingSpotRight);


        }
        else {
            telemetry.addLine("Object at middle");
            telemetry.addData("Confidence:", objectLocation[1]);
            telemetry.update();

            //sleep(250);

            drive.followTrajectory(PushPixelToMiddle);
            sleep(500);
            drive.followTrajectory(MoveToMiddleBoard);
            sleep(500);
            drive.followTrajectory(GoToParkingSpotMiddle);




        }

        PoseStorage.currentPose = drive.getPoseEstimate();

        telemetry.addLine("End of Autonomous");
        telemetry.update();

    }

}
