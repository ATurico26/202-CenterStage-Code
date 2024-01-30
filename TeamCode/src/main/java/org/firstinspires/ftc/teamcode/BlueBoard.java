package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.OpVariableStorage;

@Autonomous(name = "BlueBoard", group = "Iterative Opmode")
public class BlueBoard extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        // Build roadrunner trajectories

        Pose2d startPose = new Pose2d(12, 61.44, Math.toRadians(90 + 180));

        drive.setPoseEstimate(startPose);

        Trajectory PushPixelToRight = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(32, 30.3))
                .build();
        Trajectory PushPixelToMiddle = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(18.4, 34.4))
                .build();
        Trajectory PushPixelToLeft = drive.trajectoryBuilder(startPose)
                .forward(15)
                .splineTo(new Vector2d(12.3, 32.3), Math.toRadians(135 + 90))
                .build();

        Trajectory MoveToRightBoard = drive.trajectoryBuilder(PushPixelToRight.end(), true)
                .back(15)
                .splineTo(new Vector2d(49, 41.5), Math.toRadians(0))
                .build();
        Trajectory MoveToMiddleBoard = drive.trajectoryBuilder(PushPixelToMiddle.end(), true)
                .back(10)
                .splineTo(new Vector2d(49, 35.1), Math.toRadians(0))
                .build();
        Trajectory MoveToLeftBoard = drive.trajectoryBuilder(PushPixelToLeft.end(), true)
                .back(10)
                .splineTo(new Vector2d(49, 27), Math.toRadians(0))
                .build();

        Trajectory GoToParkingSpotRight = drive.trajectoryBuilder(MoveToRightBoard.end())
                .lineToConstantHeading(new Vector2d(45, 65))
                .build();
        Trajectory GoToParkingSpotMiddle = drive.trajectoryBuilder(MoveToMiddleBoard.end())
                .lineToConstantHeading(new Vector2d(45, 65))
                .build();
        Trajectory GoToParkingSpotLeft = drive.trajectoryBuilder(MoveToLeftBoard.end())
                .lineToConstantHeading(new Vector2d(45, 65))
                .build();

        telemetry.addLine("Finished Building Trajectories");
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        //Close claw
        robot.Claw.setPosition(0 + robot.ClawOffset);


        // Find where the team object is, move, and place pixel

        double[] objectLocation = robot.findTeamObject(2);

        if (Math.round(objectLocation[0]) == 2) {
            telemetry.addLine("Object at right");
            telemetry.addData("Confidence:", objectLocation[1]);
            telemetry.update();

            drive.followTrajectory(PushPixelToLeft);
            sleep(500);
            drive.followTrajectory(MoveToLeftBoard);
            //sleep(500);
            robot.dropPixelOnBackboard();
            //sleep(500);
            drive.followTrajectory(GoToParkingSpotLeft);


        }
        else if (Math.round(objectLocation[0]) == 1) {
            telemetry.addLine("Object at left");
            telemetry.addData("Confidence:", objectLocation[1]);
            telemetry.update();

            drive.followTrajectory(PushPixelToRight);
            sleep(500);
            drive.followTrajectory(MoveToRightBoard);
            //sleep(500);
            robot.dropPixelOnBackboard();
            //sleep(500);
            drive.followTrajectory(GoToParkingSpotRight);


        }
        else {
            telemetry.addLine("Object at middle");
            telemetry.addData("Confidence:", objectLocation[1]);
            telemetry.update();

            drive.followTrajectory(PushPixelToMiddle);
            sleep(500);
            drive.followTrajectory(MoveToMiddleBoard);
            //sleep(500);
            robot.dropPixelOnBackboard();
            //sleep(500);
            drive.followTrajectory(GoToParkingSpotMiddle);




        }

        OpVariableStorage.currentPose = drive.getPoseEstimate();
        OpVariableStorage.rotationChange = -0.5;
        //OpVariableStorage.VFBPosition = robot.VFBLeft.getCurrentPosition();

        telemetry.addLine("End of Autonomous");
        telemetry.update();

    }

}
