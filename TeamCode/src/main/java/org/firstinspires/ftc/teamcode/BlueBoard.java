package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "BlueBoard", group = "Iterative Opmode")
public class BlueBoard extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        // Build roadrunner trajectories

        Pose2d startPose = new Pose2d(11.78, -61.55, Math.toRadians(90));

        drive.setPoseEstimate(startPose);




        waitForStart();

        //Close claw
        robot.Claw.setPosition(0);

        sleep(500);


        // Find where the team object is, move, and place pixel

        double[] objectLocation = robot.teamObjectPosition(11, 10);

        if (Math.round(objectLocation[0]) == 0) {
            telemetry.addLine("Object at left");
            telemetry.addData("Confidence:", objectLocation[1]);
            telemetry.update();

            sleep(500);


        }
        else if (Math.round(objectLocation[0]) == 2) {
            telemetry.addLine("Object at right");
            telemetry.addData("Confidence:", objectLocation[1]);
            telemetry.update();

            sleep(500);


        }
        else {
            telemetry.addLine("Object at middle");
            telemetry.addData("Confidence:", objectLocation[1]);
            telemetry.update();

            sleep(500);


        }

        sleep(500);


        telemetry.addLine("End of Autonomous");
        telemetry.update();

    }

}
