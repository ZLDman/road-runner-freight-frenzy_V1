package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class auto extends LinearOpMode {

    OpenCvCamera webcam;
    openCV.BallCubeOpenCV pipeline;

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera2Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        // Create camera instance
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // With live preview
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Open async and start streaming inside opened callback
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

                pipeline = new openCV.BallCubeOpenCV(telemetry);
                webcam.setPipeline(pipeline);
            }
        });

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(250);

        waitForStart();

        while (opModeIsActive())
        {
            double f = 0;
            double l = 0;

            Rect[] c = openCV.cboundRect;

            if(c != null){
                //left
                if (c[0].tl().x > 180) {
                    l += 0.2;
                }
                //right
                if (c[0].tl().x < 140) {
                    l -= 0.2;
                }

                //forward
                if(c[0].tl().y > 140){
                    f += 0.2;
                }
                //back
                if(c[0].tl().y < 100) {
                    l -= 0.2;
                }

                telemetry.addData("x",c[0].tl().x);
                telemetry.addData("y",c[0].tl().y);
                telemetry.update();
            }
            else{
                telemetry.addData("c is ","null");
                telemetry.addData("c is ",openCV.cboundRect.length);
                telemetry.update();
            }

            //telemetry.update();
            drive.setWeightedDrivePower(new Pose2d(f,0,l));
            //drive.setMotorPowers(f - l,-f + l,-f + l,f - l);
        }
    }
}
