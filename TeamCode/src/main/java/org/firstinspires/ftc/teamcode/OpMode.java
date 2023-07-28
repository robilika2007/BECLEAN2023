package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class OpMode extends LinearOpMode {

    DcMotorEx glis;
    Servo joint;
    Servo cleste;

    public static Pose2d START_POSE = new Pose2d(0, 0, 0);
    public static double jDefault = 0.5; //clestele paralel cu pamantu
    public static double jDrop = 1; //pozitie de drop
    public static double cInchis = 1; //pozitie de drop
    public static double cDeschis = 0.3; //pozitie de drop

    public static int sus = 1500;
    public static int dif = 10; //diferenta intre pozitia a 2 conuri

    public SampleMecanumDrive drive;

    public void runOpMode()
    {
        drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence traj = null;

        glis = this.hardwareMap.get(DcMotorEx.class, "glis");
        joint = this.hardwareMap.get(Servo.class, "joint");
        cleste = this.hardwareMap.get(Servo.class, "cleste");

        glis.setDirection(DcMotorSimple.Direction.FORWARD); //maybe need change
        glis.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        waitForStart();

        joint.setPosition(jDefault);

        drive.setPoseEstimate(START_POSE);
        drive.followTrajectorySequence(traj);
    }
    public TrajectorySequence buildtraj()
    {
        TrajectorySequenceBuilder trajBuilder = drive.trajectorySequenceBuilder(START_POSE);
        trajBuilder
                .forward(20)
                .turn(Math.toRadians(90))
                .forward(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    glis.setTargetPosition(10);
                    glis.setPower(1);

                    cleste.setPosition(cDeschis);
                   for(int i = 0; i < 20; ++i)
                   {
                       cleste.setPosition(cInchis);
                       int poz = glis.getCurrentPosition();
                       glis.setTargetPosition(sus);

                       while(glis.isBusy());
                       joint.setPosition(jDrop);
                       try {
                           wait(100);
                       } catch (InterruptedException e) {
                           e.printStackTrace();
                       }
                       cleste.setPosition(cDeschis);

                       glis.setTargetPosition(poz + dif);
                       glis.setPower(1);

                       joint.setPosition(jDefault);
                       cleste.setPosition(cDeschis);
                   }
                });
        return trajBuilder.build();
    }
}
