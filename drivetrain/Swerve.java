package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import frc.robot.SwerveModule;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PoseConfig;
import frc.lib.util.Limelight;
import frc.lib.util.OdometryImpl;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimator; 
    public SwerveModule[] mSwerveMods;
    public OdometryImpl odometryImpl;
    public Pigeon2 gyro;

    public Field2d field;

    public Limelight limelightShooter;
    public Limelight limelightArm;


    public boolean aimedAtSpeaker = false;

    private double speedMultiplier = 1;

    // Initialize pose estimator
    public final Thread poseEstimatorInitializer = new Thread(() -> {
        DriverStation.reportWarning("Initialziing pose estimator", false);
        Pose2d origin = new Pose2d();
        
        if (RobotContainer.alliance == DriverStation.Alliance.Blue) {
            origin = Constants.BlueTeamPoses.blueOrigin;
        }
        else {
            origin = Constants.RedTeamPoses.redOrigin;
        }

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            origin,
            odometryImpl.createStdDevs(PoseConfig.kPositionStdDevX, PoseConfig.kPositionStdDevY, PoseConfig.kPositionStdDevTheta),
            odometryImpl.createStdDevs(PoseConfig.kVisionStdDevX, PoseConfig.kVisionStdDevY, PoseConfig.kVisionStdDevTheta)
        );
    });

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.pigeonCanName);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        field = new Field2d();
        odometryImpl = new OdometryImpl(this);

        limelightShooter = new Limelight(Constants.LimelightConstants.limelightShooter);
        limelightArm = new Limelight(Constants.LimelightConstants.limelightArm);

        limelightShooter.setPipeline(LimelightConstants.limelightShooterTagPipeline);
        limelightArm.setPipeline(LimelightConstants.limelightArmTagPipeline);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
            } catch (Exception e) {}
            resetModulesToAbsolute();
        }).start();

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(
                            Constants.AutoConstants.translationkP, 
                            Constants.AutoConstants.translationkI,
                            Constants.AutoConstants.translationkD
                        ),
                        new PIDConstants(
                            Constants.AutoConstants.rotationkP, 
                            Constants.AutoConstants.rotationkI,
                            Constants.AutoConstants.rotationkD
                        ),
                        4.3,
                        Constants.Swerve.trackWidth / Math.sqrt(2),
                        new ReplanningConfig()
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX() * speedMultiplier, 
                                    translation.getY() * speedMultiplier, 
                                    rotation * speedMultiplier, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX() * speedMultiplier, 
                                    translation.getY() * speedMultiplier, 
                                    rotation * speedMultiplier)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    // Calculates a target angle
    public double calculateTurnAngle(Pose2d target, double robotAngle) {
        double tx = target.getX(); 
        double ty = target.getY(); 
        double rx = getRelativePose().getX();
        double ry = getRelativePose().getY();

        double requestedAngle = Math.atan((ty - ry) / (tx - rx)) * (180 / Math.PI);
        double calculatedAngle = (180 - robotAngle + requestedAngle);

        return ((calculatedAngle + 360) % 360);
    }

    public void toggleMultiplier() {
        speedMultiplier = speedMultiplier == 1 ? 0.2 : 1;
    }

    public boolean isLowGear() {
        return speedMultiplier == 0.2;
    }

    public Pose2d getPose() {
        if(poseEstimator == null) return new Pose2d();
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getRelativePose() {
        if(poseEstimator == null) return new Pose2d();

        if(RobotContainer.alliance == DriverStation.Alliance.Blue) {
            return poseEstimator.getEstimatedPosition();
        }
        else {
            return poseEstimator.getEstimatedPosition().relativeTo(Constants.RedTeamPoses.redOrigin);
        }
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getRelativePose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        Pose2d zeroPose;
        if(RobotContainer.alliance == DriverStation.Alliance.Blue) {
            zeroPose = new Pose2d(getPose().getTranslation(), new Rotation2d());
        }
        else {
            zeroPose = new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(180));
        }
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), zeroPose);
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    private void addLimelightToEstimator(Limelight limelight) {
        if (poseEstimator == null) return;

        Pose2d visionMeasurement = odometryImpl.getVisionMeasurement(limelight);
        if (visionMeasurement != null) {
            Vector<N3> stdDevs = odometryImpl.getCalculatedStdDevs(limelight);
            
            poseEstimator.setVisionMeasurementStdDevs(stdDevs);
            // poseEstimator.setVisionMeasurementStdDevs(odometryImpl.createStdDevs(Constants.PoseConfig.kVisionStdDevX, Constants.PoseConfig.kVisionStdDevY, Constants.PoseConfig.kVisionStdDevTheta));
            poseEstimator.addVisionMeasurement(visionMeasurement, limelight.getLimelightLatency());
        }
    }

    @Override
    public void periodic(){
        if (poseEstimator != null) poseEstimator.update(getGyroYaw(), getModulePositions());
    
        if ((Robot.state != Robot.State.AUTON || RobotContainer.useVisionInAuton()) && RobotContainer.addVisionMeasurement) {
            addLimelightToEstimator(limelightShooter);
            addLimelightToEstimator(limelightArm);
        }

        field.setRobotPose(getPose());

        Logger.recordOutput("Odometry/Robot", getPose());
        Logger.recordOutput("Odometry/Robot3d", new Pose3d(getPose()));

        SmartDashboard.putNumber("swerve/pose error ll arm", odometryImpl.getVisionPoseError(limelightArm)); 
        SmartDashboard.putNumber("swerve/pose error ll shooter", odometryImpl.getVisionPoseError(limelightShooter));

        for(SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("swerve/Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("swerve/Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("swerve/Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
            SmartDashboard.putNumber("swerve/Mod " + mod.moduleNumber + " Voltage", mod.getVoltage());
        }

        SmartDashboard.putData("field", field);
    }

    public void stop() {
        RobotContainer.s_Swerve.drive(
            new Translation2d(0, 0), 
            0, true, true
        );
    }
    
}
