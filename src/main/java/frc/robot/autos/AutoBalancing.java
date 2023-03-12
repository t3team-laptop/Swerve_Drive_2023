package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoBalancing extends CommandBase {
    private Swerve s_Swerve;
    private boolean sideways;
    private boolean phase1;
    private Timer timer;

    /**
     * 
     * @param s_Swerve TODO
     */

    public AutoBalancing(Swerve s_Swerve, boolean sideways) {
        this.s_Swerve = s_Swerve;
        this.sideways = sideways;
        this.timer = new Timer();
        this.phase1 = true;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        if(!sideways && Math.abs(s_Swerve.getPitch().getDegrees()) < 7 && phase1){
            phase1 = false;
            timer.start();
            } else if(sideways && Math.abs(s_Swerve.getRoll().getDegrees()) < 7 && phase1){
                phase1 = false;
                timer.start();
            }
        if(phase1){
            if(!sideways){
            double translationVal = s_Swerve.getPitch().getDegrees() > 0 ? -.25 : .25;
            s_Swerve.drive(
                    new Translation2d(translationVal, 0).times(Constants.Swerve.maxSpeed), 0, true, false);
            } else{
                double translationVal = s_Swerve.getRoll().getDegrees() > 0 ? -.15 : .15;
            s_Swerve.drive(
                    new Translation2d(translationVal, 0).times(Constants.Swerve.maxSpeed), 0, true, false);
            }
        } else{
            if(!sideways){
                double translationVal = s_Swerve.getPitch().getDegrees() > 0 ? -.25 : .25;
                s_Swerve.drive(
                        new Translation2d(translationVal, 0).times(Constants.Swerve.maxSpeed), 0, true, false);
                } else{
                    double translationVal = s_Swerve.getRoll().getDegrees() > 0 ? -.15 : .15;
                s_Swerve.drive(
                        new Translation2d(translationVal, 0).times(Constants.Swerve.maxSpeed), 0, true, false);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(
                new Translation2d(0, .1).times(Constants.Swerve.maxSpeed), 0, true, false);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 2;
    }
}