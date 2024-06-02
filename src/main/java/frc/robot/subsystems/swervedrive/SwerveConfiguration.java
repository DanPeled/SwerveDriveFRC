package frc.robot.subsystems.swervedrive;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * The SwerveConfiguration class represents a configuration file for the swerve
 * drive.
 * It constructs a file path based on the given configuration name, relative to
 * the
 * deployment directory.
 */
public class SwerveConfiguration {
    private final File m_file;

    /**
     * Constructs a SwerveConfiguration object.
     *
     * @param configurationName The name of the configuration file (excluding the
     *                          swerve/
     *                          directory from the name).
     */
    public SwerveConfiguration(String configurationName) {
        m_file = new File(Filesystem.getDeployDirectory(), "swerve/" + configurationName);
    }

    /**
     * Returns the configuration file associated with this SwerveConfiguration.
     *
     * @return The configuration file.
     */
    public File getFile() {
        return m_file;
    }
}
