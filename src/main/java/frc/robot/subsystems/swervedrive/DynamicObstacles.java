package frc.robot.subsystems.swervedrive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.NoSuchElementException;
import java.util.function.Supplier;

import javax.management.openmbean.KeyAlreadyExistsException;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Util.ArrayUtil;

/**
 * Represents a collection of obstacles on the field.
 * 
 * <p>
 * Each obstacle is defined by a pair of {@link Translation2d} objects, where
 * the first represents the start point and the second represents the end point
 * of the obstacle.
 */
public class DynamicObstacles {
    private HashMap<String, Pair<Translation2d, Translation2d>> m_obstacles;
    private HashMap<String, Pair<Translation2d, Translation2d>> m_initialObstacles;
    private final Supplier<Translation2d> m_translation2DSupplier;

    /**
     * Constructs an instance of {@code DynamicObstacles}.
     * 
     * <p>
     * The initial set of obstacles is empty.
     * 
     * @param translation2DSupplier A supplier function to provide the current
     *                              robot position as a {@link Translation2d}.
     */
    public DynamicObstacles(Supplier<Translation2d> translation2DSupplier) {
        this(new ArrayList<>(), translation2DSupplier);
    }

    /**
     * Constructs an instance of {@code DynamicObstacles} with an initial set of
     * obstacles.
     * 
     * @param initialObstacles      The initial set of obstacles, each defined by a
     *                              pair of {@link Translation2d} objects.
     * @param translation2DSupplier A supplier function to provide the current
     *                              robot position as a {@link Translation2d}.
     */
    public DynamicObstacles(ArrayList<Pair<Translation2d, Translation2d>> initialObstacles,
            Supplier<Translation2d> translation2DSupplier) {
        this.m_initialObstacles = new HashMap<>();
        setDefault(initialObstacles);
        this.m_obstacles = new HashMap<>(m_initialObstacles);
        this.m_translation2DSupplier = translation2DSupplier;
    }

    /**
     * Adds an obstacle to the field with an auto-generated identifier.
     * 
     * <p>
     * The obstacle is represented by a pair of {@link Translation2d} objects,
     * where the first represents the start point and the second represents the end
     * point of the obstacle.
     *
     * @param obstacle A pair of {@link Translation2d} objects representing the
     *                 start and end points of the obstacle.
     * @throws KeyAlreadyExistsException If an obstacle with the generated
     *                                   identifier already exists.
     */
    public void addObstacle(Pair<Translation2d, Translation2d> obstacle) throws KeyAlreadyExistsException {
        addObstacle(Integer.toString(m_obstacles.size()), obstacle);
    }

    /**
     * Adds an obstacle to the field with a specified identifier.
     * 
     * <p>
     * The obstacle is represented by a pair of {@link Translation2d} objects,
     * where the first represents the start point and the second represents the end
     * point of the obstacle.
     *
     * @param name     The identifier for the obstacle.
     * @param obstacle A pair of {@link Translation2d} objects representing the
     *                 start and end points of the obstacle.
     * @throws KeyAlreadyExistsException If an obstacle with the specified
     *                                   identifier already exists.
     */
    public void addObstacle(String name, Pair<Translation2d, Translation2d> obstacle) throws KeyAlreadyExistsException {
        if (!m_obstacles.containsKey(name)) {
            m_obstacles.put(name, obstacle);
        } else {
            throw new KeyAlreadyExistsException("Obstacle with that name" + "( " + name + " )" + " already exists!");
        }
    }

    /**
     * Updates the specified obstacle with new values.
     *
     * @param name          The identifier for the obstacle.
     * @param obstacleValue A pair of {@link Translation2d} objects representing
     *                      the new start and end points of the obstacle.
     * @throws NoSuchElementException If no obstacle with the specified identifier
     *                                exists.
     */
    public void updateObstacle(String name, Pair<Translation2d, Translation2d> obstacleValue)
            throws NoSuchElementException {
        if (m_obstacles.containsKey(name)) {
            m_obstacles.replace(name, obstacleValue);
        } else {
            throw new NoSuchElementException("No such obstacle with name " + "( " + name + " )" + "exists!");
        }
    }

    /**
     * Removes an obstacle from the field.
     * 
     * <p>
     * The obstacle to be removed is represented by a pair of {@link Translation2d}
     * objects, where the first represents the start point and the second represents
     * the end point of the obstacle.
     *
     * @param obstacle A pair of {@link Translation2d} objects representing the
     *                 start and end points of the obstacle to be removed.
     */
    public void removeObstacle(Pair<Translation2d, Translation2d> obstacle) {
        for (String key : m_obstacles.keySet()) {
            if (m_obstacles.get(key) == obstacle) {
                removeObstacle(key);
            }
        }
    }

    /**
     * Removes an obstacle from the field by its identifier.
     *
     * @param obstacleName The identifier for the obstacle to be removed.
     */
    public void removeObstacle(String obstacleName) {
        m_obstacles.remove(obstacleName);
    }

    /**
     * Resets the obstacles to their initial state.
     * 
     * <p>
     * This method restores the obstacles to the initial set of obstacles
     * provided during construction.
     */
    public void reset() {
        this.m_obstacles = new HashMap<>(m_initialObstacles);
        Pathfinding.setDynamicObstacles(getObstacles(),
                m_translation2DSupplier.get());
    }

    /**
     * Clears all obstacles from the field.
     * 
     * <p>
     * This method removes all obstacles from the field, effectively clearing it.
     * It also updates the dynamic obstacles for pathfinding.
     */
    public void clearObstacles() {
        m_obstacles.clear();
        Pathfinding.setDynamicObstacles(getObstacles(),
                m_translation2DSupplier.get());
    }

    /**
     * Sets the default set of obstacles.
     * 
     * @param obstacles The default set of obstacles, each defined by a pair of
     *                  {@link Translation2d} objects.
     */
    public void setDefault(ArrayList<Pair<Translation2d, Translation2d>> obstacles) {
        m_initialObstacles = new HashMap<>();
        for (int i = 0; i < obstacles.size(); i++) {
            m_initialObstacles.put(Integer.toString(i), obstacles.get(i));
        }
    }

    /**
     * Gets the default set of obstacles.
     * 
     * @return The default set of obstacles, each defined by a pair of
     *         {@link Translation2d} objects.
     */
    public ArrayList<Pair<Translation2d, Translation2d>> getDefault() {
        return ArrayUtil.collectionToArray(m_initialObstacles.values());
    }

    /**
     * Gets the current set of obstacles.
     * 
     * @return The current set of obstacles, each defined by a pair of
     *         {@link Translation2d} objects.
     */
    public ArrayList<Pair<Translation2d, Translation2d>> getObstacles() {
        return ArrayUtil.collectionToArray(m_obstacles.values());
    }

    public void assign() {
        // TODO
    }
}
