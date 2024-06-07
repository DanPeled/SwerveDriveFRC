package frc.robot.Util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.function.Function;

/**
 * Utility class for working with arrays and collections.
 */
public class ArrayUtil {

    /**
     * Returns the first element in the array that matches the given condition,
     * or a default value if no such element is found.
     *
     * @param <T>          the type of elements in the array
     * @param array        the array to search
     * @param supplier     the condition to match
     * @param defaultValue the value to return if no match is found
     * @return the first matching element, or the default value if no match is found
     */
    public static <T> T firstOrDefault(ArrayList<T> array, Function<T, Boolean> supplier, T defaultValue) {
        for (T v : array) {
            if (supplier.apply(v)) {
                return v;
            }
        }
        return defaultValue;
    }

    /**
     * Returns the first element in the array that matches the given condition,
     * or null if no such element is found.
     *
     * @param <T>      the type of elements in the array
     * @param array    the array to search
     * @param supplier the condition to match
     * @return the first matching element, or null if no match is found
     */
    public static <T> T firstOrDefault(ArrayList<T> array, Function<T, Boolean> supplier) {
        return firstOrDefault(array, supplier, null);
    }

    /**
     * Returns a new array containing all elements in the given array that match
     * the given condition.
     *
     * @param <T>      the type of elements in the array
     * @param array    the array to filter
     * @param supplier the condition to match
     * @return a new array containing all matching elements
     */
    public static <T> ArrayList<T> filter(ArrayList<T> array, Function<T, Boolean> supplier) {
        ArrayList<T> res = new ArrayList<>();
        array.forEach((v) -> {
            if (supplier.apply(v)) {
                res.add(v);
            }
        });
        return res;
    }

    /**
     * Returns a new array containing all elements from the original array except
     * those
     * that match the given condition.
     *
     * @param <T>      the type of elements in the array
     * @param array    the original array
     * @param supplier the condition to match for removal
     * @return a new array with elements that do not match the condition
     */
    public static <T> ArrayList<T> deduce(ArrayList<T> array, Function<T, Boolean> supplier) {
        ArrayList<T> res = new ArrayList<T>(array);

        res.forEach((v) -> {
            if (supplier.apply(v)) {
                res.remove(v);
            }
        });

        return res;
    }

    /**
     * Returns a new array containing the results of applying the given function
     * to each element in the given array.
     *
     * @param <T>               the type of elements in the input array
     * @param <R>               the type of elements in the resulting array
     * @param array             the array to transform
     * @param selectionFunction the function to apply to each element
     * @return a new array containing the results of the function
     */
    public static <T, R> ArrayList<R> select(ArrayList<T> array, Function<T, R> selectionFunction) {
        ArrayList<R> results = new ArrayList<>();
        array.forEach((T v) -> {
            results.add(selectionFunction.apply(v));
        });
        return results;
    }

    /**
     * Converts a collection to an array.
     *
     * @param <T>        the type of elements in the collection
     * @param collection the collection to convert
     * @return a new array containing all elements in the collection
     */
    public static <T> ArrayList<T> collectionToArray(Collection<T> collection) {
        return new ArrayList<T>(collection);
    }
}
