package frc.robot.Util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.function.Function;

public class ArrayUtil {
    public static <T> T firstOrDefault(ArrayList<T> array, Function<T, Boolean> supplier, T defaultValue) {
        for (T v : array) {
            if (supplier.apply(v)) {
                return v;
            }
        }
        return defaultValue;
    }

    public static <T> T firstOrDefault(ArrayList<T> array, Function<T, Boolean> supplier) {
        return firstOrDefault(array, supplier, null);
    }

    public static <T, R> ArrayList<R> select(ArrayList<T> array, Function<T, R> selectionFunction) {
        ArrayList<R> results = new ArrayList<>();
        array.forEach((T v) -> {
            results.add(selectionFunction.apply(v));
        });
        return results;
    }

    public static <T> ArrayList<T> collectionToArray(
            Collection<T> collection) {
        return new ArrayList<T>(collection);
    }
}
