public class Pair implements Comparable<Pair> {
    public final int index;
    public final float value;

    public Pair(int index, float value) {
        this.index = index;
        this.value = value;
    }

    @Override
    public int compareTo(Pair other) {
        return Float.valueOf(this.value).compareTo(other.value);
    }
}
