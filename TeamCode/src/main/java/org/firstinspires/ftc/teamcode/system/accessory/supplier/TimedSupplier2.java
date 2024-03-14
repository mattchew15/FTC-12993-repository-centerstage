package org.firstinspires.ftc.teamcode.system.accessory.supplier;

import java.util.function.Consumer;
import java.util.function.Supplier;

@Deprecated
public class TimedSupplier2<T>
{

    private double period;
    private Long timer;
    private boolean running;
    private Supplier<T> supplier;
    private T value;

    public TimedSupplier2(Supplier<T> supplier, double period)
    {
        this.period = period;
        this.timer = (long) (System.currentTimeMillis() + period);
        this.supplier = supplier;
        this.running = false;

    }

    public void update()
    {
        if (!running && System.currentTimeMillis() > timer)
        {
            CompleteSupplier<T> supplier1 = supplier.getSupplier();
            running = true;
            supplier1.thenAccept((result) -> {
                this.value = result;
                this.running = false;
            });
        }
    }

    public T get()
    {
        return value;
    }
    @FunctionalInterface
    public interface Supplier<T>
    {
        CompleteSupplier<T> getSupplier();
    }

    public interface CompleteSupplier<T>
    {
        CompleteSupplier<Void> thenAccept(Consumer<? super T> action);
    }
}