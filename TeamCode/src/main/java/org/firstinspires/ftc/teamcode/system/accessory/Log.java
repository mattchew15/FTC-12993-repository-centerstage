package org.firstinspires.ftc.teamcode.system.accessory;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.Arrays;
import java.util.Objects;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Log
{
    private static final String BASE_FOLDER_NAME = "Log";
    private Writer fileWriter;
    private String line;
    private boolean logTime;
    private long startTime;
    private boolean disabled = false;
    private Telemetry telemetry;

    public Log(String filename, boolean logTime, Telemetry telemetry) {
        this.telemetry = telemetry;
        if (logTime) startTime = System.nanoTime();
        this.logTime = logTime;
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        //noinspection ResultOfMethodCallIgnored
        directory.mkdir();
        try {
            fileWriter = new FileWriter(directoryPath+"/"+filename+".csv");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public boolean isDisabled() {
        return disabled;
    }

    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }

    public void close() {
        try {
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void update() {
        if (disabled) return;
        try {
            if (logTime) {
                long timeDifference = System.nanoTime()-startTime;
                line = timeDifference/1E9+","+line;
            }
            line = line.replace("[", "");
            line = line.replace("]", "");
            fileWriter.write(line+"\n");
            line = "";
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void addData(String data) {
        try
        {
            if (disabled) return;
            if (!line.equals("")) line += ",";
            line += data;
            telemetry.addData("Data state: ", "Added");
        }
        catch (NullPointerException e)
        {
            telemetry.addData("Data state: ","Not added");
        }
    }
    public void addData(Object data) {
        addData(data.toString());
    }
    public void addData(String[] data)
    {
        for (String d: data)
        {
            try
            {
                if (disabled) return;
                //noinspection StringConcatenation
                if (!line.equals("")) line += ",";
                line += d;
                if (!d.equals(data[data.length - 1]))
                {
                    line += ",";
                }
                telemetry.addData("Data state: ", "Added");
            }
            catch (NullPointerException e)
            {
                telemetry.addData("Data state: ","Not added");
            }
        }
    }

    public void addData(boolean... data)
    {
        addData(new String[]{Arrays.toString(data)});
    }
    public void addData(byte... data)
    {
        addData(new String[]{Arrays.toString(data)});
    }

    public void addData(double... data)
    {
        addData(new String[]{Arrays.toString(data)});
    }
    public void addData(int... data)
    {
        addData(new String[]{Arrays.toString(data)});
    }
    public void addData(char... data)
    {
        addData(new String[]{Arrays.toString(data)});
    }
    /*
    public void addData(Object... data)
    {
        addData(new String[]{Arrays.toString(data)});
    }
     */
    public void addData(float... data)
    {
        addData(new String[]{Arrays.toString(data)});
    }
    public void addData(long... data)
    {
        addData(new String[]{Arrays.toString(data)});
    }
    public void addData(short... data)
    {
        addData(new String[]{Arrays.toString(data)});
    }

}
