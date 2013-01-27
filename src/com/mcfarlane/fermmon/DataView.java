package com.mcfarlane.fermmon;

import java.util.Set;
import java.util.Arrays;

import android.app.Activity;

import android.graphics.Color;
import android.os.Bundle;
import android.content.Intent;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.widget.Toast;

import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;

import com.androidplot.xy.SimpleXYSeries;
import com.androidplot.series.XYSeries;
import com.androidplot.xy.*;


public class DataView extends Activity implements OnTouchListener 
{
    private XYPlot mySimpleXYPlot;

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState )
    {
        super.onCreate(savedInstanceState);

        // initialize our XYPlot reference:
        mySimpleXYPlot = (XYPlot) findViewById(R.id.mySimpleXYPlot);
	mySimpleXYPlot.setOnTouchListener(this);

        // Create a couple arrays of y-values to plot:
        Number[] series1Numbers = {1, 8, 5, 2, 7, 4};
        Number[] series2Numbers = {4, 6, 3, 8, 2, 10};

        // Turn the above arrays into XYSeries':
        XYSeries series1 = new SimpleXYSeries(
                Arrays.asList(series1Numbers),          // SimpleXYSeries takes a List so turn our array into a List
                SimpleXYSeries.ArrayFormat.Y_VALS_ONLY, // Y_VALS_ONLY means use the element index as the x value
                "Series1");                             // Set the display title of the series

        // same as above
        XYSeries series2 = new SimpleXYSeries(Arrays.asList(series2Numbers), SimpleXYSeries.ArrayFormat.Y_VALS_ONLY, "Series2");

        // Create a formatter to use for drawing a series using LineAndPointRenderer:
        LineAndPointFormatter series1Format = new LineAndPointFormatter(
                Color.rgb(0, 200, 0),                   // line color
                Color.rgb(0, 100, 0),                   // point color
                null);                                  // fill color (none)

        // add a new series' to the xyplot:
        mySimpleXYPlot.addSeries(series1, series1Format);

        // same as above:
        mySimpleXYPlot.addSeries(series2,
                new LineAndPointFormatter(Color.rgb(0, 0, 200), Color.rgb(0, 0, 100), null));

        // reduce the number of range labels
        mySimpleXYPlot.setTicksPerRangeLabel(3);

        // by default, AndroidPlot displays developer guides to aid in laying out your plot.
        // To get rid of them call disableAllMarkup():
        mySimpleXYPlot.disableAllMarkup();
    }


    @Override
    public boolean onTouch(View arg0, MotionEvent event) {
        switch (event.getAction() & MotionEvent.ACTION_MASK) {
            case MotionEvent.ACTION_DOWN: // Start gesture
               //firstFinger = new PointF(event.getX(), event.getY());
               //mode = ONE_FINGER_DRAG;
               Toast.makeText(this, "Touched", Toast.LENGTH_LONG).show();
               break;
        }
        return true;
    }
}
