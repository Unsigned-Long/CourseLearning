package com.example.juster.ui.dashboard;

import android.content.Context;
import android.graphics.Color;
import android.graphics.Typeface;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;

import com.example.juster.databinding.FragmentDashboardBinding;
import com.github.mikephil.charting.charts.LineChart;
import com.github.mikephil.charting.components.XAxis;
import com.github.mikephil.charting.components.YAxis;
import com.github.mikephil.charting.data.Entry;
import com.github.mikephil.charting.data.LineData;
import com.github.mikephil.charting.data.LineDataSet;

import java.text.DecimalFormat;
import java.util.LinkedList;
import java.util.List;

public class DashboardFragment extends Fragment implements SensorEventListener {

    /**
     * max length for line chart drawing
     */
    private final Integer _MAX_LIST_ITEMS = 20;
    private FragmentDashboardBinding binding;
    /**
     * the manager of the sensors
     */
    private SensorManager _sensorManager;
    /**
     * main sensor data
     */
    private LinkedList<DataItem> _acceleration;
    private LineChart _acceLineChart;
    private LinkedList<DataItem> _gyroscope;
    private LineChart _gyroLineChart;
    private LinkedList<DataItem> _magnetic;
    private LineChart _magnaLineChart;
    private LinkedList<DataItem> _orientation;
    private LineChart _orientLineChart;
    private LinkedList<DataItem> _orient_customer;
    private LineChart _orientLineChartCust;
    /**
     * decide the cauculation of oriention using magnetic and accelerometer
     */
    private boolean _newAcceleration;
    private boolean _newMagnetic;

    /**
     * display the info
     */
    private TextView _tv_ax;
    private TextView _tv_ay;
    private TextView _tv_az;
    private TextView _tv_gx;
    private TextView _tv_gy;
    private TextView _tv_gz;
    private TextView _tv_mx;
    private TextView _tv_my;
    private TextView _tv_mz;
    private TextView _tv_yaw;
    private TextView _tv_pitch;
    private TextView _tv_roll;
    private TextView _tv_ox_customer;
    private TextView _tv_oy_customer;
    private TextView _tv_oz_customer;

    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container, Bundle savedInstanceState) {
        DashboardViewModel dashboardViewModel =
                new ViewModelProvider(this).get(DashboardViewModel.class);

        binding = FragmentDashboardBinding.inflate(inflater, container, false);
        View root = binding.getRoot();

        // init the arraylists [acce, gyro]
        this.initVars();
        // find the ids for buttons and textview, and set the listeners
        this.registerControls();
        // start the sensors
        this.registerSensor();

        return root;
    }

    /**
     * init
     */
    public void initVars() {
        this._sensorManager = (SensorManager) getActivity().getSystemService(Context.SENSOR_SERVICE);

        this._acceleration = new LinkedList<>();
        this._gyroscope = new LinkedList<>();
        this._magnetic = new LinkedList<>();
        this._orientation = new LinkedList<>();
        this._orient_customer = new LinkedList<>();

        this._newAcceleration = false;
        this._newMagnetic = false;
    }

    /**
     * register the controls in the activity
     */
    public void registerControls() {
        this._tv_ax = binding.acceX;
        this._tv_ay = binding.acceY;
        this._tv_az = binding.acceZ;

        this._tv_gx = binding.gyroX;
        this._tv_gy = binding.gyroY;
        this._tv_gz = binding.gyroZ;

        this._tv_mx = binding.magnX;
        this._tv_my = binding.magnY;
        this._tv_mz = binding.magnZ;

        this._tv_yaw = binding.orienX;
        this._tv_pitch = binding.orienY;
        this._tv_roll = binding.orienZ;

        this._tv_ox_customer = binding.orienXCust;
        this._tv_oy_customer = binding.orienYCust;
        this._tv_oz_customer = binding.orienZCust;

        this._acceLineChart = binding.linechartAcce;
        this._acceLineChart.setNoDataText("NO DATA FOR ACCELERATION");
        this._acceLineChart.setNoDataTextColor(Color.RED);
        this._acceLineChart.setNoDataTextTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD_ITALIC));
        this._acceLineChart.getDescription().setText("Unit(M/S^2)");

        this._gyroLineChart = binding.linechartGyro;
        this._gyroLineChart.setNoDataText("NO DATA FOR GYROSCOPE");
        this._gyroLineChart.setNoDataTextColor(Color.RED);
        this._gyroLineChart.setNoDataTextTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD_ITALIC));
        this._gyroLineChart.getDescription().setText("Unit(RAD/S)");

        this._magnaLineChart = binding.linechartMagn;
        this._magnaLineChart.setNoDataText("NO DATA FOR MAGNETIC FIELD");
        this._magnaLineChart.setNoDataTextColor(Color.RED);
        this._magnaLineChart.setNoDataTextTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD_ITALIC));
        this._magnaLineChart.getDescription().setText("Unit(UT)");

        this._orientLineChart = binding.linechartOrien;
        this._orientLineChart.setNoDataText("NO DATA FOR ORIENTATION");
        this._orientLineChart.setNoDataTextColor(Color.RED);
        this._orientLineChart.setNoDataTextTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD_ITALIC));
        this._orientLineChart.getDescription().setText("Unit(RAD)");

        this._orientLineChartCust = binding.linechartOrienCust;
        this._orientLineChartCust.setNoDataText("NO DATA FOR ORIENTATION");
        this._orientLineChartCust.setNoDataTextColor(Color.RED);
        this._orientLineChartCust.setNoDataTextTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD_ITALIC));
        this._orientLineChartCust.getDescription().setText("Unit(RAD)");
    }

    /**
     * register the sensor
     * TYPE_GYROSCOPE, TYPE_ACCELEROMETER
     */
    public void registerSensor() {
        this._sensorManager.registerListener(this, this._sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE), SensorManager.SENSOR_DELAY_NORMAL);
        this._sensorManager.registerListener(this, this._sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_NORMAL);
        this._sensorManager.registerListener(this, this._sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_NORMAL);
    }

    /**
     * unregister the sensor
     */
    public void unregisterSensor() {
        this._sensorManager.unregisterListener(this);
    }

    public float[] phone2world(float[] vector, float roll, float pitch, float yaw) {

        float x = vector[0];
        float y = vector[1];
        float z = vector[2];

        float x_after_roll = (float) (x * Math.cos(roll) + z * Math.sin(roll));
        float z_after_roll = (float) (x * (-Math.sin(roll)) + z * Math.cos(roll));

        float y_after_pitch = (float) (y * Math.cos(pitch) + z_after_roll * Math.sin(pitch));
        float z_after_pitch = (float) (y * (-Math.sin(pitch)) + z_after_roll * Math.cos(pitch));

        float x_after_yaw = (float) (x_after_roll * Math.cos(yaw) + y_after_pitch * Math.sin(yaw));
        float y_after_yaw = (float) (x_after_roll * (-Math.sin(yaw)) + y_after_pitch * Math.cos(yaw));

        return new float[]{x_after_yaw, y_after_yaw, z_after_pitch};
    }

    @Override
    public void onDestroyView() {
        this.unregisterSensor();
        super.onDestroyView();
        binding = null;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        DecimalFormat ft = new DecimalFormat("+00.0000;-00.0000");

        switch (event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER: {

                _newAcceleration = true;

                float ax = event.values[0];
                float ay = event.values[1];
                float az = event.values[2];

                this._acceleration.add(new DataItem(new float[]{ax, ay, az}));
                if (this._acceleration.size() > this._MAX_LIST_ITEMS) {
                    this._acceleration.removeFirst();
                }

                // set content for the text views
                this._tv_ax.setText(ft.format(ax));
                this._tv_ay.setText(ft.format(ay));
                this._tv_az.setText(ft.format(az));

                // draw
                this.drawLineChart(this._acceLineChart, this._acceleration, "A(x)", "A(y)", "A(z)");
            }
            break;
            case Sensor.TYPE_GYROSCOPE: {

                float gx = event.values[0];
                float gy = event.values[1];
                float gz = event.values[2];

                this._gyroscope.add(new DataItem(new float[]{gx, gy, gz}));
                if (this._gyroscope.size() > this._MAX_LIST_ITEMS) {
                    this._gyroscope.removeFirst();
                }

                // set content for the text views
                this._tv_gx.setText(ft.format(gx));
                this._tv_gy.setText(ft.format(gy));
                this._tv_gz.setText(ft.format(gz));

                // draw
                this.drawLineChart(this._gyroLineChart, this._gyroscope, "G(x)", "G(y)", "G(z)");
            }
            break;
            case Sensor.TYPE_MAGNETIC_FIELD: {

                _newMagnetic = true;

                float mx = event.values[0];
                float my = event.values[1];
                float mz = event.values[2];

                this._magnetic.add(new DataItem(new float[]{mx, my, mz}));
                if (this._magnetic.size() > this._MAX_LIST_ITEMS) {
                    this._magnetic.removeFirst();
                }

                // set content for the text views
                this._tv_mx.setText(ft.format(mx));
                this._tv_my.setText(ft.format(my));
                this._tv_mz.setText(ft.format(mz));

                // draw
                this.drawLineChart(this._magnaLineChart, this._magnetic, "M(x)", "M(y)", "M(z)");
            }
            break;
            default:
                throw new IllegalStateException("Unexpected value: " + event.sensor.getType());
        }

        // if the acceleration and gyroscope data is updated, then compute the orientation
        if (_newAcceleration && _newMagnetic) {
            float[] curAcce = this._acceleration.get(this._acceleration.size() - 1)._values;
            float[] curMagne = this._magnetic.get(this._magnetic.size() - 1)._values;

            float ax = curAcce[0];
            float ay = curAcce[1];
            float az = curAcce[2];

            // system
            float[] rotationMatrix = new float[9];
            SensorManager.getRotationMatrix(rotationMatrix, null, curAcce, curMagne);

            float[] orientationAngles = new float[3];
            SensorManager.getOrientation(rotationMatrix, orientationAngles);

            float yaw = orientationAngles[0];
            float pitch = orientationAngles[1];
            float roll = orientationAngles[2];

            this._orientation.add(new DataItem(orientationAngles));
            if (this._orientation.size() > this._MAX_LIST_ITEMS) {
                this._orientation.removeFirst();
            }

            // set content for the text views
            this._tv_yaw.setText(ft.format(yaw));
            this._tv_pitch.setText(ft.format(pitch));
            this._tv_roll.setText(ft.format(roll));

            // draw
            this.drawLineChart(this._orientLineChart, this._orientation, "Yaw", "Pitch", "Roll");

            // customer
            float a = (float) Math.sqrt(ax * ax + ay * ay + az * az);
            float thetaRoll = (float) Math.atan2(-ax, az);
            float thetaPitch = (float) Math.asin(-ay / a);

            float[] vec = this.phone2world(curMagne, thetaRoll, thetaPitch, 0.0f);
            float thetaYaw = (float) Math.atan2(-vec[0], vec[1]);

            this._orient_customer.add(new DataItem(new float[]{thetaYaw, thetaPitch, thetaRoll}));
            if (this._orient_customer.size() > this._MAX_LIST_ITEMS) {
                this._orient_customer.removeFirst();
            }

            // set content for the text views
            this._tv_ox_customer.setText(ft.format(thetaYaw));
            this._tv_oy_customer.setText(ft.format(thetaPitch));
            this._tv_oz_customer.setText(ft.format(thetaRoll));

            // draw
            this.drawLineChart(this._orientLineChartCust, this._orient_customer, "Yaw", "Pitch", "Roll");
            // reset
            _newAcceleration = false;
            _newMagnetic = false;
        }
    }

    public <T extends DataItem>
    void drawLineChart(LineChart chart, LinkedList<T> data, String xName, String yName, String zName) {
        List<Entry> lsX = new LinkedList<>();
        List<Entry> lsY = new LinkedList<>();
        List<Entry> lsZ = new LinkedList<>();

        int count = 0;
        float maxAbs = 0.0f;

        for (T elem : data) {

            lsX.add(new Entry(count, elem._values[0]));
            lsY.add(new Entry(count, elem._values[1]));
            lsZ.add(new Entry(count, elem._values[2]));

            if (Math.abs(elem._values[0]) > maxAbs) {
                maxAbs = Math.abs(elem._values[0]);
            }
            if (Math.abs(elem._values[1]) > maxAbs) {
                maxAbs = Math.abs(elem._values[1]);
            }
            if (Math.abs(elem._values[2]) > maxAbs) {
                maxAbs = Math.abs(elem._values[2]);
            }

            ++count;
        }

        maxAbs *= 1.2f;


        LineDataSet dsX = new LineDataSet(lsX, xName);
        dsX.setLineWidth(1);
        dsX.setColor(Color.parseColor("#A62121"));
        dsX.setDrawValues(false);
        dsX.setCircleColor(Color.parseColor("#A62121"));

        LineDataSet dsY = new LineDataSet(lsY, yName);
        dsY.setLineWidth(1);
        dsY.setColor(Color.parseColor("#8BC34A"));
        dsY.setDrawValues(false);
        dsY.setCircleColor(Color.parseColor("#8BC34A"));

        LineDataSet dsZ = new LineDataSet(lsZ, zName);
        dsZ.setLineWidth(1);
        dsZ.setColor(Color.parseColor("#FF9800"));
        dsZ.setDrawValues(false);
        dsZ.setCircleColor(Color.parseColor("#FF9800"));

        XAxis xAxis = chart.getXAxis();
        xAxis.setAxisMinimum(0);
        xAxis.setAxisMaximum(this._MAX_LIST_ITEMS);
        xAxis.enableGridDashedLine(5, 4, 0);
        xAxis.setPosition(XAxis.XAxisPosition.BOTTOM);
        xAxis.setTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD));
        xAxis.setDrawAxisLine(false);

        YAxis leftAxis = chart.getAxisLeft();
        leftAxis.setAxisMinimum(-maxAbs);
        leftAxis.setAxisMaximum(maxAbs);
        leftAxis.enableGridDashedLine(5, 4, 0);
        leftAxis.setTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD));
        leftAxis.setLabelCount(7);
        leftAxis.setDrawAxisLine(false);

        YAxis rightAxis = chart.getAxisRight();
        rightAxis.setAxisMinimum(-maxAbs);
        rightAxis.setAxisMaximum(maxAbs);
        rightAxis.enableGridDashedLine(5, 4, 0);
        rightAxis.setTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD));
        rightAxis.setLabelCount(7);
        rightAxis.setDrawAxisLine(false);


        chart.animateXY(0,0);
        chart.getDescription().setTextSize(12);
        chart.getDescription().setTextColor(Color.parseColor("#DC4949"));
        chart.getDescription().setTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD_ITALIC));
        chart.getLegend().setTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD));
        chart.setDrawMarkers(true);
        LineData ld = new LineData();
        ld.addDataSet(dsX);
        ld.addDataSet(dsY);
        ld.addDataSet(dsZ);
        chart.setData(ld);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    /**
     * structures to organize the data
     */
    static class DataItem {
        float[] _values;

        DataItem(float[] values) {
            this._values = values;
        }
    }

}