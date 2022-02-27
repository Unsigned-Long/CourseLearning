package com.example.juster.ui.home;

import android.annotation.SuppressLint;
import android.content.Context;
import android.graphics.Color;
import android.graphics.Typeface;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;

import com.example.juster.databinding.FragmentHomeBinding;
import com.github.mikephil.charting.charts.ScatterChart;
import com.github.mikephil.charting.components.XAxis;
import com.github.mikephil.charting.components.YAxis;
import com.github.mikephil.charting.data.Entry;
import com.github.mikephil.charting.data.ScatterData;
import com.github.mikephil.charting.data.ScatterDataSet;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.time.LocalTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;

enum SensorType {
    Accelerometer, Gyroscope, MagneticField, Orientation
}

public class HomeFragment extends Fragment implements SensorEventListener {

    private final int _HMAWinSize = 10;
    //0.371, 0.227 and 1
    private final float _model_param_a = 0.371f;
    private final float _model_param_b = 0.227f;
    private final float _EMA_param_fir = 0.7f;
    private final float _EMA_param_sed = 0.8f;
    private final float _EMA_param_trd = 0.9f;
    private float _model_param_c;
    private float _model_param_height;
    private FragmentHomeBinding binding;
    private Point2f _position;

    private TextView _tv_time_display;
    private TextView _tv_step_frequency;
    private TextView _tv_step_length;
    private TextView _tv_XPosition;
    private TextView _tv_YPosition;

    private SensorManager _sensorManager;
    private ArrayList<DataItem> _acceleration;
    private ArrayList<DataItem> _gyroscope;
    private ArrayList<DataItem> _magnetic;
    private ArrayList<DataItem> _orientation;

    private ArrayList<SensorDataItem> _sensorData;

    private LinkedList<DataItem> _processSequence;

    private float _stepFrequency;
    private float _stepLength;
    private long _lastTimeStamp;

    private boolean _newAcceleration;
    private boolean _newMagnetic;

    private ScatterChart _chart;
    private ArrayList<Point2f> _positionList;

    private float _lastYaw_fir;
    private float _lastYaw_sed;
    private float _lastYaw_trd;
    private boolean _lastYawHasInit;

    private boolean _isStatic;

    /**
     * weighted moving average method
     */
    public static LinkedList<Float> weightMoveAverage(LinkedList<Float> data, int winSize) {
        LinkedList<Float> result = new LinkedList<>();
        for (int i = 0; i != winSize - 1; ++i) {
            result.add(data.get(i));
        }
        for (int i = winSize - 1; i != data.size(); ++i) {
            float val = 0.0f;
            for (int j = i - winSize + 1; j != i + 1; ++j) {
                val += (winSize + j - i) * data.get(j);
            }
            val /= (double) winSize * (winSize + 1) / 2;
            result.add(val);
        }
        return result;
    }

    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container, Bundle savedInstanceState) {
        HomeViewModel homeViewModel =
                new ViewModelProvider(this).get(HomeViewModel.class);

        binding = FragmentHomeBinding.inflate(inflater, container, false);
        View root = binding.getRoot();

        // init the arraylists [acceleration, gyro]
        this.initVars();
        // find the ids for buttons and textview, and set the listeners
        this.registerControls();

        return root;
    }

    /**
     * init
     */
    public void initVars() {
        this._sensorManager = (SensorManager) getActivity().getSystemService(Context.SENSOR_SERVICE);

        this._acceleration = new ArrayList<>();
        this._gyroscope = new ArrayList<>();
        this._magnetic = new ArrayList<>();
        this._orientation = new ArrayList<>();

        this._processSequence = new LinkedList<>();

        this._sensorData = new ArrayList<>();

        _positionList = new ArrayList<>();

    }

    /**
     * register the controls in the activity
     */
    @SuppressLint("SetTextI18n")
    public void registerControls() {
        this._tv_time_display = binding.timeDisplay;
        this._tv_step_frequency = binding.textviewSf;
        this._tv_step_length = binding.textviewSl;
        this._tv_XPosition = binding.textviewXpos;
        this._tv_YPosition = binding.textviewYpos;

        this._chart = binding.chart;
        this._chart.setNoDataText("NO DATA FOR POSITION");
        this._chart.setNoDataTextColor(Color.RED);
        this._chart.setNoDataTextTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD_ITALIC));

        Button btn_clear = binding.btnClear;
        btn_clear.setOnClickListener(view -> {
            _acceleration.clear();
            _gyroscope.clear();
            _magnetic.clear();
            _orientation.clear();

            _tv_step_frequency.setText("HZ");
            _tv_step_length.setText("M");

            _tv_XPosition.setText("M");
            _tv_YPosition.setText("M");

            _tv_time_display.setText("");
            _positionList.clear();
            _chart.clear();

        });

        Button btn_save = binding.btnSave;
        btn_save.setOnClickListener(view -> {
            try {
                writeToFile(_acceleration, "acceleration.csv");
                writeToFile(_gyroscope, "gyroscope.csv");
                writeToFile(_magnetic, "magnetic.csv");
                writeToFile(_orientation, "orientation.csv");
                writeToFile(_sensorData, "sensor.csv");
                writeToFile(this._positionList, "positionData.csv");
                // save finished
                Toast.makeText(getActivity(), "SAVE FINISHED", Toast.LENGTH_SHORT).show();
                return;
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
            Toast.makeText(getActivity(), "SAVE FAILED", Toast.LENGTH_SHORT).show();
        });

        Button btn_start = binding.btnStart;
        btn_start.setOnClickListener(view -> {
            registerSensors();
            btn_clear.setEnabled(false);
            btn_save.setEnabled(false);

            // reset params
            _position = new Point2f(0.0f, 0.0f, 0L);
            _lastTimeStamp = 0L;

            _newAcceleration = false;
            _newMagnetic = false;
            _lastYawHasInit = false;
            _processSequence.clear();

            EditText et_height = binding.height;
            EditText et_param_c = binding.paramC;

            _model_param_height=Float.parseFloat(et_height.getText().toString());
            _model_param_c=Float.parseFloat(et_param_c.getText().toString());

        });

        Button btn_stop = binding.btnStop;
        btn_stop.setOnClickListener(view -> {
            unregisterSensors();
            btn_clear.setEnabled(true);
            btn_save.setEnabled(true);
        });

    }

    public <T> void writeToFile(ArrayList<T> data, String filename) throws FileNotFoundException {
        FileOutputStream stream = getActivity().openFileOutput(filename, Context.MODE_PRIVATE);
        PrintWriter writer = new PrintWriter(stream);
        for (T item : data) {
            writer.println(item.toString());
        }
        writer.close();
    }

    /**
     * register the sensor
     * TYPE_GYROSCOPE, TYPE_ACCELEROMETER
     */
    public void registerSensors() {
        this._sensorManager.registerListener(this, this._sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE), SensorManager.SENSOR_DELAY_UI);
        this._sensorManager.registerListener(this, this._sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_UI);
        this._sensorManager.registerListener(this, this._sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_UI);
    }

    /**
     * unregister the sensor
     */
    public void unregisterSensors() {
        this._sensorManager.unregisterListener(this);
    }

    @Override
    public void onDestroyView() {
        this.unregisterSensors();
        super.onDestroyView();
        binding = null;
    }

    @SuppressLint("SetTextI18n")
    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        // set the current time stamp
        this._tv_time_display.setText(LocalTime.now().toString() + " " + System.currentTimeMillis() + "(ms)");

        switch (sensorEvent.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER: {
                _newAcceleration = true;

                float ax = sensorEvent.values[0];
                float ay = sensorEvent.values[1];
                float az = sensorEvent.values[2];
                DataItem item = new DataItem(new float[]{ax, ay, az}, System.currentTimeMillis());
                this._acceleration.add(item);

                this._sensorData.add(new SensorDataItem(SensorType.Accelerometer, new float[]{ax, ay, az}, System.currentTimeMillis()));

            }
            break;
            case Sensor.TYPE_GYROSCOPE: {

                float gx = sensorEvent.values[0];
                float gy = sensorEvent.values[1];
                float gz = sensorEvent.values[2];

                this._gyroscope.add(new DataItem(new float[]{gx, gy, gz}, System.currentTimeMillis()));

                this._sensorData.add(new SensorDataItem(SensorType.Gyroscope, new float[]{gx, gy, gz}, System.currentTimeMillis()));

            }
            break;
            case Sensor.TYPE_MAGNETIC_FIELD: {

                _newMagnetic = true;

                float mx = sensorEvent.values[0];
                float my = sensorEvent.values[1];
                float mz = sensorEvent.values[2];

                this._magnetic.add(new DataItem(new float[]{mx, my, mz}, System.currentTimeMillis()));

                this._sensorData.add(new SensorDataItem(SensorType.MagneticField, new float[]{mx, my, mz}, System.currentTimeMillis()));

            }
            break;
            default:
                throw new IllegalStateException("Unexpected value: " + sensorEvent.sensor.getType());
        }

        // if the acceleration and gyro data is updated, then compute the orientation
        if (_newAcceleration && _newMagnetic) {

            // system
            float[] orientationSystem = this.estimateOrientationBySystem();

            float yaw = orientationSystem[0];
            float pitch = orientationSystem[1];
            float roll = orientationSystem[2];

            this._orientation.add(new DataItem(orientationSystem, System.currentTimeMillis()));

            this._sensorData.add(new SensorDataItem(SensorType.Orientation, new float[]{yaw, pitch, roll}, System.currentTimeMillis()));

            // average the yaw
            if (!_lastYawHasInit) {
                _lastYaw_fir = yaw;
                _lastYaw_sed = yaw;
                _lastYaw_trd = yaw;
                _lastYawHasInit = true;
            } else {
                if (Math.abs(_lastYaw_trd - yaw) > 5.0f) {
                    // from 3.14 to -3.14 or from -3.14 to 3.14
                    _lastYaw_fir = yaw;
                    _lastYaw_sed = yaw;
                    _lastYaw_trd = yaw;
                } else {
                    _lastYaw_fir = _lastYaw_fir * _EMA_param_fir + (1.0f - _EMA_param_fir) * yaw;
                    _lastYaw_sed = _lastYaw_sed * _EMA_param_sed + (1.0f - _EMA_param_sed) * _lastYaw_fir;
                    _lastYaw_trd = _lastYaw_trd * _EMA_param_trd + (1.0f - _EMA_param_trd) * _lastYaw_sed;
                }
            }
            // customer
            // float[] orientationCustomer = this.estimateOrientationByCustomer();

            // reset
            _newAcceleration = false;
            _newMagnetic = false;

            // estimate
            this.estimatePosition(_lastYaw_trd);
        }
    }

    /**
     * cast the vector from phone coordination to world coordination
     */
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

    @SuppressLint("SetTextI18n")
    public Boolean estimateStepFrequency() {
        // length limit
        if (this._processSequence.size() < 3 * this._HMAWinSize) {
            this._tv_step_frequency.setText("init");
            this._tv_step_length.setText("init");
            this._tv_XPosition.setText("init");
            this._tv_YPosition.setText("init");
            return false;
        }

        // compute the total acceleration
        LinkedList<Float> totalAcceleration = new LinkedList<>();
        for (DataItem item : this._processSequence) {
            totalAcceleration.add((float) Math.sqrt(item._values[0] * item._values[0] +
                    item._values[1] * item._values[1] +
                    item._values[2] * item._values[2]));
        }

        // compute the hull moving average
        LinkedList<Float> wma_T_2 = weightMoveAverage(totalAcceleration, this._HMAWinSize / 2);
        LinkedList<Float> wma_T = weightMoveAverage(totalAcceleration, this._HMAWinSize);
        LinkedList<Float> newSeq = new LinkedList<>();
        for (int i = 0; i != totalAcceleration.size(); ++i) {
            newSeq.add(wma_T_2.get(i) * 2 - wma_T.get(i));
        }
        LinkedList<Float> hma = weightMoveAverage(newSeq, (int) Math.sqrt(this._HMAWinSize));

        // statistic the average step frequency
        for (int i = hma.size() - 2; i != 0; --i) {
            // if is a peak
            if (hma.get(i - 1) < hma.get(i) && hma.get(i) > hma.get(i + 1)) {
                long curTimeStamp = this._processSequence.get(i)._timeStamp;

                if (this._lastTimeStamp == 0L) {
                    // first find the peak
                    this._lastTimeStamp = curTimeStamp;
                    return false;
                } else if (this._lastTimeStamp == curTimeStamp) {
                    // the same peak
                    this._processSequence.remove(0);
                    return false;
                } else {
                    // a new peak
                    if (hma.get(i) - 9.8f > 0.5f) {
                        this._stepFrequency = 1.0f / ((float) ((curTimeStamp - this._lastTimeStamp) / 1000.0));
                        _isStatic = false;
                    } else {
                        // the state is static
                        _isStatic = true;
                    }
                    this._lastTimeStamp = curTimeStamp;
                    this._processSequence.remove(0);
                    return true;
                }
            }
        }
        return false;
    }

    public void estimateStepLength() {
        if (_isStatic) {
            // the state is static
            this._stepLength = 0.0f;
            this._stepFrequency = 0.0f;
        } else {
            this._stepLength = (float) ((0.7f + _model_param_a * (_model_param_height - 1.75) + _model_param_b * (_stepFrequency - 1.79) * _model_param_height / 1.75) * _model_param_c);
        }
    }

    private float[] estimateOrientationBySystem() {
        float[] rotationMatrix = new float[9];
        SensorManager.getRotationMatrix(rotationMatrix, null,
                this._acceleration.get(this._acceleration.size() - 1)._values,
                this._magnetic.get(this._magnetic.size() - 1)._values);

        float[] orientationAngles = new float[3];
        SensorManager.getOrientation(rotationMatrix, orientationAngles);
        return orientationAngles;
    }

    private float[] estimateOrientationByCustomer() {
        float[] curAcceleration = this._acceleration.get(this._acceleration.size() - 1)._values;
        float[] curMagnetic = this._magnetic.get(this._magnetic.size() - 1)._values;

        float ax = curAcceleration[0];
        float ay = curAcceleration[1];
        float az = curAcceleration[2];

        float totalAcceleration = (float) Math.sqrt(ax * ax + ay * ay + az * az);
        float thetaRoll = (float) Math.atan2(-ax, az);
        float thetaPitch = (float) Math.asin(-ay / totalAcceleration);

        float[] vec = this.phone2world(curMagnetic, thetaRoll, thetaPitch, 0.0f);
        float thetaYaw = (float) Math.atan2(-vec[0], vec[1]);

        return new float[]{thetaYaw, thetaPitch, thetaRoll};
    }

    private void estimatePosition(float yaw) {
        this._processSequence.add(this._acceleration.get(this._acceleration.size() - 1));
        if (this.estimateStepFrequency()) {
            this.estimateStepLength();

            DecimalFormat ft = new DecimalFormat("0.000");

            this._position._x += this._stepLength * Math.sin(yaw);
            this._position._y += this._stepLength * Math.cos(yaw);

            this._positionList.add(new Point2f(this._position._x, this._position._y, System.currentTimeMillis()));

            if (!_isStatic) {
                this.drawPositionChart();
            }

            this._tv_XPosition.setText(ft.format(this._position._x));
            this._tv_YPosition.setText(ft.format(this._position._y));

            this._tv_step_frequency.setText(ft.format(this._stepFrequency));
            this._tv_step_length.setText(ft.format(this._stepLength));
        }
    }

    private void drawPositionChart() {
        List<Entry> data = new LinkedList<>();
        float XMin = 0.0f;
        float YMin = 0.0f;
        float XMax = 0.0f;
        float YMax = 0.0f;

        for (int i = 0; i != _positionList.size(); ++i) {
            float x = _positionList.get(i)._x;
            float y = _positionList.get(i)._y;
            data.add(new Entry(x, y));
            if (XMin > x) {
                XMin = x;
            }
            if (YMin > y) {
                YMin = y;
            }
            if (XMax < x) {
                XMax = x;
            }
            if (YMax < y) {
                YMax = y;
            }
        }
        if (XMax - XMin > YMax - YMin) {
            float delta = (XMax - XMin) - (YMax - YMin);
            YMax += delta / 2.0f;
            YMin -= delta / 2.0f;
        } else {
            float delta = (YMax - YMin) - (XMax - XMin);
            XMax += delta / 2.0f;
            XMin -= delta / 2.0f;
        }
        float delta = (XMax - XMin) * 0.2f;
        XMin -= delta;
        XMax += delta;
        YMin -= delta;
        YMax += delta;

        // this is necessary for the chart rendering
        Collections.sort(data, new Comparator<Entry>() {
            @Override
            public int compare(Entry o1, Entry o2) {
                if (o1.getX() > o2.getX()) {
                    return 1;
                }
                if (o1.getX() < o2.getX()) {
                    return -1;
                }
                return 0;
            }
        });

        ScatterDataSet path = new ScatterDataSet(data, "PATH");
        path.setColor(Color.parseColor("#215E8F"));
        path.setDrawValues(false);
        path.setScatterShape(ScatterChart.ScatterShape.CROSS);

        XAxis xAxis = _chart.getXAxis();
        xAxis.setAxisMinimum(XMin);
        xAxis.setAxisMaximum(XMax);
        xAxis.enableGridDashedLine(5, 4, 0);
        xAxis.setPosition(XAxis.XAxisPosition.BOTTOM);
        xAxis.setTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD));
        xAxis.setDrawAxisLine(false);
        xAxis.setLabelCount(10);

        YAxis leftAxis = _chart.getAxisLeft();
        leftAxis.setAxisMinimum(YMin);
        leftAxis.setAxisMaximum(YMax);
        leftAxis.enableGridDashedLine(5, 4, 0);
        leftAxis.setTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD));
        leftAxis.setLabelCount(7);
        leftAxis.setDrawAxisLine(false);
        leftAxis.setLabelCount(10);

        YAxis rightAxis = _chart.getAxisRight();
        rightAxis.setAxisMinimum(YMin);
        rightAxis.setAxisMaximum(YMax);
        rightAxis.enableGridDashedLine(5, 4, 0);
        rightAxis.setTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD));
        rightAxis.setLabelCount(7);
        rightAxis.setDrawAxisLine(false);
        rightAxis.setLabelCount(10);


        _chart.animateXY(0, 0);
        _chart.getDescription().setTextSize(12);
        _chart.getDescription().setTextColor(Color.parseColor("#DC4949"));
        _chart.getDescription().setTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD_ITALIC));
        _chart.getLegend().setTypeface(Typeface.create("Ubuntu Mono", Typeface.BOLD));
        _chart.setDrawMarkers(true);
        _chart.getDescription().setText("Unit(M)");

        _chart.setData(new ScatterData(path));
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    /**
     * structures to organize the data [acceleration orientation gyroscope magnetic]
     */
    static class Point2f {
        public float _x;
        public float _y;
        public long _timeStamp;

        Point2f(float x, float y, long timeStamp) {
            this._x = x;
            this._y = y;
            this._timeStamp = timeStamp;
        }

        @Override
        public String toString() {
            return this._timeStamp + "," + this._x + "," + this._y;
        }
    }

    static class DataItem {
        float[] _values;
        /**
         * unit: ms
         */
        long _timeStamp;

        DataItem(float[] values, long timeStamp) {
            this._values = values;
            this._timeStamp = timeStamp;
        }

        @Override
        public String toString() {
            return _timeStamp + "," + _values[0] + "," + _values[1] + "," + _values[2];
        }
    }

    static class SensorDataItem extends DataItem {
        private final SensorType _type;

        SensorDataItem(SensorType type, float[] values, long timeStamp) {
            super(values, timeStamp);
            this._type = type;
        }

        @NonNull
        @Override
        public String toString() {
            return _type.toString() + "," + super.toString();
        }
    }
}