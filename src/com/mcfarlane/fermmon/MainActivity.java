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

public class MainActivity extends Activity
{
    // Local Bluetooth adapter
    private BluetoothAdapter mBluetoothAdapter = null;

    // Message types sent from the BluetoothChatService Handler
    public static final int MESSAGE_STATE_CHANGE = 1;
    public static final int MESSAGE_READ = 2;
    public static final int MESSAGE_WRITE = 3;
    public static final int MESSAGE_DEVICE_NAME = 4;
    public static final int MESSAGE_TOAST = 5;

    // Key names received from the BluetoothChatService Handler
    public static final String DEVICE_NAME = "device_name";
    public static final String TOAST = "toast";

    // Intent request codes
    private static final int REQUEST_CONNECT_DEVICE_SECURE = 1;
    private static final int REQUEST_CONNECT_DEVICE_INSECURE = 2;
    private static final int REQUEST_ENABLE_BT = 3;

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState )
    {
        int REQUEST_ENABLE_BT = 1;

        super.onCreate(savedInstanceState);
        // Get local Bluetooth adapter
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        // If the adapter is null, then Bluetooth is not supported
        if (mBluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth is not available", Toast.LENGTH_LONG).show();
            finish();
            return;
        }

        if (!mBluetoothAdapter.isEnabled()) {
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
        }

        Set<BluetoothDevice> pairedDevices = mBluetoothAdapter.getBondedDevices();
        // If there are paired devices
        if (pairedDevices.size() > 0) {
            // Loop through paired devices
            for (BluetoothDevice device : pairedDevices) {
                Toast.makeText(this, "Bluetooth is available on " + device.getName() + "\n" + device.getAddress(), Toast.LENGTH_LONG).show();
            }
        }

//        Toast.makeText(this, "Bluetooth is available", Toast.LENGTH_LONG).show();

        setContentView(R.layout.main);
    }
}
