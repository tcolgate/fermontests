package com.mcfarlane.fermmon;

import java.util.Set;
import java.util.Arrays;

import android.app.Activity;

import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.content.Intent;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.widget.TextView;
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

    private TextView txt;
    private Handler mTextHandler = new Handler() {
      @Override
      public void handleMessage(Message msg) {
        Bundle b = msg.getData();
        String val = b.getString("My Key");
        //txt.setText(txt.getText()+"Item " + val + System.getProperty("line.separator"));
      }
    };


    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState )
    {
        int REQUEST_ENABLE_BT = 1;

        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        txt=(TextView)findViewById(R.id.myTextView);

        // Get local Bluetooth adapter
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        // If the adapter is null, then Bluetooth is not supported
        if (mBluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth is not available", Toast.LENGTH_LONG).show();
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

    }

    @Override
    public void onStart()
    {
      super.onStart();

      Toast.makeText(this, "onStart()", Toast.LENGTH_LONG).show();

      Thread worker = new Thread(new Runnable(){
        public int i = 0;

        @Override
        public void run() {
          try {
            while(true) {
              Thread.sleep(1000);
              i++;
              Message m = new Message();
              Bundle b = new Bundle();
              b.putString("My Key", "Value: " + String.valueOf(i));
              m.setData(b);
              // send message to the handler with the current message handler          
              mTextHandler.sendMessage(m);
            }
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      });

      worker.start();
    }

    /** Called when the user clicks the Send button */
    public void viewData(View view) {
      // Do something in response to button

       Intent intent = new Intent(this, DataView.class);
       startActivity(intent);
    }
}
