package com.kareem.ballbot.serial

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import java.util.concurrent.Executors

class UsbSerialManager(
    private val context: Context,
    private val onStatus: (String) -> Unit
) {
    private val actionUsbPermission = "com.kareem.ballbot.USB_PERMISSION"
    private val usbManager = context.getSystemService(Context.USB_SERVICE) as UsbManager
    private val executor = Executors.newSingleThreadExecutor()
    private var port: UsbSerialPort? = null

    private val permissionReceiver = object : BroadcastReceiver() {
        override fun onReceive(ctx: Context?, intent: Intent?) {
            if (intent?.action != actionUsbPermission) return
            val granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)
            val device = intent.getParcelableExtra<UsbDevice>(UsbManager.EXTRA_DEVICE)
            if (granted && device != null) openDevice(device) else onStatus("USB permission denied")
        }
    }

    init {
        context.registerReceiver(permissionReceiver, IntentFilter(actionUsbPermission), Context.RECEIVER_NOT_EXPORTED)
    }

    fun connectFirstAvailable() {
        val drivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager)
        if (drivers.isEmpty()) {
            onStatus("No USB serial device found")
            return
        }
        val device = drivers.first().device
        if (!usbManager.hasPermission(device)) {
            val pendingIntent = PendingIntent.getBroadcast(
                context,
                0,
                Intent(actionUsbPermission),
                PendingIntent.FLAG_IMMUTABLE
            )
            usbManager.requestPermission(device, pendingIntent)
            onStatus("Requesting USB permission")
            return
        }
        openDevice(device)
    }

    private fun openDevice(device: UsbDevice) {
        val driver = UsbSerialProber.getDefaultProber().probeDevice(device)
        if (driver == null) {
            onStatus("Unsupported USB serial device")
            return
        }
        val connection = usbManager.openDevice(device)
        if (connection == null) {
            onStatus("Failed to open USB device")
            return
        }
        port = driver.ports.first().apply {
            open(connection)
            setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
            dtr = true
            rts = true
        }
        onStatus("USB connected")
    }

    fun send(data: String) {
        val bytes = data.toByteArray()
        executor.execute {
            try {
                port?.write(bytes, 100)
                onStatus("Sent: ${data.trim()}")
            } catch (e: Exception) {
                onStatus("USB write error: ${e.message}")
            }
        }
    }

    fun close() {
        try {
            context.unregisterReceiver(permissionReceiver)
        } catch (_: Exception) {
        }
        port?.close()
        executor.shutdownNow()
    }
}
