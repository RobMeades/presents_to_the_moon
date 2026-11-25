/*
 * Copyright 2026 Rob Meades
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* This written by Google Gemini from my prompts. */

package com.example.presents_to_the_moon

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Context
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.SystemClock
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.animation.AnimatedVisibility
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.BasicTextField
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.focus.onFocusChanged
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalFocusManager
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import kotlinx.coroutines.*
import java.text.SimpleDateFormat
import java.util.*
import kotlin.math.abs

class MainActivity : ComponentActivity() {

    private val bluetoothAdapter: BluetoothAdapter? by lazy {
        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothManager.adapter
    }

    // Mutable states exposed to UI (initialized to null / blank until read from device)
    var connectionStatus = mutableStateOf("Disconnected")
    var deviceStateName = mutableStateOf("UNKNOWN")
    var currentHeightMm = mutableStateOf<String?>(null)
    var t0EpochSec = mutableStateOf<Long?>(null)

    // Config values (Null until read from connected device)
    var emergencyStop = mutableStateOf<Boolean?>(null)
    var offPeakEnable = mutableStateOf<Boolean?>(null)
    var offPeakStartSec = mutableStateOf<Int?>(null)
    var offPeakDurationSec = mutableStateOf<Int?>(null)
    var offPeakLaunchIntervalSec = mutableStateOf<String?>(null)
    var peakEnable = mutableStateOf<Boolean?>(null)
    var peakStartSec = mutableStateOf<Int?>(null)
    var peakDurationSec = mutableStateOf<Int?>(null)
    var peakLaunchIntervalSec = mutableStateOf<String?>(null)
    var heightMaxMm = mutableStateOf<String?>(null)
    var speedMmSec = mutableStateOf<String?>(null)
    var delugeTrack = mutableStateOf<String?>(null)
    var launchingTrack = mutableStateOf<String?>(null)
    var danglingTrack = mutableStateOf<String?>(null)
    var resettingTrack = mutableStateOf<String?>(null)
    var muteEnable = mutableStateOf<Boolean?>(null)

    private var bluetoothGatt: BluetoothGatt? = null
    private val TARGET_DEVICE_NAME = "presents_to_the_moon"

    private var pollingJob: Job? = null
    private val scope = CoroutineScope(Dispatchers.Main + SupervisorJob())

    // UUID mappings from presents_to_the_moon firmware characteristic table
    private val SERVICE_UUID = UUID.fromString("0000FFE0-0000-1000-8000-00805F9B34FB")
    private val CHAR_UUIDS = mapOf(
        "LAUNCH_NOW" to UUID.fromString("0000FFE1-0000-1000-8000-00805F9B34FB"),
        "UP" to UUID.fromString("0000FFE2-0000-1000-8000-00805F9B34FB"),
        "DOWN" to UUID.fromString("0000FFE3-0000-1000-8000-00805F9B34FB"),
        "THIS_IS_GROUND_LEVEL" to UUID.fromString("0000FFE4-0000-1000-8000-00805F9B34FB"),
        "THIS_IS_HEIGHT_MAX" to UUID.fromString("0000FFE5-0000-1000-8000-00805F9B34FB"),
        "RESET_TO_DEFAULTS" to UUID.fromString("0000FFE6-0000-1000-8000-00805F9B34FB"),
        "EMERGENCY_STOP_ENABLE" to UUID.fromString("0000FFE7-0000-1000-8000-00805F9B34FB"),
        "PLAY_TRACK" to UUID.fromString("0000FFE8-0000-1000-8000-00805F9B34FB"),
        "OFF_PEAK" to UUID.fromString("0000FFE9-0000-1000-8000-00805F9B34FB"),
        "OFF_PEAK_START_SECONDS_SINCE_MIDNIGHT" to UUID.fromString("0000FFEA-0000-1000-8000-00805F9B34FB"),
        "OFF_PEAK_DURATION_SECONDS" to UUID.fromString("0000FFEB-0000-1000-8000-00805F9B34FB"),
        "OFF_PEAK_LAUNCH_INTERVAL_SECONDS" to UUID.fromString("0000FFEC-0000-1000-8000-00805F9B34FB"),
        "PEAK" to UUID.fromString("0000FFED-0000-1000-8000-00805F9B34FB"),
        "PEAK_START_SECONDS_SINCE_MIDNIGHT" to UUID.fromString("0000FFEE-0000-1000-8000-00805F9B34FB"),
        "PEAK_DURATION_SECONDS" to UUID.fromString("0000FFEF-0000-1000-8000-00805F9B34FB"),
        "PEAK_LAUNCH_INTERVAL_SECONDS" to UUID.fromString("0000FFF0-0000-1000-8000-00805F9B34FB"),
        "HEIGHT_MAX_MM" to UUID.fromString("0000FFF1-0000-1000-8000-00805F9B34FB"),
        "SPEED_MM_PER_SECOND" to UUID.fromString("0000FFF2-0000-1000-8000-00805F9B34FB"),
        "COMMAND_TRACK_NUMBER_DELUGE" to UUID.fromString("0000FFF3-0000-1000-8000-00805F9B34FB"),
        "COMMAND_TRACK_NUMBER_LAUNCHING" to UUID.fromString("0000FFF4-0000-1000-8000-00805F9B34FB"),
        "COMMAND_TRACK_NUMBER_DANGLING" to UUID.fromString("0000FFF5-0000-1000-8000-00805F9B34FB"),
        "COMMAND_TRACK_NUMBER_RESETTING" to UUID.fromString("0000FFF6-0000-1000-8000-00805F9B34FB"),
        "MUTE" to UUID.fromString("0000FFF7-0000-1000-8000-00805F9B34FB"),
        "STATE" to UUID.fromString("0000FFF8-0000-1000-8000-00805F9B34FB"),
        "HEIGHT_CURRENT_MM" to UUID.fromString("0000FFF9-0000-1000-8000-00805F9B34FB"),
        "T0" to UUID.fromString("0000FFFA-0000-1000-8000-00805F9B34FB")
    )

    // Unified list controlling the exact order characteristics are read/updated
    private val ALL_CHARACTERISTIC_KEYS = listOf(
        "EMERGENCY_STOP_ENABLE",
        "STATE",
        "HEIGHT_CURRENT_MM",
        "T0",
        "OFF_PEAK",
        "OFF_PEAK_START_SECONDS_SINCE_MIDNIGHT",
        "OFF_PEAK_DURATION_SECONDS",
        "OFF_PEAK_LAUNCH_INTERVAL_SECONDS",
        "PEAK",
        "PEAK_START_SECONDS_SINCE_MIDNIGHT",
        "PEAK_DURATION_SECONDS",
        "PEAK_LAUNCH_INTERVAL_SECONDS",
        "MUTE",
        "SPEED_MM_PER_SECOND",
        "HEIGHT_MAX_MM",
        "COMMAND_TRACK_NUMBER_DELUGE",
        "COMMAND_TRACK_NUMBER_LAUNCHING",
        "COMMAND_TRACK_NUMBER_DANGLING",
        "COMMAND_TRACK_NUMBER_RESETTING"
    )

    private val launcherStates = listOf(
        "NULL", "HEIGHT_UNKNOWN", "READY", "MOVE_UP", "MOVE_DOWN",
        "RUN", "RUNNING_DELUGE", "RUNNING_IGNITION", "RUNNING_LAUNCHING",
        "RUNNING_DANGLING", "RUNNING_RESETTING", "HALT"
    )

    // Queue for sequential GATT reads
    private val readQueue = LinkedList<UUID>()
    private var isReading = false
    private var activeReadUuid: UUID? = null
    private var lastCompletedReadUuid: UUID? = null
    private var lastCompletedReadAtMs: Long = 0L

    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        if (permissions.all { it.value }) {
            startBleScan()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            MaterialTheme {
                Surface(modifier = Modifier.fillMaxSize(), color = MaterialTheme.colorScheme.background) {
                    MainScreen(
                        connectionStatus = connectionStatus.value,
                        deviceState = deviceStateName.value,
                        currentHeight = currentHeightMm.value,
                        t0EpochSec = t0EpochSec.value,
                        emergencyStop = emergencyStop.value,
                        offPeakEnable = offPeakEnable.value,
                        offPeakStartSec = offPeakStartSec.value,
                        offPeakDurationSec = offPeakDurationSec.value,
                        offPeakLaunchIntervalSec = offPeakLaunchIntervalSec.value,
                        peakEnable = peakEnable.value,
                        peakStartSec = peakStartSec.value,
                        peakDurationSec = peakDurationSec.value,
                        peakLaunchIntervalSec = peakLaunchIntervalSec.value,
                        heightMaxMm = heightMaxMm.value,
                        speedMmSec = speedMmSec.value,
                        delugeTrack = delugeTrack.value,
                        launchingTrack = launchingTrack.value,
                        danglingTrack = danglingTrack.value,
                        resettingTrack = resettingTrack.value,
                        muteEnable = muteEnable.value,
                        onSendCommand = { name -> writeCommand(name) },
                        onSendValueCommand = { name, value -> writeValueCommand(name, value) }
                    )
                }
            }
        }
        checkPermissionsAndScan()
    }

    override fun onDestroy() {
        super.onDestroy()
        pollingJob?.cancel()
        try {
            bluetoothGatt?.disconnect()
            bluetoothGatt?.close()
        } catch (_: SecurityException) {}
        bluetoothGatt = null
    }

    private fun checkPermissionsAndScan() {
        val requiredPermissions = if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT
            )
        } else {
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        }

        if (requiredPermissions.all { ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED }) {
            startBleScan()
        } else {
            requestPermissionLauncher.launch(requiredPermissions)
        }
    }

    @SuppressLint("MissingPermission")
    private fun startBleScan() {
        val scanner = bluetoothAdapter?.bluetoothLeScanner
        if (scanner == null) {
            connectionStatus.value = "Bluetooth Le Scanner unavailable"
            return
        }

        connectionStatus.value = "Scanning for $TARGET_DEVICE_NAME..."
        val scanCallback = object : ScanCallback() {
            override fun onScanResult(callbackType: Int, result: ScanResult?) {
                val device = result?.device
                val name = device?.name ?: result?.scanRecord?.deviceName
                if (name == TARGET_DEVICE_NAME) {
                    scanner.stopScan(this)
                    connectionStatus.value = "Connecting..."
                    device?.connectGatt(applicationContext, false, gattCallback)
                }
            }
        }
        try {
            scanner.startScan(scanCallback)
        } catch (e: SecurityException) {
            connectionStatus.value = "Permission denied for scan"
        }
    }

    @SuppressLint("PrivateApi")
    private fun refreshGattCache(gatt: BluetoothGatt?): Boolean {
        return try {
            val refreshMethod = gatt?.javaClass?.getMethod("refresh")
            refreshMethod?.invoke(gatt) as? Boolean ?: false
        } catch (e: Exception) {
            false
        }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                bluetoothGatt = gatt
                connectionStatus.value = "Connected"
                refreshGattCache(gatt)
                gatt.discoverServices()
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                connectionStatus.value = "Disconnected (Re-scanning...)"
                pollingJob?.cancel()
                try {
                    gatt.close()
                } catch (_: SecurityException) {}
                bluetoothGatt = null
                readQueue.clear()
                isReading = false
                activeReadUuid = null

                android.os.Handler(mainLooper).postDelayed({
                    startBleScan()
                }, 1000)
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                readQueue.clear()
                ALL_CHARACTERISTIC_KEYS.forEach { key ->
                    CHAR_UUIDS[key]?.let { readQueue.add(it) }
                }
                processNextRead()
                startPeriodicPolling()
            }
        }

        // Callback for Android 13+ (API 33+)
        override fun onCharacteristicRead(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray,
            status: Int
        ) {
            handleCharacteristicRead(characteristic, value, status)
        }

        // Callback for Android 12 and below (API < 33)
        @Deprecated("Deprecated in Java")
        override fun onCharacteristicRead(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            status: Int
        ) {
            handleCharacteristicRead(characteristic, characteristic.value ?: byteArrayOf(), status)
        }

        private fun handleCharacteristicRead(
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray,
            status: Int
        ) {
            val uuid = characteristic.uuid
            val nowMs = SystemClock.elapsedRealtime()

            // Ignore a duplicate callback for the exact same UUID just completed
            if (!isReading && uuid == lastCompletedReadUuid && (nowMs - lastCompletedReadAtMs) < 150L) {
                return
            }

            isReading = false
            activeReadUuid = null
            lastCompletedReadUuid = uuid
            lastCompletedReadAtMs = nowMs

            if (status == BluetoothGatt.GATT_SUCCESS) {
                val longVal = parseLittleEndianLong(value)

                when (uuid) {
                    CHAR_UUIDS["EMERGENCY_STOP_ENABLE"] -> emergencyStop.value = longVal.toInt() != 0
                    CHAR_UUIDS["STATE"] -> {
                        if (longVal in launcherStates.indices) {
                            deviceStateName.value = launcherStates[longVal.toInt()]
                        }
                    }
                    CHAR_UUIDS["HEIGHT_CURRENT_MM"] -> {
                        currentHeightMm.value = if (longVal.toInt() == -1) "Unknown" else "$longVal mm"
                    }
                    CHAR_UUIDS["T0"] -> t0EpochSec.value = longVal
                    CHAR_UUIDS["OFF_PEAK"] -> offPeakEnable.value = longVal.toInt() != 0
                    CHAR_UUIDS["OFF_PEAK_START_SECONDS_SINCE_MIDNIGHT"] -> offPeakStartSec.value = longVal.toInt()
                    CHAR_UUIDS["OFF_PEAK_DURATION_SECONDS"] -> offPeakDurationSec.value = longVal.toInt()
                    CHAR_UUIDS["OFF_PEAK_LAUNCH_INTERVAL_SECONDS"] -> offPeakLaunchIntervalSec.value = longVal.toString()
                    CHAR_UUIDS["PEAK"] -> peakEnable.value = longVal.toInt() != 0
                    CHAR_UUIDS["PEAK_START_SECONDS_SINCE_MIDNIGHT"] -> peakStartSec.value = longVal.toInt()
                    CHAR_UUIDS["PEAK_DURATION_SECONDS"] -> peakDurationSec.value = longVal.toInt()
                    CHAR_UUIDS["PEAK_LAUNCH_INTERVAL_SECONDS"] -> peakLaunchIntervalSec.value = longVal.toString()
                    CHAR_UUIDS["HEIGHT_MAX_MM"] -> heightMaxMm.value = longVal.toString()
                    CHAR_UUIDS["SPEED_MM_PER_SECOND"] -> speedMmSec.value = longVal.toString()
                    CHAR_UUIDS["COMMAND_TRACK_NUMBER_DELUGE"] -> delugeTrack.value = longVal.toString()
                    CHAR_UUIDS["COMMAND_TRACK_NUMBER_LAUNCHING"] -> launchingTrack.value = longVal.toString()
                    CHAR_UUIDS["COMMAND_TRACK_NUMBER_DANGLING"] -> danglingTrack.value = longVal.toString()
                    CHAR_UUIDS["COMMAND_TRACK_NUMBER_RESETTING"] -> resettingTrack.value = longVal.toString()
                    CHAR_UUIDS["MUTE"] -> muteEnable.value = longVal.toInt() != 0
                }
            }
            processNextRead()
        }
    }

    private fun startPeriodicPolling() {
        pollingJob?.cancel()
        pollingJob = scope.launch {
            while (isActive && bluetoothGatt != null) {
                queueRead(CHAR_UUIDS["STATE"])
                queueRead(CHAR_UUIDS["HEIGHT_CURRENT_MM"])
                queueRead(CHAR_UUIDS["T0"])
                delay(3000)
            }
        }
    }

    private fun queueRead(uuid: UUID?) {
        uuid?.let {
            if (!readQueue.contains(it)) {
                readQueue.add(it)
            }
            processNextRead()
        }
    }

    private fun queueAllSettingsRead() {
        ALL_CHARACTERISTIC_KEYS.forEach { key ->
            CHAR_UUIDS[key]?.let { readQueue.add(it) }
        }
        processNextRead()
    }

    @SuppressLint("MissingPermission")
    private fun processNextRead() {
        if (isReading || readQueue.isEmpty()) return
        val gatt = bluetoothGatt ?: return
        val nextUuid = readQueue.poll() ?: return
        val service = gatt.getService(SERVICE_UUID)
        val characteristic = service?.getCharacteristic(nextUuid)

        if (characteristic != null) {
            isReading = true
            activeReadUuid = nextUuid
            val success = gatt.readCharacteristic(characteristic)
            if (!success) {
                isReading = false
                activeReadUuid = null
                processNextRead()
            }
        } else {
            processNextRead()
        }
    }

    private fun parseLittleEndianLong(bytes: ByteArray): Long {
        if (bytes.isEmpty()) return 0L
        var result = 0L
        val count = minOf(bytes.size, 8)
        for (i in 0 until count) {
            result = result or ((bytes[i].toLong() and 0xFFL) shl (i * 8))
        }
        return result
    }

    private fun interruptAndPrepareForAction() {
        val wasReadingOrQueued = isReading || readQueue.isNotEmpty()
        readQueue.clear()
        isReading = false
        activeReadUuid = null

        if (wasReadingOrQueued) {
            android.os.Handler(mainLooper).postDelayed({
                if (readQueue.isEmpty()) {
                    queueAllSettingsRead()
                }
            }, 3000)
        }
    }

    @SuppressLint("MissingPermission")
    private fun writeCommand(charKey: String) {
        val gatt = bluetoothGatt ?: return
        val service = gatt.getService(SERVICE_UUID) ?: return
        val uuid = CHAR_UUIDS[charKey] ?: return
        val characteristic = service.getCharacteristic(uuid) ?: return

        interruptAndPrepareForAction()

        characteristic.value = byteArrayOf()
        characteristic.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
        gatt.writeCharacteristic(characteristic)

        if (charKey == "THIS_IS_GROUND_LEVEL") {
            android.os.Handler(mainLooper).postDelayed({
                queueRead(CHAR_UUIDS["HEIGHT_CURRENT_MM"])
            }, 500)
        } else if (charKey == "THIS_IS_HEIGHT_MAX") {
            android.os.Handler(mainLooper).postDelayed({
                queueRead(CHAR_UUIDS["HEIGHT_MAX_MM"])
            }, 500)
        } else if (charKey == "RESET_TO_DEFAULTS") {
            android.os.Handler(mainLooper).postDelayed({
                queueAllSettingsRead()
            }, 1000)
        }
    }

    @SuppressLint("MissingPermission")
    private fun writeValueCommand(charKey: String, intValue: Int) {
        val gatt = bluetoothGatt ?: return
        val service = gatt.getService(SERVICE_UUID) ?: return
        val uuid = CHAR_UUIDS[charKey] ?: return
        val characteristic = service.getCharacteristic(uuid) ?: return

        interruptAndPrepareForAction()

        val byteArray = byteArrayOf(
            (intValue and 0xFF).toByte(),
            ((intValue ushr 8) and 0xFF).toByte(),
            ((intValue ushr 16) and 0xFF).toByte(),
            ((intValue ushr 24) and 0xFF).toByte()
        )
        characteristic.value = byteArray
        characteristic.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
        gatt.writeCharacteristic(characteristic)

        android.os.Handler(mainLooper).postDelayed({
            queueRead(uuid)
        }, 500)
    }
}

private fun formatEpochToClockTime(epochSec: Long): String {
    val formatter = SimpleDateFormat("HH:mm:ss", Locale.getDefault())
    return formatter.format(Date(epochSec * 1000L))
}

private fun formatT0Display(t0EpochSec: Long?, nowEpochSec: Long): String {
    if (t0EpochSec == null || t0EpochSec <= 0L || t0EpochSec == Long.MAX_VALUE) {
        return "T0: --"
    }
    val absTimeStr = formatEpochToClockTime(t0EpochSec)
    val deltaSec = t0EpochSec - nowEpochSec
    // Future: deltaSec > 0 -> "T-HH:MM:SS"; Past: deltaSec <= 0 -> "T+HH:MM:SS"
    val sign = if (deltaSec > 0L) "-" else "+"
    val magSec = abs(deltaSec)
    val hours = magSec / 3600L
    val minutes = (magSec % 3600L) / 60L
    val seconds = magSec % 60L
    val countdownStr = String.format(Locale.US, "T%s%02d:%02d:%02d", sign, hours, minutes, seconds)
    return "T0: $absTimeStr ($countdownStr)"
}

private fun formatSecondsToHhMm(secondsSinceMidnight: Int?): String {
    if (secondsSinceMidnight == null) return ""
    val clamped = secondsSinceMidnight.coerceIn(0, 86399)
    val hours = clamped / 3600
    val minutes = (clamped % 3600) / 60
    return String.format(Locale.US, "%02d:%02d", hours, minutes)
}

private fun parseHhMmToSeconds(hhMm: String): Int? {
    val trimmed = hhMm.trim()
    val parts = trimmed.split(":")
    if (parts.size != 2 || parts[0].isEmpty() || parts[1].length != 2) return null
    val hours = parts[0].toIntOrNull() ?: return null
    val minutes = parts[1].toIntOrNull() ?: return null
    if (hours !in 0..23 || minutes !in 0..59) return null
    return hours * 3600 + minutes * 60
}

private fun formatDurationToHhMm(durationSeconds: Int?): String {
    if (durationSeconds == null) return ""
    val nonNeg = durationSeconds.coerceAtLeast(0)
    val hours = nonNeg / 3600
    val minutes = (nonNeg % 3600) / 60
    return String.format(Locale.US, "%02d:%02d", hours, minutes)
}

private fun parseDurationHhMmToSeconds(hhMm: String): Int? {
    val trimmed = hhMm.trim()
    val parts = trimmed.split(":")
    if (parts.size != 2 || parts[0].isEmpty() || parts[1].length != 2) return null
    val hours = parts[0].toIntOrNull() ?: return null
    val minutes = parts[1].toIntOrNull() ?: return null
    if (hours < 0 || minutes !in 0..59) return null
    val totalSeconds = hours.toLong() * 3600L + minutes.toLong() * 60L
    if (totalSeconds > Int.MAX_VALUE.toLong()) return null
    return totalSeconds.toInt()
}

@Composable
fun MoveActionButton(
    text: String,
    charKey: String,
    targetStateName: String,
    isConnected: Boolean,
    deviceState: String,
    containerColor: Color = MaterialTheme.colorScheme.primary,
    contentColor: Color = MaterialTheme.colorScheme.onPrimary,
    modifier: Modifier = Modifier,
    onWriteValue: (String, Int) -> Unit
) {
    val coroutineScope = rememberCoroutineScope()

    val backgroundColor = if (isConnected) containerColor else containerColor.copy(alpha = 0.12f)
    val textColor = if (isConnected) contentColor else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)

    Box(
        contentAlignment = Alignment.Center,
        modifier = modifier
            .height(40.dp)
            .background(
                color = backgroundColor,
                shape = MaterialTheme.shapes.extraLarge
            )
            .pointerInput(isConnected) {
                if (!isConnected) return@pointerInput
                detectTapGestures(
                    onPress = {
                        onWriteValue(charKey, 1)

                        val pressJob = coroutineScope.launch {
                            while (isActive) {
                                delay(250)
                                onWriteValue(charKey, 1)
                            }
                        }

                        try {
                            awaitRelease()
                        } finally {
                            pressJob.cancel()

                            coroutineScope.launch {
                                val startTime = System.currentTimeMillis()
                                while (isActive) {
                                    onWriteValue(charKey, 0)

                                    if (deviceState != targetStateName || (System.currentTimeMillis() - startTime) > 5000) {
                                        break
                                    }
                                    delay(250)
                                }
                            }
                        }
                    }
                )
            }
            .padding(horizontal = 12.dp)
    ) {
        Text(
            text = text,
            style = MaterialTheme.typography.labelLarge,
            color = textColor
        )
    }
}

@Composable
fun SectionDivider(title: String) {
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 4.dp),
        contentAlignment = Alignment.Center
    ) {
        HorizontalDivider(
            modifier = Modifier.fillMaxWidth(),
            thickness = 1.dp,
            color = MaterialTheme.colorScheme.outlineVariant
        )
        Surface(
            color = MaterialTheme.colorScheme.background,
            modifier = Modifier.padding(horizontal = 8.dp)
        ) {
            Text(
                text = title,
                style = MaterialTheme.typography.titleSmall,
                color = MaterialTheme.colorScheme.primary,
                modifier = Modifier.padding(horizontal = 4.dp)
            )
        }
    }
}

@Composable
fun SwitchParamRow(
    label: String,
    value: Boolean?,
    isConnected: Boolean,
    onSave: (Int) -> Unit,
    confirmAndExecute: (String, String, () -> Unit) -> Unit,
    modifier: Modifier = Modifier
) {
    if (value != null) {
        var fieldVal by remember(value) { mutableStateOf(value) }
        Row(
            verticalAlignment = Alignment.CenterVertically,
            modifier = modifier
                .fillMaxWidth()
                .heightIn(min = 20.dp)
                .padding(vertical = 0.dp)
        ) {
            Text(
                text = label,
                style = MaterialTheme.typography.bodyMedium,
                color = if (isConnected) MaterialTheme.colorScheme.onSurface else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)
            )
            Spacer(modifier = Modifier.weight(1f))
            Switch(
                checked = fieldVal,
                enabled = isConnected,
                onCheckedChange = { newVal ->
                    confirmAndExecute("Toggle $label", "Are you sure you want to change $label?") {
                        fieldVal = newVal
                        onSave(if (newVal) 1 else 0)
                    }
                },
                modifier = Modifier
                    .height(36.dp)
                    .padding(4.dp)
            )
        }
    } else {
        Text(
            "$label: Connecting / Reading...",
            style = MaterialTheme.typography.bodySmall,
            color = Color.Gray,
            modifier = modifier.padding(vertical = 0.dp)
        )
    }
}

@Composable
fun CompactParamRow(
    label: String,
    value: String?,
    isConnected: Boolean,
    onSave: (Int) -> Unit,
    confirmAndExecute: (String, String, () -> Unit) -> Unit,
    onFieldFocusChange: (Boolean) -> Unit = {}
) {
    var textValue by remember(value) { mutableStateOf(value ?: "") }
    val focusManager = LocalFocusManager.current

    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 4.dp),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(
            text = label,
            modifier = Modifier.weight(1f),
            style = MaterialTheme.typography.bodyMedium,
            color = if (isConnected) MaterialTheme.colorScheme.onSurface else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)
        )

        BasicTextField(
            value = textValue,
            onValueChange = { textValue = it },
            enabled = isConnected,
            singleLine = true,
            textStyle = MaterialTheme.typography.bodyMedium.copy(
                color = if (isConnected) MaterialTheme.colorScheme.onSurface else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)
            ),
            modifier = Modifier
                .width(70.dp)
                .background(MaterialTheme.colorScheme.surfaceVariant, MaterialTheme.shapes.small)
                .padding(horizontal = 8.dp, vertical = 6.dp)
                .onFocusChanged { focusState ->
                    onFieldFocusChange(focusState.isFocused)
                    if (!focusState.isFocused) {
                        textValue = value ?: ""
                    }
                }
        )

        Button(
            onClick = {
                textValue.toIntOrNull()?.let { intVal ->
                    confirmAndExecute("Update $label", "Are you sure you want to update $label to $intVal?") {
                        focusManager.clearFocus()
                        onSave(intVal)
                    }
                }
            },
            enabled = isConnected && textValue.toIntOrNull() != null && value != null,
            contentPadding = PaddingValues(horizontal = 12.dp, vertical = 4.dp),
            modifier = Modifier.height(36.dp)
        ) {
            Text("Save", style = MaterialTheme.typography.labelMedium)
        }
    }
}

@Composable
fun CompactTimeHhMmRow(
    label: String,
    startSeconds: Int?,
    isConnected: Boolean,
    onSaveSeconds: (Int) -> Unit,
    confirmAndExecute: (String, String, () -> Unit) -> Unit,
    onFieldFocusChange: (Boolean) -> Unit = {}
) {
    val formattedInitial = remember(startSeconds) { formatSecondsToHhMm(startSeconds) }
    var textValue by remember(formattedInitial) { mutableStateOf(formattedInitial) }
    val focusManager = LocalFocusManager.current
    val parsedSeconds = parseHhMmToSeconds(textValue)

    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 4.dp),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(
            text = label,
            modifier = Modifier.weight(1f),
            style = MaterialTheme.typography.bodyMedium,
            color = if (isConnected) MaterialTheme.colorScheme.onSurface else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)
        )

        BasicTextField(
            value = textValue,
            onValueChange = { textValue = it },
            enabled = isConnected,
            singleLine = true,
            textStyle = MaterialTheme.typography.bodyMedium.copy(
                color = if (isConnected) MaterialTheme.colorScheme.onSurface else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)
            ),
            modifier = Modifier
                .width(70.dp)
                .background(MaterialTheme.colorScheme.surfaceVariant, MaterialTheme.shapes.small)
                .padding(horizontal = 8.dp, vertical = 6.dp)
                .onFocusChanged { focusState ->
                    onFieldFocusChange(focusState.isFocused)
                    if (!focusState.isFocused) {
                        textValue = formattedInitial
                    }
                }
        )

        Button(
            onClick = {
                parsedSeconds?.let { secVal ->
                    val normalizedHhMm = formatSecondsToHhMm(secVal)
                    confirmAndExecute(
                        "Update $label",
                        "Are you sure you want to update $label to $normalizedHhMm ($secVal s)?"
                    ) {
                        focusManager.clearFocus()
                        onSaveSeconds(secVal)
                    }
                }
            },
            enabled = isConnected && parsedSeconds != null && startSeconds != null,
            contentPadding = PaddingValues(horizontal = 12.dp, vertical = 4.dp),
            modifier = Modifier.height(36.dp)
        ) {
            Text("Save", style = MaterialTheme.typography.labelMedium)
        }
    }
}

@Composable
fun CompactDurationHhMmRow(
    label: String,
    durationSeconds: Int?,
    isConnected: Boolean,
    onSaveSeconds: (Int) -> Unit,
    confirmAndExecute: (String, String, () -> Unit) -> Unit,
    onFieldFocusChange: (Boolean) -> Unit = {}
) {
    val formattedInitial = remember(durationSeconds) { formatDurationToHhMm(durationSeconds) }
    var textValue by remember(formattedInitial) { mutableStateOf(formattedInitial) }
    val focusManager = LocalFocusManager.current
    val parsedSeconds = parseDurationHhMmToSeconds(textValue)

    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 4.dp),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(
            text = label,
            modifier = Modifier.weight(1f),
            style = MaterialTheme.typography.bodyMedium,
            color = if (isConnected) MaterialTheme.colorScheme.onSurface else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)
        )

        BasicTextField(
            value = textValue,
            onValueChange = { textValue = it },
            enabled = isConnected,
            singleLine = true,
            textStyle = MaterialTheme.typography.bodyMedium.copy(
                color = if (isConnected) MaterialTheme.colorScheme.onSurface else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)
            ),
            modifier = Modifier
                .width(70.dp)
                .background(MaterialTheme.colorScheme.surfaceVariant, MaterialTheme.shapes.small)
                .padding(horizontal = 8.dp, vertical = 6.dp)
                .onFocusChanged { focusState ->
                    onFieldFocusChange(focusState.isFocused)
                    if (!focusState.isFocused) {
                        textValue = formattedInitial
                    }
                }
        )

        Button(
            onClick = {
                parsedSeconds?.let { secVal ->
                    val normalizedHhMm = formatDurationToHhMm(secVal)
                    confirmAndExecute(
                        "Update $label",
                        "Are you sure you want to update $label to $normalizedHhMm ($secVal s)?"
                    ) {
                        focusManager.clearFocus()
                        onSaveSeconds(secVal)
                    }
                }
            },
            enabled = isConnected && parsedSeconds != null && durationSeconds != null,
            contentPadding = PaddingValues(horizontal = 12.dp, vertical = 4.dp),
            modifier = Modifier.height(36.dp)
        ) {
            Text("Save", style = MaterialTheme.typography.labelMedium)
        }
    }
}

@Composable
fun ExpandablePeriodSection(
    title: String,
    enableLabel: String,
    enabledValue: Boolean?,
    startSecValue: Int?,
    durationSecValue: Int?,
    intervalSecValue: String?,
    isConnected: Boolean,
    isExpanded: Boolean,
    onExpandedChange: (Boolean) -> Unit,
    onToggleEnable: (Int) -> Unit,
    onSaveStartSec: (Int) -> Unit,
    onSaveDurationSec: (Int) -> Unit,
    onSaveIntervalSec: (Int) -> Unit,
    confirmAndExecute: (String, String, () -> Unit) -> Unit
) {
    var focusedChildCount by remember { mutableIntStateOf(0) }

    LaunchedEffect(focusedChildCount) {
        onExpandedChange(focusedChildCount > 0)
    }

    fun childFocusCallback(isFocused: Boolean) {
        focusedChildCount = (focusedChildCount + if (isFocused) 1 else -1).coerceAtLeast(0)
    }

    Column(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 2.dp)
            .background(
                color = if (isExpanded) MaterialTheme.colorScheme.surfaceVariant.copy(alpha = 0.35f) else Color.Transparent,
                shape = MaterialTheme.shapes.small
            )
            .padding(horizontal = if (isExpanded) 6.dp else 0.dp, vertical = 2.dp)
    ) {
        SwitchParamRow(
            label = enableLabel,
            value = enabledValue,
            isConnected = isConnected,
            onSave = onToggleEnable,
            confirmAndExecute = confirmAndExecute
        )

        Row(
            modifier = Modifier
                .fillMaxWidth()
                .clickable { onExpandedChange(!isExpanded) }
                .padding(vertical = 2.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            val startDisplay = if (startSecValue != null) formatSecondsToHhMm(startSecValue) else "--:--"
            val durationDisplay = if (durationSecValue != null) formatDurationToHhMm(durationSecValue) else "--:--"
            val summary = "Start: $startDisplay • Duration: $durationDisplay • Interval: ${intervalSecValue ?: "--"}s"
            Text(
                text = summary,
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                modifier = Modifier.weight(1f)
            )
            Text(
                text = if (isExpanded) "Hide ▲" else "Edit ▼",
                style = MaterialTheme.typography.labelMedium,
                color = MaterialTheme.colorScheme.primary
            )
        }

        AnimatedVisibility(visible = isExpanded) {
            Column(modifier = Modifier.fillMaxWidth().padding(top = 4.dp)) {
                CompactTimeHhMmRow(
                    label = "$title Start (HH:MM)",
                    startSeconds = startSecValue,
                    isConnected = isConnected,
                    onSaveSeconds = onSaveStartSec,
                    confirmAndExecute = confirmAndExecute,
                    onFieldFocusChange = ::childFocusCallback
                )
                CompactDurationHhMmRow(
                    label = "$title Duration (HH:MM)",
                    durationSeconds = durationSecValue,
                    isConnected = isConnected,
                    onSaveSeconds = onSaveDurationSec,
                    confirmAndExecute = confirmAndExecute,
                    onFieldFocusChange = ::childFocusCallback
                )
                CompactParamRow(
                    label = "$title Launch Interval (sec)",
                    value = intervalSecValue,
                    isConnected = isConnected,
                    onSave = onSaveIntervalSec,
                    confirmAndExecute = confirmAndExecute,
                    onFieldFocusChange = ::childFocusCallback
                )
            }
        }
    }
}

@Composable
fun MainScreen(
    connectionStatus: String,
    deviceState: String,
    currentHeight: String?,
    t0EpochSec: Long?,
    emergencyStop: Boolean?,
    offPeakEnable: Boolean?,
    offPeakStartSec: Int?,
    offPeakDurationSec: Int?,
    offPeakLaunchIntervalSec: String?,
    peakEnable: Boolean?,
    peakStartSec: Int?,
    peakDurationSec: Int?,
    peakLaunchIntervalSec: String?,
    heightMaxMm: String?,
    speedMmSec: String?,
    delugeTrack: String?,
    launchingTrack: String?,
    danglingTrack: String?,
    resettingTrack: String?,
    muteEnable: Boolean?,
    onSendCommand: (String) -> Unit,
    onSendValueCommand: (String, Int) -> Unit
) {
    val scrollState = rememberScrollState()
    val focusManager = LocalFocusManager.current
    val isConnected = (connectionStatus == "Connected")
    val isEStopActive = emergencyStop == true
    val areMotorsEnabled = isConnected && !isEStopActive

    var nowEpochSec by remember { mutableLongStateOf(System.currentTimeMillis() / 1000L) }
    LaunchedEffect(Unit) {
        while (isActive) {
            nowEpochSec = System.currentTimeMillis() / 1000L
            delay(1000L)
        }
    }

    var offPeakExpanded by remember { mutableStateOf(false) }
    var peakExpanded by remember { mutableStateOf(false) }

    var showDialog by remember { mutableStateOf(false) }
    var dialogTitle by remember { mutableStateOf("") }
    var dialogMessage by remember { mutableStateOf("") }
    var pendingAction by remember { mutableStateOf<(() -> Unit)?>(null) }

    fun confirmAndExecute(title: String, message: String, action: () -> Unit) {
        dialogTitle = title
        dialogMessage = message
        pendingAction = action
        showDialog = true
    }

    if (showDialog) {
        AlertDialog(
            onDismissRequest = { showDialog = false },
            title = { Text(dialogTitle) },
            text = { Text(dialogMessage) },
            confirmButton = {
                TextButton(onClick = {
                    showDialog = false
                    pendingAction?.invoke()
                }) {
                    Text("Confirm")
                }
            },
            dismissButton = {
                TextButton(onClick = { showDialog = false }) {
                    Text("Cancel")
                }
            }
        )
    }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .pointerInput(Unit) {
                detectTapGestures(onTap = {
                    focusManager.clearFocus()
                })
            }
            .padding(8.dp)
            .statusBarsPadding()
            .verticalScroll(scrollState),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text("Presents to the Moon", style = MaterialTheme.typography.headlineMedium)
        Spacer(modifier = Modifier.height(8.dp))

        // Connection & Status Card
        Card(
            modifier = Modifier.fillMaxWidth(),
            colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant)
        ) {
            Column(modifier = Modifier.padding(4.dp)) {
                Text("Connection: $connectionStatus", style = MaterialTheme.typography.bodyLarge)
                Text("Device State: $deviceState", style = MaterialTheme.typography.titleMedium, color = Color.Red)
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text(
                        "Current Height: ${currentHeight ?: "--"}",
                        style = MaterialTheme.typography.bodyMedium
                    )
                    Text(
                        formatT0Display(t0EpochSec, nowEpochSec),
                        style = MaterialTheme.typography.bodyMedium
                    )
                }
                if (isEStopActive) {
                    Text("EMERGENCY STOP ENGAGED", style = MaterialTheme.typography.titleMedium, color = MaterialTheme.colorScheme.error)
                }
            }
        }

        Spacer(modifier = Modifier.height(16.dp))

        // Launch and E-Stop Row
        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
            Button(
                onClick = { onSendCommand("LAUNCH_NOW") },
                enabled = areMotorsEnabled,
                colors = ButtonDefaults.buttonColors(
                    containerColor = MaterialTheme.colorScheme.primary,
                    contentColor = MaterialTheme.colorScheme.onPrimary,
                    disabledContainerColor = MaterialTheme.colorScheme.primary.copy(alpha = 0.12f),
                    disabledContentColor = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)
                ),
                elevation = ButtonDefaults.buttonElevation(defaultElevation = 4.dp),
                modifier = Modifier.weight(1f).padding(horizontal = 4.dp)
            ) {
                Text("Launch")
            }

            val eStopContainerColor = if (isEStopActive) MaterialTheme.colorScheme.error else MaterialTheme.colorScheme.primary
            val eStopContentColor = if (isEStopActive) MaterialTheme.colorScheme.onError else MaterialTheme.colorScheme.onPrimary

            Button(
                onClick = {
                    if (isEStopActive) {
                        confirmAndExecute(
                            "Clear E-Stop",
                            "Are you sure you want to clear the Emergency Stop?"
                        ) {
                            onSendValueCommand("EMERGENCY_STOP_ENABLE", 0)
                        }
                    } else {
                        onSendValueCommand("EMERGENCY_STOP_ENABLE", 1)
                    }
                },
                enabled = isConnected,
                colors = ButtonDefaults.buttonColors(
                    containerColor = eStopContainerColor,
                    contentColor = eStopContentColor,
                    disabledContainerColor = MaterialTheme.colorScheme.primary.copy(alpha = 0.12f),
                    disabledContentColor = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.38f)
                ),
                elevation = ButtonDefaults.buttonElevation(defaultElevation = if (isEStopActive) 8.dp else 4.dp),
                modifier = Modifier.weight(1f).padding(horizontal = 4.dp)
            ) {
                Text(if (isEStopActive) "E-STOP ACTIVE" else "EMERGENCY STOP", style = MaterialTheme.typography.labelLarge)
            }
        }

        // Move up / Move down row
        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
            MoveActionButton(
                text = "Move up",
                charKey = "UP",
                targetStateName = "MOVE_UP",
                isConnected = areMotorsEnabled,
                deviceState = deviceState,
                containerColor = MaterialTheme.colorScheme.primary,
                contentColor = MaterialTheme.colorScheme.onPrimary,
                modifier = Modifier.weight(1f).padding(horizontal = 4.dp),
                onWriteValue = { name, value -> onSendValueCommand(name, value) }
            )
            MoveActionButton(
                text = "Move down",
                charKey = "DOWN",
                targetStateName = "MOVE_DOWN",
                isConnected = areMotorsEnabled,
                deviceState = deviceState,
                containerColor = MaterialTheme.colorScheme.primary,
                contentColor = MaterialTheme.colorScheme.onPrimary,
                modifier = Modifier.weight(1f).padding(horizontal = 4.dp),
                onWriteValue = { name, value -> onSendValueCommand(name, value) }
            )
        }

        // Guarded Position / Reset Controls
        SectionDivider("Calibration")
        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly) {
            Button(
                onClick = {
                    confirmAndExecute("Set Ground Level", "Are you sure you want to set the current position as ground level?") {
                        onSendCommand("THIS_IS_GROUND_LEVEL")
                    }
                },
                enabled = isConnected
            ) { Text("This is ground level") }

            Button(
                onClick = {
                    confirmAndExecute("Set Max Height", "Are you sure you want to record the current height as the new maximum height?") {
                        onSendCommand("THIS_IS_HEIGHT_MAX")
                    }
                },
                enabled = isConnected
            ) { Text("This is max height") }
        }

        // Settings & Configurations
        SectionDivider("Settings")
        Button(
            onClick = {
                confirmAndExecute("Reset to Defaults", "Are you sure you want to reset all device settings and NVRAM to defaults?") {
                    onSendCommand("RESET_TO_DEFAULTS")
                }
            },
            enabled = isConnected,
            colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.error)
        ) {
            Text("Reset to Defaults")
        }

        Spacer(modifier = Modifier.height(4.dp))

        ExpandablePeriodSection(
            title = "Peak",
            enableLabel = "Peak Enable",
            enabledValue = peakEnable,
            startSecValue = peakStartSec,
            durationSecValue = peakDurationSec,
            intervalSecValue = peakLaunchIntervalSec,
            isConnected = isConnected,
            isExpanded = peakExpanded,
            onExpandedChange = { peakExpanded = it },
            onToggleEnable = { onSendValueCommand("PEAK", it) },
            onSaveStartSec = { onSendValueCommand("PEAK_START_SECONDS_SINCE_MIDNIGHT", it) },
            onSaveDurationSec = { onSendValueCommand("PEAK_DURATION_SECONDS", it) },
            onSaveIntervalSec = { onSendValueCommand("PEAK_LAUNCH_INTERVAL_SECONDS", it) },
            confirmAndExecute = ::confirmAndExecute
        )

        HorizontalDivider(modifier = Modifier.padding(vertical = 2.dp), thickness = 0.5.dp)

        ExpandablePeriodSection(
            title = "Off-Peak",
            enableLabel = "Off-Peak Enable",
            enabledValue = offPeakEnable,
            startSecValue = offPeakStartSec,
            durationSecValue = offPeakDurationSec,
            intervalSecValue = offPeakLaunchIntervalSec,
            isConnected = isConnected,
            isExpanded = offPeakExpanded,
            onExpandedChange = { offPeakExpanded = it },
            onToggleEnable = { onSendValueCommand("OFF_PEAK", it) },
            onSaveStartSec = { onSendValueCommand("OFF_PEAK_START_SECONDS_SINCE_MIDNIGHT", it) },
            onSaveDurationSec = { onSendValueCommand("OFF_PEAK_DURATION_SECONDS", it) },
            onSaveIntervalSec = { onSendValueCommand("OFF_PEAK_LAUNCH_INTERVAL_SECONDS", it) },
            confirmAndExecute = ::confirmAndExecute
        )

        HorizontalDivider(modifier = Modifier.padding(vertical = 2.dp), thickness = 0.5.dp)

        SwitchParamRow(
            label = "Mute Enable",
            value = muteEnable,
            isConnected = isConnected,
            onSave = { onSendValueCommand("MUTE", it) },
            confirmAndExecute = ::confirmAndExecute
        )

        CompactParamRow(
            label = "Speed (mm/s)",
            value = speedMmSec,
            isConnected = isConnected,
            onSave = { onSendValueCommand("SPEED_MM_PER_SECOND", it) },
            confirmAndExecute = ::confirmAndExecute
        )
        CompactParamRow(
            label = "Max Height (mm)",
            value = heightMaxMm,
            isConnected = isConnected,
            onSave = { onSendValueCommand("HEIGHT_MAX_MM", it) },
            confirmAndExecute = ::confirmAndExecute
        )
        CompactParamRow(
            label = "Deluge Track",
            value = delugeTrack,
            isConnected = isConnected,
            onSave = { onSendValueCommand("COMMAND_TRACK_NUMBER_DELUGE", it) },
            confirmAndExecute = ::confirmAndExecute
        )
        CompactParamRow(
            label = "Launching Track",
            value = launchingTrack,
            isConnected = isConnected,
            onSave = { onSendValueCommand("COMMAND_TRACK_NUMBER_LAUNCHING", it) },
            confirmAndExecute = ::confirmAndExecute
        )
        CompactParamRow(
            label = "Dangling Track",
            value = danglingTrack,
            isConnected = isConnected,
            onSave = { onSendValueCommand("COMMAND_TRACK_NUMBER_DANGLING", it) },
            confirmAndExecute = ::confirmAndExecute
        )
        CompactParamRow(
            label = "Resetting Track",
            value = resettingTrack,
            isConnected = isConnected,
            onSave = { onSendValueCommand("COMMAND_TRACK_NUMBER_RESETTING", it) },
            confirmAndExecute = ::confirmAndExecute
        )

        Spacer(modifier = Modifier.height(24.dp))
    }
}