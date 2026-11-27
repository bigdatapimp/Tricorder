# Data Log: what to add to the app project

The Data Log records the Tricorder's readings to a CSV file on the phone and keeps going with the screen off. For that
Android needs a **foreground service**, which must be declared in `AndroidManifest.xml`. This cannot be done from the
Kotlin files.

## 1. New file
Add `LogService.kt` and `DataLog.kt` to the same folder as the other Kotlin files (package `com.example.tricorder`).

## 2. AndroidManifest.xml

Inside `<manifest>`, next to the other `<uses-permission>` lines (before `<application>`):

```xml
<!-- Data log: keeps recording with the screen off -->
<uses-permission android:name="android.permission.FOREGROUND_SERVICE" />
<uses-permission android:name="android.permission.FOREGROUND_SERVICE_CONNECTED_DEVICE" />
<uses-permission android:name="android.permission.POST_NOTIFICATIONS" />
```

Inside `<application>`, next to the `<activity>`:

```xml
<service
    android:name=".LogService"
    android:exported="false"
    android:foregroundServiceType="connectedDevice" />
```

(`.LogService` assumes the manifest's package / namespace is `com.example.tricorder`, as for `.MainActivity`.)

**Sharing a recording** (the Share CSV / Share GPX keys) also needs this block inside `<application>`, next to the `<service>`:

```xml
<provider
    android:name="androidx.core.content.FileProvider"
    android:authorities="${applicationId}.fileprovider"
    android:exported="false"
    android:grantUriPermissions="true">
    <meta-data
        android:name="android.support.FILE_PROVIDER_PATHS"
        android:resource="@xml/file_paths" />
</provider>
```

and a new file `app/src/main/res/xml/file_paths.xml` (create the `xml` folder if it is not there; it may already have `backup_rules.xml`
and `data_extraction_rules.xml`): copy `res/xml/file_paths.xml` from the update.

## 3. Replace
`TricorderBle.kt`, `MainActivity.kt`, `Screens.kt`, `DataLog.kt`.

## 4. First test
1. Utility > Data Log, choose 5 s, Start recording. Allow notifications when asked.
2. A notification "Tricorder data log" appears. Switch the phone's screen off and carry on for a few minutes.
3. Open the app: the row count should have kept growing. Stop recording, then save it as CSV and as GPX, and try Share CSV: the Android share sheet should open.

## If recording stops when the screen is off
Some phones put background apps to sleep. Set the app's battery use to **Unrestricted**
(Settings > Apps > Tricorder > Battery) and, on Samsung, take it out of "Sleeping apps".

## If nothing happens when you tap Start
Since this update the screen tells you why. The usual causes, in order:

1. **"Android did not find the log service"**: the `<service>` block is missing from `AndroidManifest.xml`, or it is
   outside `<application>`, or misspelt. Android ignores a request to start a service that is not declared, without any
   error, so without this message the button simply appears to do nothing. Add the block from step 2 and rebuild.
2. **"The log service did not start"** (after about 4 seconds): the service was found but never reported in. Check that
   all three `<uses-permission>` lines are present, and that the service has
   `android:foregroundServiceType="connectedDevice"`.
3. **"Could not start the log service: ..."**: Android refused. The text after the colon says why.
4. **A permission window for notifications appears the first time**: allow it. If you refuse, recording still works but
   its notification stays hidden.

## What is recorded
One row every 1, 2, 5, 10, 30, 60 or 300 s, while the Tricorder is connected:
time (UTC), latitude, longitude, altitude, speed, course, satellites, HDOP, fix age, temperature, humidity, pressure,
battery, and the compass heading (only while the Navigation screen is open on the Tricorder). A value that is not
available (no GPS fix, an old packet) is left empty. Nothing is written while the link is down; recording carries on by
itself when the app has reconnected.
