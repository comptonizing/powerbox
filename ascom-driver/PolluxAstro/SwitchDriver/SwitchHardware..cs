// ASCOM Switch hardware class for PolluxAstroPowerbox
//
// Description: A driver for the DIY powerbox
//
// Implements:	ASCOM Switch interface version: 1
// Author:		Philipp Weber <philipp@whyisitso.de>
//

using ASCOM;
using ASCOM.Astrometry;
using ASCOM.Astrometry.AstroUtils;
using ASCOM.Astrometry.NOVAS;
using ASCOM.DeviceInterface;
using ASCOM.LocalServer;
using ASCOM.Utilities;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Threading;
using System.Windows.Forms;

namespace ASCOM.PolluxAstroPowerbox.Switch
{

    /// <summary>
    /// ASCOM Switch hardware class for PolluxAstroPowerbox.
    /// </summary>
    [HardwareClass()] // Class attribute flag this as a device hardware class that needs to be disposed by the local server when it exits.
    internal static class SwitchHardware
    {
        // Constants used for Profile persistence
        internal const string comPortProfileName = "COM Port";
        internal const string comPortDefault = "COM1";
        internal const string traceStateProfileName = "Trace Level";
        internal const string traceStateDefault = "true";

        private static string DriverProgId = ""; // ASCOM DeviceID (COM ProgID) for this driver, the value is set by the driver's class initialiser.
        private static string DriverDescription = ""; // The value is set by the driver's class initialiser.
        internal static string comPort; // COM port name (if required)
        private static bool connectedState = false; // Local server's connected state
        private static bool runOnce = false; // Flag to enable "one-off" activities only to run once.
        internal static Util utilities; // ASCOM Utilities object for use as required
        internal static AstroUtils astroUtilities; // ASCOM AstroUtilities object for use as required
        internal static TraceLogger tl; // Local server's trace logger object for diagnostic log with information that you specify

        internal static ASCOM.Utilities.Serial port = null;
        internal static int updateInterval = 2000; // milliseconds
        // Go ten seconds into the past to get an update on the next read
        internal static DateTime lastUpdate = DateTime.UtcNow.AddSeconds(-10);
        internal static string lastResponse = null;
        internal static JsonNode lastJson = null;
        internal static Mutex updateInnerMutex = new Mutex();
        internal static Mutex updateOuterMutex = new Mutex();
        internal static Mutex connectedMutex = new Mutex();

        internal static void checkDeviceError(string rsp)
        {
            JsonNode json = JsonSerializer.Deserialize<JsonNode>(rsp);
            if (json["Error"] == null)
            {
                return;
            }
            throw new ASCOM.DriverException("Device error: " + json["Error"].GetValue<double>());
        }

        internal static void updateResponseNow()
        {
            LogMessage("updateResponseNow", "Updating response now");
            LogMessage("updateResponseNow", "Locking inner mutex");
            updateInnerMutex.WaitOne();
            try
            {
                LogMessage("updateResponseNow", "Sending status");
                string rsp = CommandString("status", false);
                LogMessage("updateResponseNow", "Checking for device error");
                checkDeviceError(rsp);
                LogMessage("updateResponseNow", "Deserialising JSON");
                JsonNode json = JsonSerializer.Deserialize<JsonNode>(rsp);
                LogMessage("updateResponseNow", "Setting values");
                lastResponse = rsp;
                lastJson = json;
                lastUpdate = DateTime.UtcNow;
            } finally
            {
                updateInnerMutex.ReleaseMutex();
            }
        }

        internal static void updateResponse()
        {
            LogMessage("updateResponse", "Locking outer mutex");
            updateOuterMutex.WaitOne();
            try
            {
                DateTime now = DateTime.UtcNow;
                if (lastResponse == null ||
                    lastJson == null ||
                    (now - lastUpdate).TotalMilliseconds >= updateInterval )
                {
                    LogMessage("updateResponse", "Updating response");
                    updateResponseNow();
                } else
                {
                    LogMessage("updateResponse", "Using cached response");
                }
            } finally
            {
                updateOuterMutex.ReleaseMutex();
            }
        }

        const short IDVoltage = 0;
        const short IDEnvTemperature = 1;
        const short IDEnvHumidity = 2;
        const short IDEnvPressure = 3;
        const short IDEnvDewpoint = 4;
        const short IDEnvTemperatureOffset = 5;
        const short IDRailState = 6;
        const short IDAdjState = 7;
        const short IDAdjVoltage = 8;
        const short IDDH1Mode = 9;
        const short IDDH1DutyCycle = 10;
        const short IDDH1Temperature = 11;
        const short IDDH1TemperatureOffset = 12;
        const short IDDH1Fixed = 13;
        const short IDDH1DewpointOffset = 14;
        const short IDDH1AmbientOffset = 15;
        const short IDDH1MidpointOffset = 16;
        const short IDDH2Mode = 17;
        const short IDDH2DutyCycle = 18;
        const short IDDH2Temperature = 19;
        const short IDDH2TemperatureOffset = 20;
        const short IDDH2Fixed = 21;
        const short IDDH2DewpointOffset = 22;
        const short IDDH2AmbientOffset = 23;
        const short IDDH2MidpointOffset = 24;
        
        static UInt16 crcUpdate(UInt16 crc, byte a)
        {
            crc ^= a;
            for (int ii=0; ii<8; ii++)
            {
                if ( (crc & 1) != 0)
                {
                    crc >>= 1;
                    crc ^= 0xA001;
                } else
                {
                    crc >>= 1;
                }
            }
            return crc;
        }

        static UInt16 crcCalc(string data)
        {
            UInt16 crc = 0;
            foreach (char c in data)
            {
                crc = crcUpdate(crc, ((byte)c));
            }
            return crc;
        }

        static UInt16 crcCalc(byte[] data)
        {
            UInt16 crc = 0;
            foreach (byte b in data)
            {
                crc = crcUpdate(crc, b);
            }
            return crc;
        }

        static byte[] cmdCrc(string data)
        {
            int l = data.Length;
            UInt16 crc = crcCalc(data);
            byte[] ret = new byte[l + 3];
            for (int ii = 0; ii < l; ii++)
            {
                ret[ii] = (byte)data[ii];
            }
            ret[l] = (byte)'\0';
            ret[l + 1] = (byte)(crc & 0xFF);
            if (ret[l + 1] == '$' )
            {
                ret[l + 1] = (byte)'1';
            }
            ret[l + 2] = (byte)((crc >> 8) & 0xFF);
            if (ret[l + 2] == '$' )
            {
                ret[l + 2] = (byte)'1';
            }

            return ret;
        }

        static bool checkCrc(byte[] rsp)
        {
            int l = rsp.Length;
            UInt16 crcGotten = 0;
            byte[] crcGottenBytes = new byte[] { rsp[l - 2], rsp[l - 1] };
            crcGotten |= (UInt16)rsp[l - 2];
            crcGotten |= (UInt16)(rsp[l - 1] << 8);

            rsp = rsp.Take(l - 3).ToArray();
            UInt16 crcCalculated = crcCalc(rsp);
            byte[] crcBytes = new byte[2];
            crcBytes[0] = (byte)(crcCalculated & 0x00FF);
            crcBytes[1] = (byte)((crcCalculated >> 8) & 0x00FF);
            if (crcBytes[0] == '$' )
            {
                crcBytes[0] = (byte)'1';
            }
            if (crcBytes[1] == '$' )
            {
                crcBytes[1] = (byte)'1';
            }

            return (crcBytes[0] == crcGottenBytes[0]) && (crcBytes[1] == crcGottenBytes[1]);
        }

        /// <summary>
        /// Initializes a new instance of the device Hardware class.
        /// </summary>
        static SwitchHardware()
        {
            try
            {
                // Create the hardware trace logger in the static initialiser.
                // All other initialisation should go in the InitialiseHardware method.
                tl = new TraceLogger("", "PolluxAstroPowerbox.Hardware");

                // DriverProgId has to be set here because it used by ReadProfile to get the TraceState flag.
                DriverProgId = Switch.DriverProgId; // Get this device's ProgID so that it can be used to read the Profile configuration values

                // ReadProfile has to go here before anything is written to the log because it loads the TraceLogger enable / disable state.
                ReadProfile(); // Read device configuration from the ASCOM Profile store, including the trace state

                LogMessage("SwitchHardware", $"Static initialiser completed.");
            }
            catch (Exception ex)
            {
                try { LogMessage("SwitchHardware", $"Initialisation exception: {ex}"); } catch { }
                MessageBox.Show($"{ex.Message}", "Exception creating ASCOM.PolluxAstroPowerbox.Switch", MessageBoxButtons.OK, MessageBoxIcon.Error);
                throw;
            }
        }

        /// <summary>
        /// Place device initialisation code here
        /// </summary>
        /// <remarks>Called every time a new instance of the driver is created.</remarks>
        internal static void InitialiseHardware()
        {
            // This method will be called every time a new ASCOM client loads your driver
            LogMessage("InitialiseHardware", $"Start.");

            // Make sure that "one off" activities are only undertaken once
            if (runOnce == false)
            {
                LogMessage("InitialiseHardware", $"Starting one-off initialisation.");

                DriverDescription = Switch.DriverDescription; // Get this device's Chooser description

                LogMessage("InitialiseHardware", $"ProgID: {DriverProgId}, Description: {DriverDescription}");

                connectedState = false; // Initialise connected to false
                utilities = new Util(); //Initialise ASCOM Utilities object
                astroUtilities = new AstroUtils(); // Initialise ASCOM Astronomy Utilities object

                LogMessage("InitialiseHardware", "Completed basic initialisation");

                // Add your own "one off" device initialisation here e.g. validating existence of hardware and setting up communications

                LogMessage("InitialiseHardware", $"One-off initialisation complete.");
                runOnce = true; // Set the flag to ensure that this code is not run again
            }
        }

        // PUBLIC COM INTERFACE ISwitchV2 IMPLEMENTATION

        #region Common properties and methods.

        /// <summary>
        /// Displays the Setup Dialogue form.
        /// If the user clicks the OK button to dismiss the form, then
        /// the new settings are saved, otherwise the old values are reloaded.
        /// THIS IS THE ONLY PLACE WHERE SHOWING USER INTERFACE IS ALLOWED!
        /// </summary>
        public static void SetupDialog()
        {
            // Don't permit the setup dialogue if already connected
            if ( Connected )
            {
                LogMessage("SetupDialog", "Already connected");
                MessageBox.Show("Already connected, just press OKaaayyyy");
            }

            using (SetupDialogForm F = new SetupDialogForm(tl))
            {
                var result = F.ShowDialog();
                if (result == DialogResult.OK)
                {
                    WriteProfile(); // Persist device configuration values to the ASCOM Profile store
                }
            }
        }

        /// <summary>Returns the list of custom action names supported by this driver.</summary>
        /// <value>An ArrayList of strings (SafeArray collection) containing the names of supported actions.</value>
        public static ArrayList SupportedActions
        {
            get
            {
                LogMessage("SupportedActions Get", "Returning empty ArrayList");
                return new ArrayList();
            }
        }

        /// <summary>Invokes the specified device-specific custom action.</summary>
        /// <param name="ActionName">A well known name agreed by interested parties that represents the action to be carried out.</param>
        /// <param name="ActionParameters">List of required parameters or an <see cref="String.Empty">Empty String</see> if none are required.</param>
        /// <returns>A string response. The meaning of returned strings is set by the driver author.
        /// <para>Suppose filter wheels start to appear with automatic wheel changers; new actions could be <c>QueryWheels</c> and <c>SelectWheel</c>. The former returning a formatted list
        /// of wheel names and the second taking a wheel name and making the change, returning appropriate values to indicate success or failure.</para>
        /// </returns>
        public static string Action(string actionName, string actionParameters)
        {
            LogMessage("Action", $"Action {actionName}, parameters {actionParameters} is not implemented");
            throw new ActionNotImplementedException("Action " + actionName + " is not implemented by this driver");
        }

        /// <summary>
        /// Transmits an arbitrary string to the device and does not wait for a response.
        /// Optionally, protocol framing characters may be added to the string before transmission.
        /// </summary>
        /// <param name="Command">The literal command string to be transmitted.</param>
        /// <param name="Raw">
        /// if set to <c>true</c> the string is transmitted 'as-is'.
        /// If set to <c>false</c> then protocol framing characters may be added prior to transmission.
        /// </param>
        public static void CommandBlind(string command, bool raw)
        {
            CheckConnected("CommandBlind");
            throw new MethodNotImplementedException($"CommandBlind - Command:{command}, Raw: {raw}.");
        }

        /// <summary>
        /// Transmits an arbitrary string to the device and waits for a boolean response.
        /// Optionally, protocol framing characters may be added to the string before transmission.
        /// </summary>
        /// <param name="Command">The literal command string to be transmitted.</param>
        /// <param name="Raw">
        /// if set to <c>true</c> the string is transmitted 'as-is'.
        /// If set to <c>false</c> then protocol framing characters may be added prior to transmission.
        /// </param>
        /// <returns>
        /// Returns the interpreted boolean response received from the device.
        /// </returns>
        public static bool CommandBool(string command, bool raw)
        {
            throw new MethodNotImplementedException($"CommandBool - Command:{command}, Raw: {raw}.");
        }

        /// <summary>
        /// Transmits an arbitrary string to the device and waits for a string response.
        /// Optionally, protocol framing characters may be added to the string before transmission.
        /// </summary>
        /// <param name="Command">The literal command string to be transmitted.</param>
        /// <param name="Raw">
        /// if set to <c>true</c> the string is transmitted 'as-is'.
        /// If set to <c>false</c> then protocol framing characters may be added prior to transmission.
        /// </param>
        /// <returns>
        /// Returns the string response received from the device.
        /// </returns>
        public static string CommandString(string command, bool raw)
        {
            tl.LogMessage("CommandString", "Command: " + command);

            byte[] cmd = cmdCrc(command);

            if (!raw)
            {
                cmd = cmd.Prepend((byte)'#').ToArray();
                cmd = cmd.Append((byte)'$').ToArray();
            }
            port.TransmitBinary(cmd);

            byte tmp = port.ReceiveByte();
            while ( tmp != 35 )
            {
                // Sometimes there's garbage on the line so read until the next '#'
                tmp = port.ReceiveByte();
                LogMessage("CommandString", "Garbage: " + (char)tmp);
            }
            LogMessage("CommandString", "Begin: " + (char)tmp);
            byte[] term = { (byte)'$' };
            byte[] rsp = port.ReceiveTerminatedBinary(term);
            foreach (byte b in rsp)
            {
                LogMessage("CommandString", "CHAR: " + (char)b);
            }
            // Strip last '$'
            rsp = rsp.Take(rsp.Length - 1).ToArray();

            if (!checkCrc(rsp))
            {
                throw new ASCOM.DriverException("Checksum error");
            }

            // Remove last '\0' + 2 checksum bytes
            rsp = rsp.Take(rsp.Length - 3).ToArray();
            string str = Encoding.Default.GetString(rsp);
            LogMessage("CommandString", "Response: " + str);
            return str;
        }

        /// <summary>
        /// Deterministically release both managed and unmanaged resources that are used by this class.
        /// </summary>
        /// <remarks>
        /// 
        /// Do not call this method from the Dispose method in your driver class.
        ///
        /// This is because this hardware class is decorated with the <see cref="HardwareClassAttribute"/> attribute and this Dispose() method will be called 
        /// automatically by the  local server executable when it is irretrievably shutting down. This gives you the opportunity to release managed and unmanaged 
        /// resources in a timely fashion and avoid any time delay between local server close down and garbage collection by the .NET runtime.
        ///
        /// For the same reason, do not call the SharedResources.Dispose() method from this method. Any resources used in the static shared resources class
        /// itself should be released in the SharedResources.Dispose() method as usual. The SharedResources.Dispose() method will be called automatically 
        /// by the local server just before it shuts down.
        /// 
        /// </remarks>
        public static void Dispose()
        {
            try { LogMessage("Dispose", $"Disposing of assets and closing down."); } catch { }

            try
            {
                // Clean up the trace logger and utility objects
                tl.Enabled = false;
                tl.Dispose();
                tl = null;
            }
            catch { }

            try
            {
                port.Connected = false;
                port.Dispose();
                port = null;
            }
            catch { }

            try
            {
                utilities.Dispose();
                utilities = null;
            }
            catch { }

            try
            {
                astroUtilities.Dispose();
                astroUtilities = null;
            }
            catch { }
            lastUpdate = DateTime.UtcNow.AddSeconds(-10);
            lastResponse = null;
            lastJson = null;
        }

        /// <summary>
        /// Set True to connect to the device hardware. Set False to disconnect from the device hardware.
        /// You can also read the property to check whether it is connected. This reports the current hardware state.
        /// </summary>
        /// <value><c>true</c> if connected to the hardware; otherwise, <c>false</c>.</value>
        public static bool Connected
        {
            get
            {
                connectedMutex.WaitOne();
                LogMessage("Connected", $"Get {connectedState}");
                bool val = connectedState;
                connectedMutex.ReleaseMutex();
                return val;
            }
            set
            {
                connectedMutex.WaitOne();
                try
                {
                    LogMessage("Connected", $"Set {value}");
                    if (value == connectedState)
                        return;

                    if (value)
                    {
                        LogMessage("Connected Set", $"Connecting to port {comPort}");
                        if (port != null)
                        {
                            port.Connected = false;
                            port.Dispose();
                        }
                        port = new ASCOM.Utilities.Serial();
                        port.PortName = comPort;
                        port.Speed = ASCOM.Utilities.SerialSpeed.ps115200;
                        port.ReceiveTimeoutMs = 2000;
                        port.Connected = true;
                        try
                        {
                            string rsp = CommandString("status", false);
                            checkDeviceError(rsp);
                        }
                        catch
                        {
                            LogMessage("Conencted set", "Caught error, setting connectedState to false");
                            connectedState = false;
                            LogMessage("Connected set", $"connectedState is now ${connectedState}");
                            port.Connected = false;
                            port.Dispose();
                            port = null;
                            throw;
                        }
                        connectedState = true;
                    }
                    else
                    {
                        LogMessage("Connected Set", $"Disconnecting from port {comPort}");
                        if ( port != null )
                        {
                            port.Connected = false;
                            port.Dispose();
                            port = null;
                        }
                        connectedState = false;
                    }
                } finally
                {
                    connectedMutex.ReleaseMutex();
                }
            }
        }

        /// <summary>
        /// Returns a description of the device, such as manufacturer and model number. Any ASCII characters may be used.
        /// </summary>
        /// <value>The description.</value>
        public static string Description
        {
            get
            {
                LogMessage("Description Get", DriverDescription);
                return DriverDescription;
            }
        }

        /// <summary>
        /// Descriptive and version information about this ASCOM driver.
        /// </summary>
        public static string DriverInfo
        {
            get
            {
                Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
                string driverInfo = $"Version: {version.Major}.{version.Minor}";
                LogMessage("DriverInfo Get", driverInfo);
                return driverInfo;
            }
        }

        /// <summary>
        /// A string containing only the major and minor version of the driver formatted as 'm.n'.
        /// </summary>
        public static string DriverVersion
        {
            get
            {
                Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
                string driverVersion = $"{version.Major}.{version.Minor}";
                LogMessage("DriverVersion Get", driverVersion);
                return driverVersion;
            }
        }

        /// <summary>
        /// The interface version number that this device supports.
        /// </summary>
        public static short InterfaceVersion
        {
            // set by the driver wizard
            get
            {
                LogMessage("InterfaceVersion Get", "2");
                return Convert.ToInt16("2");
            }
        }

        /// <summary>
        /// The short name of the driver, for display purposes
        /// </summary>
        public static string Name
        {
            get
            {
                string name = "Powerbox";
                LogMessage("Name Get", name);
                return name;
            }
        }

        #endregion

        #region ISwitchV2 Implementation

        private static short numSwitch = 25;

        /// <summary>
        /// The number of switches managed by this driver
        /// </summary>
        /// <returns>The number of devices managed by this driver.</returns>
        internal static short MaxSwitch
        {
            get
            {
                LogMessage("MaxSwitch Get", numSwitch.ToString());
                return numSwitch;
            }
        }

        /// <summary>
        /// Return the name of switch device n.
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>The name of the device</returns>
        internal static string GetSwitchName(short id)
        {
            Validate("GetSwitchName", id);
            switch (id)
            {
                case IDVoltage:
                    return "Voltage [V]";
                case IDEnvTemperature:
                    return "Temperature [°C]";
                case IDEnvHumidity:
                    return "Humidity [%]";
                case IDEnvPressure:
                    return "Pressure [mbar]";
                case IDEnvDewpoint:
                    return "Dewpoint [°C]";
                case IDEnvTemperatureOffset:
                    return "Environment temperature offset [°C]";
                case IDRailState:
                    return "12V rail state";
                case IDAdjState:
                    return "Adjustable output state";
                case IDAdjVoltage:
                    return "Adjustable Output Voltage [V]";
                case IDDH1Mode:
                    return "Dewheater 1 mode";
                case IDDH1DutyCycle:
                    return "Dewheater 1 dutycycle [%]";
                case IDDH1Temperature:
                    return "Dewheater 1 temperature [°C]";
                case IDDH1TemperatureOffset:
                    return "Dewheater 1 temperature offset [°C]";
                case IDDH1Fixed:
                    return "Dewheater 1 fixed dutycycle [%]";
                case IDDH1DewpointOffset:
                    return "Dewheater 1 dewpoint offset [°C]";
                case IDDH1AmbientOffset:
                    return "Dewheater 1 ambient offset [°C]";
                case IDDH1MidpointOffset:
                    return "Dewheater 1 midpoint offset [°C]";
                case IDDH2Mode:
                    return "Dewheater 2 mode";
                case IDDH2DutyCycle:
                    return "Dewheater 2 dutycycle [%]";
                case IDDH2Temperature:
                    return "Dewheater 2 temperature [°C]";
                case IDDH2TemperatureOffset:
                    return "Dewheater 2 temperature offset [°C]";
                case IDDH2Fixed:
                    return "Dewheater 2 fixed dutycycle [%]";
                case IDDH2DewpointOffset:
                    return "Dewheater 2 dewpoint offset [°C]";
                case IDDH2AmbientOffset:
                    return "Dewheater 2 ambient offset [°C]";
                case IDDH2MidpointOffset:
                    return "Dewheater 2 midpoint offset [°C]";
                default:
                    throw new NotImplementedException("This should never happen!");
            }
        }

        /// <summary>
        /// Set a switch device name to a specified value.
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <param name="name">The name of the device</param>
        internal static void SetSwitchName(short id, string name)
        {
            Validate("SetSwitchName", id);
            LogMessage("SetSwitchName", $"SetSwitchName({id}) = {name} - not implemented");
            throw new MethodNotImplementedException("SetSwitchName");
        }

        /// <summary>
        /// Gets the description of the specified switch device. This is to allow a fuller description of
        /// the device to be returned, for example for a tool tip.
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>
        /// String giving the device description.
        /// </returns>
        internal static string GetSwitchDescription(short id)
        {
            Validate("GetSwitchDescription", id);
            switch (id)
            {
                case IDVoltage:
                    return "Input voltage";
                case IDEnvTemperature:
                    return "Environment Temperature";
                case IDEnvHumidity:
                    return "Environment relative humidity";
                case IDEnvPressure:
                    return "Environment absolute pressure";
                case IDEnvDewpoint:
                    return "Environment dewpoint";
                case IDEnvTemperatureOffset:
                    return "Correction offset for the environment temperature probe";
                case IDRailState:
                    return "State of the 4x12V outputs";
                case IDAdjState:
                    return "State of the adjustable output";
                case IDAdjVoltage:
                    return "Output voltage of the adjustable output";
                case IDDH1Mode:
                    return "0: Fixed\n1: Dewpoint\n2: Ambient\n3: Midpoint\n4: Slave";
                case IDDH1DutyCycle:
                    return "0%: Off\n100%: Full power";
                case IDDH1Temperature:
                    return "Temperature of the dew cap belonging to dewheater 1";
                case IDDH1TemperatureOffset:
                    return "Correction offset for the temperature probe of dewheater 1";
                case IDDH1Fixed:
                    return "Dutycycle for the fixed mode of dewheater 1";
                case IDDH1DewpointOffset:
                    return "Offset for the dewpoint mode of dewheater 1";
                case IDDH1AmbientOffset:
                    return "Offset for the ambient mode of dewheater 1";
                case IDDH1MidpointOffset:
                    return "Offset for the midpoint mode of dewheater 1";
                case IDDH2Mode:
                    return "0: Fixed\n1: Dewpoint\n2: Ambient\n3: Midpoint\n4: Slave";
                case IDDH2DutyCycle:
                    return "0%: Off\n100%: Full power";
                case IDDH2Temperature:
                    return "Temperature of the dew cap belonging to dewheater 2";
                case IDDH2TemperatureOffset:
                    return "Correction offset for the temperature probe of dewheater 2";
                case IDDH2Fixed:
                    return "Dutycycle for the fixed mode of dewheater 2";
                case IDDH2DewpointOffset:
                    return "Offset for the dewpoint mode of dewheater 2";
                case IDDH2AmbientOffset:
                    return "Offset for the ambient mode of dewheater 2";
                case IDDH2MidpointOffset:
                    return "Offset for the midpoint mode of dewheater 2";
                default:
                    throw new NotImplementedException("This should never happen!");
            }
        }

        /// <summary>
        /// Reports if the specified switch device can be written to, default true.
        /// This is false if the device cannot be written to, for example a limit switch or a sensor.
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>
        /// <c>true</c> if the device can be written to, otherwise <c>false</c>.
        /// </returns>
        internal static bool CanWrite(short id)
        {
            Validate("CanWrite", id);
            switch (id)
            {
                case IDVoltage:
                    return false;
                case IDEnvTemperature:
                    return false;
                case IDEnvHumidity:
                    return false;
                case IDEnvPressure:
                    return false;
                case IDEnvDewpoint:
                    return false;
                case IDEnvTemperatureOffset:
                    return true;
                case IDRailState:
                    return true;
                case IDAdjState:
                    return true;
                case IDAdjVoltage:
                    return true;
                case IDDH1Mode:
                    return true;
                case IDDH1DutyCycle:
                    return false;
                case IDDH1Temperature:
                    return false;
                case IDDH1TemperatureOffset:
                    return true;
                case IDDH1Fixed:
                    return true;
                case IDDH1DewpointOffset:
                    return true;
                case IDDH1AmbientOffset:
                    return true;
                case IDDH1MidpointOffset:
                    return true;
                case IDDH2Mode:
                    return true;
                case IDDH2DutyCycle:
                    return false;
                case IDDH2Temperature:
                    return false;
                case IDDH2TemperatureOffset:
                    return true;
                case IDDH2Fixed:
                    return true;
                case IDDH2DewpointOffset:
                    return true;
                case IDDH2AmbientOffset:
                    return true;
                case IDDH2MidpointOffset:
                    return true;
                default:
                    throw new NotImplementedException("This should never happen!");
            }
        }

        #region Boolean switch members

        /// <summary>
        /// Return the state of switch device id as a boolean
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>True or false</returns>
        internal static bool GetSwitch(short id)
        {
            updateResponse();
            Validate("GetSwitch", id);
            switch (id)
            {
                case IDRailState:
                    return lastJson["R"].GetValue<bool>();
                case IDAdjState:
                    return lastJson["A"]["ON"].GetValue<bool>();
                default:
                    throw new NotImplementedException("This should never happen!");
            }
        }

        /// <summary>
        /// Sets a switch controller device to the specified state, true or false.
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <param name="state">The required control state</param>
        internal static void SetSwitch(short id, bool state)
        {
            Validate("SetSwitch", id);
            if (!CanWrite(id))
            {
                var str = $"SetSwitch({id}) - Cannot Write";
                LogMessage("SetSwitch", str);
                throw new MethodNotImplementedException(str);
            }
            switch (id)
            {
                case IDRailState:
                    CommandString("rail " + (state ? "on" : "off"), false);
                    break;
                case IDAdjState:
                    CommandString("adj " + (state ? "on" : "off"), false);
                    break;
                default:
                    throw new NotImplementedException("This should never happen!");
            }
            updateResponseNow();
        }

        #endregion

        #region Analogue members

        /// <summary>
        /// Returns the maximum value for this switch device, this must be greater than <see cref="MinSwitchValue"/>.
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>The maximum value to which this device can be set or which a read only sensor will return.</returns>
        internal static double MaxSwitchValue(short id)
        {
            Validate("MaxSwitchValue", id);
            switch (id)
            {
                case IDVoltage:
                    return 15.0;
                case IDEnvTemperature:
                    return 100.0;
                case IDEnvHumidity:
                    return 100.0;
                case IDEnvPressure:
                    return 2000.0;
                case IDEnvDewpoint:
                    return 100.0;
                case IDEnvTemperatureOffset:
                    return 10.0;
                case IDAdjVoltage:
                    return 12.0;
                case IDDH1Mode:
                    return 4;
                case IDDH1DutyCycle:
                    return 100.0;
                case IDDH1Temperature:
                    return 200.0;
                case IDDH1TemperatureOffset:
                    return 10.0;
                case IDDH1Fixed:
                    return 100.0;
                case IDDH1DewpointOffset:
                    return 10.0;
                case IDDH1AmbientOffset:
                    return 10.0;
                case IDDH1MidpointOffset:
                    return 10.0;
                case IDDH2Mode:
                    return 4;
                case IDDH2DutyCycle:
                    return 100.0;
                case IDDH2Temperature:
                    return 200.0;
                case IDDH2TemperatureOffset:
                    return 10.0;
                case IDDH2Fixed:
                    return 100.0;
                case IDDH2DewpointOffset:
                    return 10.0;
                case IDDH2AmbientOffset:
                    return 10.0;
                case IDDH2MidpointOffset:
                    return 10.0;
                default:
                    throw new NotImplementedException("This should never happen!");
            }
        }

        /// <summary>
        /// Returns the minimum value for this switch device, this must be less than <see cref="MaxSwitchValue"/>
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>The minimum value to which this device can be set or which a read only sensor will return.</returns>
        internal static double MinSwitchValue(short id)
        {
            Validate("MinSwitchValue", id);
            switch (id)
            {
                case IDVoltage:
                    return 0.0;
                case IDEnvTemperature:
                    return -100.0;
                case IDEnvHumidity:
                    return 0.0;
                case IDEnvPressure:
                    return 0.0;
                case IDEnvDewpoint:
                    return -100.0;
                case IDEnvTemperatureOffset:
                    return -10.0;
                case IDAdjVoltage:
                    return 5.0;
                case IDDH1Mode:
                    return 0.0;
                case IDDH1DutyCycle:
                    return 0.0;
                case IDDH1Temperature:
                    return -100.0;
                case IDDH1TemperatureOffset:
                    return -10.0;
                case IDDH1Fixed:
                    return 0.0;
                case IDDH1DewpointOffset:
                    return 0.0;
                case IDDH1AmbientOffset:
                    return 0.0;
                case IDDH1MidpointOffset:
                    return -10.0;
                case IDDH2Mode:
                    return 0.0;
                case IDDH2DutyCycle:
                    return 0.0;
                case IDDH2Temperature:
                    return -100.0;
                case IDDH2TemperatureOffset:
                    return -10.0;
                case IDDH2Fixed:
                    return 0.0;
                case IDDH2DewpointOffset:
                    return 0.0;
                case IDDH2AmbientOffset:
                    return 0.0;
                case IDDH2MidpointOffset:
                    return -10.0;
                default:
                    throw new NotImplementedException("This should never happen!");
            }
        }

        /// <summary>
        /// Returns the step size that this device supports (the difference between successive values of the device).
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>The step size for this device.</returns>
        internal static double SwitchStep(short id)
        {
            Validate("SwitchStep", id);
            switch (id)
            {
                case IDVoltage:
                    return 0.1;
                case IDEnvTemperature:
                    return 0.1;
                case IDEnvHumidity:
                    return 0.1;
                case IDEnvPressure:
                    return 0.1;
                case IDEnvDewpoint:
                    return 0.1;
                case IDEnvTemperatureOffset:
                    return 0.1;
                case IDAdjVoltage:
                    return 0.1;
                case IDDH1Mode:
                    return 1.0;
                case IDDH1DutyCycle:
                    return 0.1;
                case IDDH1Temperature:
                    return 0.1;
                case IDDH1TemperatureOffset:
                    return 0.1;
                case IDDH1Fixed:
                    return 1.0;
                case IDDH1DewpointOffset:
                    return 0.1;
                case IDDH1AmbientOffset:
                    return 0.1;
                case IDDH1MidpointOffset:
                    return 0.1;
                case IDDH2Mode:
                    return 1.0;
                case IDDH2DutyCycle:
                    return 0.1;
                case IDDH2Temperature:
                    return 0.1;
                case IDDH2TemperatureOffset:
                    return 0.1;
                case IDDH2Fixed:
                    return 1.0;
                case IDDH2DewpointOffset:
                    return 0.1;
                case IDDH2AmbientOffset:
                    return 0.1;
                case IDDH2MidpointOffset:
                    return 0.1;
                default:
                    throw new NotImplementedException("This should never happen!");
            }
        }

        /// <summary>
        /// Returns the value for switch device id as a double
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <returns>The value for this switch, this is expected to be between <see cref="MinSwitchValue"/> and
        /// <see cref="MaxSwitchValue"/>.</returns>
        internal static double GetSwitchValue(short id)
        {
            Validate("GetSwitchValue", id);
            updateResponse();
            switch (id)
            {
                case IDVoltage:
                    return Math.Round(lastJson["V"].GetValue<double>(),1);
                case IDEnvTemperature:
                    return Math.Round(lastJson["E"]["T"].GetValue<double>(), 1);
                case IDEnvHumidity:
                    return Math.Round(lastJson["E"]["H"].GetValue<double>(), 1);
                case IDEnvPressure:
                    return Math.Round(lastJson["E"]["P"].GetValue<double>(), 1);
                case IDEnvDewpoint:
                    return Math.Round(lastJson["E"]["D"].GetValue<double>(), 1);
                case IDEnvTemperatureOffset:
                    return lastJson["E"]["dT"].GetValue<double>();
                case IDAdjVoltage:
                    return lastJson["A"]["V"].GetValue<double>();
                case IDDH1Mode:
                    return lastJson["DH1"]["M"].GetValue<double>();
                case IDDH1DutyCycle:
                    return Math.Round(lastJson["DH1"]["DC"].GetValue<double>());
                case IDDH1Temperature:
                    if (lastJson["DH1"]["T"] == null)
                    {
                        return double.NaN;
                    }
                    return lastJson["DH1"]["T"].GetValue<double>();
                case IDDH1TemperatureOffset:
                    return lastJson["DH1"]["dT"].GetValue<double>();
                case IDDH1Fixed:
                    return Math.Round(lastJson["DH1"]["F"].GetValue<double>());
                case IDDH1DewpointOffset:
                    return lastJson["DH1"]["OD"].GetValue<double>();
                case IDDH1AmbientOffset:
                    return lastJson["DH1"]["OA"].GetValue<double>();
                case IDDH1MidpointOffset:
                    return lastJson["DH1"]["OM"].GetValue<double>();
                case IDDH2Mode:
                    return lastJson["DH2"]["M"].GetValue<double>();
                case IDDH2DutyCycle:
                    return Math.Round(lastJson["DH2"]["DC"].GetValue<double>());
                case IDDH2Temperature:
                    if (lastJson["DH2"]["T"] == null)
                    {
                        return double.NaN;
                    }
                    return lastJson["DH2"]["T"].GetValue<double>();
                case IDDH2TemperatureOffset:
                    return lastJson["DH2"]["dT"].GetValue<double>();
                case IDDH2Fixed:
                    return Math.Round(lastJson["DH2"]["F"].GetValue<double>());
                case IDDH2DewpointOffset:
                    return lastJson["DH2"]["OD"].GetValue<double>();
                case IDDH2AmbientOffset:
                    return lastJson["DH2"]["OA"].GetValue<double>();
                case IDDH2MidpointOffset:
                    return lastJson["DH2"]["OM"].GetValue<double>();
                default:
                    throw new NotImplementedException("This should never happen!");
            }
        }

        /// <summary>
        /// Set the value for this device as a double.
        /// </summary>
        /// <param name="id">The device number (0 to <see cref="MaxSwitch"/> - 1)</param>
        /// <param name="value">The value to be set, between <see cref="MinSwitchValue"/> and <see cref="MaxSwitchValue"/></param>
        internal static void SetSwitchValue(short id, double value)
        {
            Validate("SetSwitchValue", id, value);
            if (!CanWrite(id))
            {
                LogMessage("SetSwitchValue", $"SetSwitchValue({id}) - Cannot write");
                throw new ASCOM.MethodNotImplementedException($"SetSwitchValue({id}) - Cannot write");
            }
            switch (id)
            {
                case IDEnvTemperatureOffset:
                    CommandString("env offset " + (int)(value * 100), false);
                    break;
                case IDAdjVoltage:
                    CommandString("adj " + (int)(value * 100), false);
                    break;
                case IDDH1Mode:
                    CommandString("DH1 mode " + (int)value, false);
                    break;
                case IDDH1TemperatureOffset:
                    CommandString("DH1 offset " + (int)value * 100, false);
                    break;
                case IDDH1Fixed:
                    CommandString("DH1 fixed " + (int)value * 100, false);
                    break;
                case IDDH1DewpointOffset:
                    CommandString("DH1 oD " + (int)value * 100, false);
                    break;
                case IDDH1AmbientOffset:
                    CommandString("DH1 oA " + (int)value * 100, false);
                    break;
                case IDDH1MidpointOffset:
                    CommandString("DH1 oM " + (int)value * 100, false);
                    break;
                case IDDH2Mode:
                    CommandString("DH2 mode " + (int)value, false);
                    break;
                case IDDH2TemperatureOffset:
                    CommandString("DH2 offset " + (int)value * 100, false);
                    break;
                case IDDH2Fixed:
                    CommandString("DH2 fixed " + (int)value * 100, false);
                    break;
                case IDDH2DewpointOffset:
                    CommandString("DH2 oD " + (int)value * 100, false);
                    break;
                case IDDH2AmbientOffset:
                    CommandString("DH2 oA " + (int)value * 100, false);
                    break;
                case IDDH2MidpointOffset:
                    CommandString("DH2 oM " + (int)value * 100, false);
                    break;
                default:
                    throw new NotImplementedException("This should never happen!");
            }
            updateResponseNow();
        }

        #endregion

        #endregion

        #region Private methods

        /// <summary>
        /// Checks that the switch id is in range and throws an InvalidValueException if it isn't
        /// </summary>
        /// <param name="message">The message.</param>
        /// <param name="id">The id.</param>
        private static void Validate(string message, short id)
        {
            if (id < 0 || id >= numSwitch)
            {
                LogMessage(message, string.Format("Switch {0} not available, range is 0 to {1}", id, numSwitch - 1));
                throw new InvalidValueException(message, id.ToString(), string.Format("0 to {0}", numSwitch - 1));
            }
        }

        /// <summary>
        /// Checks that the switch id and value are in range and throws an
        /// InvalidValueException if they are not.
        /// </summary>
        /// <param name="message">The message.</param>
        /// <param name="id">The id.</param>
        /// <param name="value">The value.</param>
        private static void Validate(string message, short id, double value)
        {
            Validate(message, id);
            var min = MinSwitchValue(id);
            var max = MaxSwitchValue(id);
            if (value < min || value > max)
            {
                LogMessage(message, string.Format("Value {1} for Switch {0} is out of the allowed range {2} to {3}", id, value, min, max));
                throw new InvalidValueException(message, value.ToString(), string.Format("Switch({0}) range {1} to {2}", id, min, max));
            }
        }

        #endregion

        #region Private properties and methods
        // Useful methods that can be used as required to help with driver development

        /// <summary>
        /// Use this function to throw an exception if we aren't connected to the hardware
        /// </summary>
        /// <param name="message"></param>
        private static void CheckConnected(string message)
        {
            if (!Connected)
            {
                throw new NotConnectedException(message);
            }
        }

        /// <summary>
        /// Read the device configuration from the ASCOM Profile store
        /// </summary>
        internal static void ReadProfile()
        {
            using (Profile driverProfile = new Profile())
            {
                driverProfile.DeviceType = "Switch";
                tl.Enabled = Convert.ToBoolean(driverProfile.GetValue(DriverProgId, traceStateProfileName, string.Empty, traceStateDefault));
                comPort = driverProfile.GetValue(DriverProgId, comPortProfileName, string.Empty, comPortDefault);
            }
        }

        /// <summary>
        /// Write the device configuration to the  ASCOM  Profile store
        /// </summary>
        internal static void WriteProfile()
        {
            using (Profile driverProfile = new Profile())
            {
                driverProfile.DeviceType = "Switch";
                driverProfile.WriteValue(DriverProgId, traceStateProfileName, tl.Enabled.ToString());
                driverProfile.WriteValue(DriverProgId, comPortProfileName, comPort.ToString());
            }
        }

        /// <summary>
        /// Log helper function that takes identifier and message strings
        /// </summary>
        /// <param name="identifier"></param>
        /// <param name="message"></param>
        internal static void LogMessage(string identifier, string message)
        {
            tl.LogMessageCrLf(identifier, message);
        }

        /// <summary>
        /// Log helper function that takes formatted strings and arguments
        /// </summary>
        /// <param name="identifier"></param>
        /// <param name="message"></param>
        /// <param name="args"></param>
        internal static void LogMessage(string identifier, string message, params object[] args)
        {
            var msg = string.Format(message, args);
            LogMessage(identifier, msg);
        }
        #endregion
    }
}

