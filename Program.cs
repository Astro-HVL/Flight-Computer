using System;
using System.IO.Ports;
using System.Text;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.SignalR;
using Microsoft.Extensions.Hosting;
using System.Text.Json;

var builder = WebApplication.CreateBuilder(args);
builder.Services.AddSignalR();

var app = builder.Build();
app.UseDefaultFiles();
app.UseStaticFiles();
app.MapHub<TelemetryHub>("/telemetry");

var cts = new CancellationTokenSource();
var portName = Environment.GetEnvironmentVariable("TELEM_PORT") ?? (OperatingSystem.IsWindows() ? "COM5" : "/dev/ttyUSB0");
var baud = int.TryParse(Environment.GetEnvironmentVariable("TELEM_BAUD"), out var b) ? b : 115200;

var hub = app.Services.GetRequiredService<IHubContext<TelemetryHub>>();

// Only start the serial loop if the configured port actually exists on the system.
try
{
    bool portExists = false;
    if (OperatingSystem.IsWindows())
    {
        var available = SerialPort.GetPortNames();
        portExists = Array.Exists(available, p => string.Equals(p, portName, StringComparison.OrdinalIgnoreCase));
        if (!portExists)
        {
            Console.WriteLine($"Serial port '{portName}' not found. Available ports: {string.Join(',', available)}. Skipping serial loop.");
        }
    }
    else
    {
        // On Unix-like systems, check the device file
        portExists = System.IO.File.Exists(portName);
        if (!portExists)
        {
            Console.WriteLine($"Serial device '{portName}' not found. Skipping serial loop.");
        }
    }

    if (portExists)
    {
        _ = Task.Run(() => SerialLoop(portName, baud, hub, cts.Token));
        app.Lifetime.ApplicationStopping.Register(() => cts.Cancel());
    }
    else
    {
        // still register cancellation to be safe
        app.Lifetime.ApplicationStopping.Register(() => cts.Cancel());
    }
}
catch (Exception ex)
{
    Console.WriteLine("Error checking serial ports: " + ex.Message + ". Skipping serial loop.");
    app.Lifetime.ApplicationStopping.Register(() => cts.Cancel());
}
app.Run();

async Task SerialLoop(string port, int baudrate, IHubContext<TelemetryHub> hub, CancellationToken token)
{
    using var sp = new SerialPort(port, baudrate, Parity.None, 8, StopBits.One)
    {
        ReadTimeout = 1000,
        NewLine = "\n",
        Encoding = Encoding.ASCII
    };

    try
    {
        sp.Open();
        Console.WriteLine($"Opened serial {port} @ {baudrate}");
    }
    catch (Exception ex)
    {
        Console.WriteLine($"Failed to open serial {port}: {ex.Message}");
        return;
    }

    while (!token.IsCancellationRequested)
    {
        try
        {
            string line = sp.ReadLine();
            if (string.IsNullOrWhiteSpace(line)) continue;
            line = line.Trim();
            Console.WriteLine("RX RAW: " + line); // nyttig for debug i terminal

            object payload;

            // Hvis JSON
            if (line.StartsWith("{") && line.EndsWith("}"))
            {
                try
                {
                    var json = JsonSerializer.Deserialize<JsonElement>(line);
                    payload = new { type = "json", data = json };
                }
                catch
                {
                    payload = new { type = "raw", raw = line };
                }
            }
            else
            {
                // Forvent CSV: t,seq,ax,ay,az,pitch,roll,yaw,temp,vel,press,lat,lon,alt
                var parts = line.Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
                if (parts.Length >= 14
                    && double.TryParse(parts[2], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var ax)
                    && double.TryParse(parts[3], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var ay)
                    && double.TryParse(parts[4], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var az)
                    && double.TryParse(parts[5], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var pitch)
                    && double.TryParse(parts[6], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var roll)
                    && double.TryParse(parts[7], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var yaw)
                    && double.TryParse(parts[8], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var temp)
                    && double.TryParse(parts[9], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var vel)
                    && double.TryParse(parts[10], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var press)
                    && double.TryParse(parts[11], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var lat)
                    && double.TryParse(parts[12], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var lon)
                    && double.TryParse(parts[13], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var alt))
                {
                    // Her antar vi ax/ay/az allerede er i g. Hvis ikke: del på 9.80665
                    payload = new
                    {
                        type = "telemetry",
                        t = parts[0],
                        seq = parts[1],
                        ax = ax,
                        ay = ay,
                        az = az,
                        pitch = pitch,
                        roll = roll,
                        yaw = yaw,
                        temp = temp,
                        vel = vel,
                        press = press,
                        lat = lat,
                        lon = lon,
                        alt = alt
                    };
                }
                else
                {
                    payload = new { type = "raw", raw = line };
                }
            }

            // Broadcast
            await hub.Clients.All.SendAsync("telemetry", payload);
        }
        catch (TimeoutException) { /* ignore */ }
        catch (Exception ex)
        {
            Console.WriteLine("Serial read error: " + ex.Message);
            await Task.Delay(200, token);
        }
    }

    sp.Close();
}

public class TelemetryHub : Hub { }
