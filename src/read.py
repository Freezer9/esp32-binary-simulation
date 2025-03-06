#!/usr/bin/env python3
import serial
import struct
import argparse
import sys
import time
import threading
import queue
import os
import binascii

from rich.console import Console
from rich.table import Table
from rich.live import Live
from rich.panel import Panel
from rich.layout import Layout
from rich import box
from rich.text import Text

console = Console()

# Global variables for command input
command_input = ""
command_complete = ""
command_queue = queue.Queue()

# Global for debug messages
debug_messages = []
MAX_DEBUG_MESSAGES = 20  # Maximum number of debug messages to keep

# Telemetry data structure matching Arduino's TelemetryData
# Adjusted to match the exact Arduino struct definition
TELEMETRY_FORMAT = "<IBBIIIhhBffhhhh4shhI"  # Must match Arduino struct exactly
TELEMETRY_FIELDS = [
    "Packet Number", "Satellite Status", "Error Code", "Mission Time (s)",
    "Pressure 1 (Pa)", "Pressure 2 (Pa)", "Descent Rate (m/s)", "Temperature (°C)",
    "Battery Voltage (V)", "GPS Latitude (°)", "GPS Longitude (°)", "GPS Altitude (m)",
    "Pitch (°)", "Roll (°)", "Yaw (°)", "LNLN", "IoT S1 Data", "IoT S2 Data", "Team Number"
]

def parse_args():
    parser = argparse.ArgumentParser(description="Serial Telemetry Reader CLI")
    parser.add_argument("--port", default=None, help="Serial port (e.g., COM3)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--debug", action="store_true", help="Enable debug output")
    return parser.parse_args()

def list_serial_ports():
    """List available serial ports."""
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def select_port():
    """Prompt user to select a serial port if not provided."""
    ports = list_serial_ports()
    if not ports:
        console.print("[red]No serial ports available![/red]")
        sys.exit(1)
    
    console.print(Panel("Available Serial Ports", style="cyan", box=box.ROUNDED))
    for i, port in enumerate(ports, 1):
        console.print(f"[bold green]{i}[/bold green]: {port}")
    
    choice = input("Select a port number [1]: ") or "1"
    try:
        idx = int(choice) - 1
        if 0 <= idx < len(ports):
            return ports[idx]
        else:
            console.print("[red]Invalid selection![/red]")
            sys.exit(1)
    except ValueError:
        console.print("[red]Please enter a valid number![/red]")
        sys.exit(1)

def add_debug_message(message, level="info"):
    """Add a debug message to the global debug_messages list."""
    global debug_messages
    timestamp = time.strftime("%H:%M:%S", time.localtime())
    
    # Map level to color
    color_map = {
        "info": "blue",
        "success": "green",
        "warning": "yellow",
        "error": "red",
        "data": "dim"
    }
    color = color_map.get(level, "white")
    
    # Add message to list with timestamp
    debug_messages.append(f"[{color}][{timestamp}] {message}[/{color}]")
    
    # Keep only the last MAX_DEBUG_MESSAGES messages
    if len(debug_messages) > MAX_DEBUG_MESSAGES:
        debug_messages.pop(0)

# CRC8 calculation function to match Arduino's implementation
def calculate_crc8(data):
    """
    Calculate CRC8 checksum to match Arduino's CRC8 library.
    This is a simple implementation - you may need to adjust based on 
    the specific CRC8 parameters used in the Arduino code.
    """
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07  # Polynomial 0x07
            else:
                crc <<= 1
        crc &= 0xFF
    return crc

def translate_telemetry(data, debug=False):
    """Translate binary telemetry data into a dictionary."""
    try:
        if len(data) < 4:
            if debug:
                add_debug_message(f"Data too short: {len(data)} bytes", "warning")
            return None

        # Extract header (2 bytes), length (1 byte), and payload
        header_bytes = data[:2]
        header = struct.unpack("<H", header_bytes)[0]  # Little-endian unsigned short
        length = data[2]
        
        if debug:
            add_debug_message(f"Header: {hex(header)}, Length: {length}, Data len: {len(data)}", "info")
            add_debug_message(f"Binary data: {binascii.hexlify(data).decode()}", "data")
        
        # Check header and packet length
        if header != 0xABCD or length != len(data) - 4:
            if debug:
                add_debug_message(f"Invalid header ({hex(header)}!={hex(0xABCD)}) or length ({length}!={len(data)-4})", "warning")
            return None

        telemetry_data = data[3:-1]  # Exclude header, length, and CRC
        crc_received = data[-1]

        # Use CRC8 algorithm matching Arduino's implementation
        crc_calculated = calculate_crc8(telemetry_data)
        
        if crc_calculated != crc_received:
            if debug:
                add_debug_message(f"CRC mismatch: calculated={crc_calculated}, received={crc_received}", "warning")
                # If CRC fails consistently, you might want to disable this check temporarily for debugging
                # But let's try to understand why it's failing
                add_debug_message(f"Telemetry data length: {len(telemetry_data)}", "data")
            return None

        # Unpack the telemetry data according to the format
        values = struct.unpack(TELEMETRY_FORMAT, telemetry_data)
        
        # Convert LNLN bytes to string
        lnln = values[15].decode('ascii', errors='replace')
        
        # Process values according to the Arduino code's scaling
        adjusted_values = [
            values[0],                   # Packet Number
            values[1],                   # Satellite Status
            bin(values[2])[2:].zfill(8),  # Error Code as binary string
            values[3],                   # Mission Time
            values[4],                   # Pressure 1
            values[5],                   # Pressure 2
            values[6] / 100.0,           # Descent Rate (converted from int16_t * 100)
            values[7] / 100.0,           # Temperature (converted from int16_t * 100)
            values[8] / 10.0,            # Battery Voltage (converted from uint8_t * 10)
            values[9],                   # GPS Latitude
            values[10],                  # GPS Longitude
            values[11] / 10.0,           # GPS Altitude (converted from int16_t * 10)
            values[12] / 100.0,          # Pitch (converted from int16_t * 100)
            values[13] / 100.0,          # Roll (converted from int16_t * 100)
            values[14] / 100.0,          # Yaw (converted from int16_t * 100)
            lnln,                        # LNLN
            values[16] / 100.0,          # IoT S1 Data (converted from int16_t * 100)
            values[17] / 100.0,          # IoT S2 Data (converted from int16_t * 100)
            values[18]                   # Team Number
        ]
        
        if debug:
            # Print out binary representation of each field
            for i, field in enumerate(TELEMETRY_FIELDS):
                if i < len(values):
                    raw_value = values[i]
                    add_debug_message(f"Field {field}: raw={raw_value}", "data")
            
            add_debug_message("Successfully parsed telemetry packet", "success")
            
        return dict(zip(TELEMETRY_FIELDS, adjusted_values))
    except Exception as e:
        if debug:
            add_debug_message(f"Error unpacking telemetry: {e}", "error")
            import traceback
            error_trace = traceback.format_exc()
            for line in error_trace.split('\n'):
                if line.strip():
                    add_debug_message(line, "error")
        return None

def create_telemetry_table(data):
    """Create a rich table from telemetry data."""
    table = Table(title="Telemetry Data", box=box.SIMPLE, style="cyan")
    table.add_column("Field", style="bold magenta")
    table.add_column("Value", style="green")
    
    for field, value in data.items():
        # Format values nicely
        if isinstance(value, float):
            formatted_value = f"{value:.2f}"
        else:
            formatted_value = str(value)
            
        table.add_row(field, formatted_value)
    
    return table

def create_command_panel():
    """Create a panel to display the command input prompt and current command."""
    global command_input, command_complete
    
    return Panel(
        f"[bold yellow]Last Command:[/bold yellow] {command_complete}\n"
        f"[bold cyan]Current Input:[/bold cyan] {command_input}\n"
        f"Type command and press Enter (or 'q' to quit)",
        title="Command Input",
        style="yellow", 
        box=box.ROUNDED
    )

def create_connection_panel(port, connected=True, reconnecting=False):
    """Create a panel showing connection status."""
    if connected:
        status = "[bold green]Connected[/bold green]"
    elif reconnecting:
        status = "[bold yellow]Reconnecting...[/bold yellow]"
    else:
        status = "[bold red]Disconnected[/bold red]"
        
    return Panel(
        f"Port: {port}\nStatus: {status}",
        title="Connection Status",
        style="blue",
        box=box.ROUNDED
    )

def create_debug_panel():
    """Create a panel to display debug messages."""
    global debug_messages
    
    content = "\n".join(debug_messages)
    return Panel(
        Text.from_markup(content),
        title="Debug Information",
        style="blue", 
        box=box.ROUNDED
    )

def send_command(ser, command):
    """Send a command to the serial device."""
    try:
        team_number = "632419"  # Hardcoded to match Arduino
        full_command = f"CMD,{team_number},{command}\n"  # Add newline for Arduino readStringUntil('\n')
        ser.write(full_command.encode())
        add_debug_message(f"Sent: {full_command.strip()}", "success")
        return True
    except serial.SerialException:
        add_debug_message(f"Failed to send command: Device disconnected", "error")
        return False

def get_key():
    """Get a single keypress from the user without displaying it."""
    import tty
    import termios
    import sys
    
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def input_thread():
    """Thread to handle keyboard input."""
    global command_input, command_complete
    
    while True:
        try:
            # Get keypress
            key = get_key()
            
            # Process the key
            if key == '\r' or key == '\n':  # Enter key
                if command_input:
                    command_complete = command_input
                    command_queue.put(command_input)
                    command_input = ""
            elif key == '\x7f' or key == '\b':  # Backspace
                command_input = command_input[:-1]
            elif key == '\x03':  # Ctrl+C
                command_queue.put("q")
                break
            elif key == 'q' and not command_input:  # 'q' only quits if it's the first character
                command_queue.put("q")
                break
            elif key.isprintable():  # Only add printable characters
                command_input += key
        except Exception as e:
            add_debug_message(f"Input error: {e}", "error")
            break

def open_serial_connection(port, baudrate, debug=False):
    """Open a serial connection with error handling."""
    try:
        ser = serial.Serial(
            port, 
            baudrate, 
            timeout=1,
            rtscts=False,  # Disable hardware flow control
            dsrdtr=False   # Disable hardware flow control
        )
        if debug:
            add_debug_message(f"Connected to {port} at {baudrate} baud", "success")
        return ser, True
    except serial.SerialException as e:
        if debug:
            add_debug_message(f"Failed to connect: {e}", "error")
        return None, False

def is_port_available(port):
    """Check if the specified port is available."""
    import serial.tools.list_ports
    available_ports = [p.device for p in serial.tools.list_ports.comports()]
    return port in available_ports

def main():
    global command_input, command_complete
    
    args = parse_args()
    port = args.port if args.port else select_port()
    baudrate = args.baudrate
    debug = args.debug

    # Initial connection
    ser, connected = open_serial_connection(port, baudrate, debug)
    if not connected:
        console.print(f"[red]Failed to connect to {port}. Will attempt to reconnect when device is available.[/red]")
    else:
        console.print(f"[green]Connected to {port} at {baudrate} baud[/green]")

    console.print(Panel("Serial Telemetry Reader", title="Welcome", style="bold blue", box=box.DOUBLE))
    console.print("\nAvailable commands:")
    console.print("  TELEM,ON - Enable telemetry")
    console.print("  TELEM,OFF - Disable telemetry")
    console.print("  FLY - Start mission simulation")
    console.print("  CAL - Reset and calibrate")
    console.print("  q - Quit\n")

    # Set up the layout
    layout = Layout()
    
    # Create a different layout based on debug mode
    if debug:
        layout.split_column(
            Layout(name="top", ratio=3),
            Layout(name="middle", ratio=1),
            Layout(name="bottom", size=5)
        )
        layout["top"].split_row(
            Layout(name="telemetry", ratio=1),
            Layout(name="debug", ratio=1)
        )
        layout["middle"].update(create_connection_panel(port, connected))
        layout["bottom"].update(create_command_panel())
    else:
        layout.split_column(
            Layout(name="telemetry", ratio=3),
            Layout(name="connection", size=3),
            Layout(name="command", size=5)
        )
        layout["connection"].update(create_connection_panel(port, connected))

    # Start input thread
    input_thread_handle = threading.Thread(target=input_thread, daemon=True)
    input_thread_handle.start()

    buffer = bytearray()
    last_telemetry = None
    buffer_last_updated = time.time()  # Initialize outside the loop
    last_reconnect_attempt = 0
    reconnect_interval = 2.0  # seconds

    with Live(layout, console=console, refresh_per_second=10, transient=False) as live:
        while True:
            try:
                current_time = time.time()
                
                # Check connection status and attempt reconnect if needed
                if not connected:
                    if current_time - last_reconnect_attempt > reconnect_interval:
                        last_reconnect_attempt = current_time
                        
                        # Update connection panel to show reconnecting status
                        if debug:
                            layout["middle"].update(create_connection_panel(port, False, True))
                        else:
                            layout["connection"].update(create_connection_panel(port, False, True))
                        
                        # Check if port is available
                        if is_port_available(port):
                            ser, connected = open_serial_connection(port, baudrate, debug)
                            if connected:
                                add_debug_message(f"Reconnected to {port}", "success")
                                buffer = bytearray()  # Clear buffer on reconnect
                            else:
                                add_debug_message(f"Port {port} is available but connection failed", "warning")
                        else:
                            add_debug_message(f"Port {port} is not available", "warning")
                    
                    # Update connection panel
                    if debug:
                        layout["middle"].update(create_connection_panel(port, connected))
                    else:
                        layout["connection"].update(create_connection_panel(port, connected))
                
                # Process buffer timeout
                if connected and current_time - buffer_last_updated > 2.0 and len(buffer) > 0:
                    if debug:
                        add_debug_message(f"Clearing stale buffer: {binascii.hexlify(buffer).decode()}", "warning")
                    buffer = bytearray()
                
                # Update command panel
                if debug:
                    layout["bottom"].update(create_command_panel())
                    layout["debug"].update(create_debug_panel())
                else:
                    layout["command"].update(create_command_panel())
                
                # Process commands
                while not command_queue.empty():
                    cmd = command_queue.get()
                    if cmd.lower() == "q":
                        raise KeyboardInterrupt
                    elif connected:
                        if not send_command(ser, cmd):
                            connected = False
                    else:
                        add_debug_message("Cannot send command: Not connected", "warning")
                
                # Read serial data if connected
                if connected:
                    try:
                        if ser.in_waiting:
                            data = ser.read(ser.in_waiting)
                            buffer_last_updated = time.time()
                            if debug:
                                add_debug_message(f"Received {len(data)} bytes: {binascii.hexlify(data).decode()}", "data")
                            buffer.extend(data)
                            
                            # Process any complete packets in the buffer
                            while len(buffer) >= 4:
                                # Check for proper header (0xABCD in little-endian is CD AB)
                                if len(buffer) >= 2 and struct.unpack("<H", buffer[0:2])[0] == 0xABCD:
                                    if len(buffer) < 3:
                                        # Not enough data yet, need at least the length byte
                                        break
                                        
                                    length = buffer[2]
                                    packet_size = length + 4  # header (2) + length (1) + data (length) + CRC (1)
                                    
                                    if len(buffer) >= packet_size:
                                        packet = buffer[:packet_size]
                                        telemetry = translate_telemetry(packet, debug)
                                        
                                        if telemetry:
                                            last_telemetry = telemetry
                                            if debug:
                                                layout["telemetry"].update(create_telemetry_table(telemetry))
                                            else:
                                                layout["telemetry"].update(create_telemetry_table(telemetry))
                                        elif debug:
                                            add_debug_message("Failed to parse packet", "warning")
                                            
                                        buffer = buffer[packet_size:]
                                    else:
                                        # Not enough data for a complete packet
                                        break
                                else:
                                    # Invalid header, skip one byte
                                    if debug:
                                        add_debug_message(f"Discarding byte: {binascii.hexlify(buffer[0:1]).decode()}", "warning")
                                    buffer = buffer[1:]
                    except serial.SerialException as e:
                        add_debug_message(f"Serial error: {e}", "error")
                        connected = False
                        if ser:
                            try:
                                ser.close()
                            except:
                                pass
                
                # If we have telemetry data but no new data is coming in, keep displaying the last data
                if last_telemetry:
                    if debug:
                        layout["telemetry"].update(create_telemetry_table(last_telemetry))
                    else:
                        layout["telemetry"].update(create_telemetry_table(last_telemetry))
                else:
                    status = "Waiting for telemetry data..." if connected else "Not connected - waiting to reconnect..."
                    waiting_panel = Panel(status, style="yellow")
                    if debug:
                        layout["telemetry"].update(waiting_panel)
                    else:
                        layout["telemetry"].update(waiting_panel)
                
                time.sleep(0.05)
            except KeyboardInterrupt:
                break
            except Exception as e:
                if debug:
                    add_debug_message(f"Error: {e}", "error")
                    import traceback
                    error_trace = traceback.format_exc()
                    for line in error_trace.split('\n'):
                        if line.strip():
                            add_debug_message(line, "error")
                time.sleep(0.1)

    if ser and connected:
        ser.close()
    console.print("[green]Disconnected[/green]")

if __name__ == "__main__":
    # Check if running on Windows and adjust terminal settings
    if os.name == 'nt':
        import msvcrt
        # For Windows, we'd use a different method for the input thread
        def get_key():
            return msvcrt.getch().decode(errors='ignore')
    
    main()