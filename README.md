# ESP32-H2 Zigbee IR Blaster

This project turns an **ESP32-H2** into a bidirectional **Zigbee IR Blaster** using the **ESP-IDF** framework. It can learn IR codes (Receive) and transmit them (Send), integrated into Home Assistant or other Zigbee coordinators via the standard **On/Off Cluster** with a custom data attribute.

## Features

*   **IR Receive (Learning)**: Captures IR signals via a receiver (VS1838B) and reports the raw timing data to the coordinator.
*   **IR Transmit (Blasting)**: Transmits raw IR timing strings received from the coordinator via an IR LED.
*   **Zigbee Integration**: Acts as a Zigbee End Device (Endpoint 1).
    *   **Cluster**: `On/Off` (0x0006).
    *   **Custom Attribute**: `0xFF00` (Character String) used for bidirectional IR data transfer.
*   **Factory Reset**: Hold the **BOOT** button (GPIO 9) for 3 seconds to reset the device and leave the network.

## Hardware Requirements

1.  **ESP32-H2 Development Board** (e.g., ESP32-H2-DevKitM-1).
2.  **IR Receiver**: VS1838B or compatible (38kHz).
3.  **IR LED**: 940nm IR Emitter.
4.  **NPN Transistor**: 2N2222, BC547, or similar (to drive the IR LED).
5.  **Resistors**:
    *   ~1kΩ (Base of transistor).
    *   ~10-100Ω (Series with IR LED, depends on voltage and desired current).

### Wiring

| Component | Pin Name | ESP32-H2 Pin | Notes |
| :--- | :--- | :--- | :--- |
| **IR Receiver (VS1838B)** | Signal | **GPIO 11** | |
| | VCC | 3.3V | |
| | GND | GND | |
| **IR Transmitter** | Control | **GPIO 10** | Connect to Transistor Base via 1kΩ Resistor |
| **Button** | Boot | **GPIO 9** | Built-in BOOT button on most DevKits |

#### Transistor Circuit for IR LED
To enable good range, do not drive the IR LED directly from the GPIO. Use a transistor:
1.  **GPIO 11** -> 1kΩ Resistor -> **Transistor Base**.
2.  **Transistor Emitter** -> **GND**.
3.  **3.3V (or 5V)** -> Current Limiting Resistor -> **IR LED Anode (+)**.
4.  **IR LED Cathode (-)** -> **Transistor Collector**.

## Software Requirements

*   **ESP-IDF v5.1+** (ESP32-H2 support required).
*   **Espressif Zigbee SDK** (handled by `idf_component.yml`).

## Build and Flash

1.  **Set up the environment**:
    ```bash
    . $HOME/esp/esp-idf/export.sh
    ```

2.  **Set the target**:
    ```bash
    idf.py set-target esp32h2
    ```

3.  **Build**:
    ```bash
    idf.py build
    ```

4.  **Flash and Monitor**:
    ```bash
    idf.py flash monitor
    ```

## Home Assistant Integration

The device exposes itself as a standard Zigbee device with an **On/Off Cluster**. The IR data is handled via a custom attribute.

### Receiving IR Codes (Learning)
1.  Point a remote at the VS1838B receiver and press a button.
2.  The ESP32-H2 will capture the timing, encode it as a string (e.g., `300,-300,500,-500`), and update the Zigbee attribute.
3.  In Home Assistant, this appears as an attribute update. You can verify this in the "Manage Zigbee Device" -> "Cluster Attributes" or by listening to `zha_event`.

### Sending IR Codes
To send a code, use the `zha.set_zigbee_cluster_attribute` service to write the raw timing string to the device.

**Example Service Call (YAML):**
```yaml
service: zha.set_zigbee_cluster_attribute
data:
  ieee: <YOUR_DEVICE_IEEE_ADDRESS>
  endpoint_id: 1
  cluster_id: 6  # On/Off Cluster
  cluster_type: in
  attribute: 0xff00  # Custom Attribute
  value: "9000,-4500,560,-1690,560,-1690,..." # The string you learned previously
```

## How It Works

*   **IR RX**: Uses the RMT (Remote Control Peripheral) driver to capture pulse durations. It formats them into a CSV string where positive numbers are pulses (marks) and negative numbers are gaps (spaces) in microseconds.
*   **IR TX**: Parses the CSV string back into RMT items and transmits them using a 38kHz carrier frequency.
*   **Zigbee**: Uses a custom string attribute (`0xFF00`) on the standard On/Off cluster to bypass the complexity of the specific IR blasting clusters, allowing for raw raw data transfer which is more flexible for "learning" remotes.
