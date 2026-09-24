# God Command

```
stty -F /dev/ttyACM0 9600 raw -echo && timeout 3s cat /dev/ttyACM0

stty -F /dev/ttyACM0 115200 raw -echo && timeout 3s cat /dev/ttyACM0
```

```
ls -l /dev/serial/by-id/
```

Check lidar

```
python3 - <<'PY'
import serial
p = serial.Serial(
    "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
    115200, timeout=2
)
p.reset_input_buffer()
p.write(bytes((0xA5, 0x50)))
p.flush()
header = p.read(7)
data = p.read(20) if len(header) == 7 else b""
print("descriptor:", header.hex(" "))
print("payload bytes:", len(data))
print("model/fw/hw:", data[:4].hex(" ") if len(data) >= 4 else "no response")
p.close()
PY
```

```
descriptor: a5 5a 14 00 00 00 04
payload bytes: 20
model/fw/hw: 18 1d 01 07
```

ssh -N -L 8765:127.0.0.1:8765 henlowworld@172.30.81.49
