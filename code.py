import os, socketpool, busio, wifi, board, analogio, struct
import time, digitalio, displayio, terminalio

from adafruit_display_text import label
from adafruit_httpserver import Server, Request, Response, Route, GET

def reverse_maskx(a):
    return (((a & 0x1)  << 7) | ((a & 0x2) << 5) |
         ((a & 0x4)  << 3) | ((a & 0x8)  << 1) |
         ((a & 0x10) >> 1) | ((a & 0x20) >> 3) |
         ((a & 0x40) >> 5) | ((a & 0x80) >> 7))

def reverse_mask8(x):
    x = ((x & 0x5555) << 1) | ((x & 0xAAAA) >> 1)
    x = ((x & 0x3333) << 2) | ((x & 0xCCCC) >> 2)
    x = ((x & 0x0F0F) << 4) | ((x & 0xF0F0) >> 4)
  #  x = ((x & 0x00FF) << 8) | ((x & 0xFF00) >> 8)
    return x & 0xff
    
def reverse_mask16(x):
    x = ((x & 0x5555) << 1) | ((x & 0xAAAA) >> 1)
    x = ((x & 0x3333) << 2) | ((x & 0xCCCC) >> 2)
    x = ((x & 0x0F0F) << 4) | ((x & 0xF0F0) >> 4)
    x = ((x & 0x00FF) << 8) | ((x & 0xFF00) >> 8)
    return x & 0xffff

def crc16n(data):
    poly = 0x8005  # generator polinom (normal form)

    reg = 0
    for octet in data:
        octet = reverse_mask8(octet)
        # reflect in
        for i in range(8):
            topbit = reg & 0x8000
            if octet & (0x80 >> i):
                topbit ^= 0x8000
            reg <<= 1
            if topbit:
                reg ^= poly
        reg &= 0xFFFF
    reg = reverse_mask16(reg)
        # reflect out
    return reg 

POLYNOMIAL=0x8005

def crc16(data):
  crc=00
  for c in data:
    crc=crc^(c<<8)
    for j in range(8): 
        crc=(crc<<1)^POLYNOMIAL if crc & 0x8000 else crc<<1
  return crc & 0xFFFF

def crc16v(data):
    crc = 0
    for c in data:
        # crc = crc << 8
        crc ^= c
        for j in range(0, 8):
            if (crc & 0x0001) > 0:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc = crc >> 1
    return crc & 0xFFFF

def crc16k(data):
    crc = 0
    for c in data:
        c = c << 8
        
        for j in range(8):
            if (crc ^ c) & 0x8000:
                crc = (crc << 1) ^ 0xA001
            else:
                crc = crc << 1
                c = c << 1
        
    return crc & 0xFFFF
    

class RS485:
    def __init__(self):
        self.uart = busio.UART(
            tx=board.IO27, 
            rx=board.IO33, 
      #      rs485_dir=board.IO33,
            baudrate=56000)
        self.depin = digitalio.DigitalInOut(board.IO26) # high send
        self.depin.direction = digitalio.Direction.OUTPUT
        self.repin = digitalio.DigitalInOut(board.IO25) # high send
        self.repin.direction = digitalio.Direction.OUTPUT

    def poll(self, msg):
        self.depin.value = True
        self.repin.value = True
        self.uart.write(msg)
        self.depin.value = False
        self.repin.value = False
        data = self.uart.read(self.uart.in_waiting)
        assert self.uart.in_waiting == 0
        return data


class RecBms:
    def __init__(self):
        self.rs485 = RS485()
        self.test()
        ADDRESS = 6
        self.lcd1 = self.serialize("LCD1", ADDRESS)
        #self.lcd1 = self.serialize("*IDN", ADDRESS)
        self.min_cell_voltage = 0.0
        self.max_cell_voltage = 0.0
        self.pack_voltage = 0.0
        self.state_of_charge = 0.0
        self.state_of_health = 0.0
        self.current = 0.0

    def serialize(self, command, address):
        out = bytearray([
            0x55, address, 0x00, 
            5, 0, 0, 0, 0, 
            ord('?'),
            0, 0,
            0xaa])
        out[4] = ord(command[0])
        out[5] = ord(command[1])
        out[6] = ord(command[2])
        out[7] = ord(command[3])
        crc = crc16v(out[1:9])
        out[9] = (crc & 0xff00) >> 8
        out[10] = crc & 0xff
        return out

    def test(self):
        assert self.serialize("LCD1", 0x1) == bytearray(
            [0x55, 0x01, 0x00, 0x05, 0x4C, 0x43, 
             0x44, 0x31, 0x3F, 0x46, 0xd0, 0xaa])
        print("CRC test passed")
        
    def query_lcd1(self):
        response = self.rs485.poll(self.lcd1)
        if not response:
            print("No response")
            return False
        if not response[-1] == 0xaa or len(response) < 37:
            print("Invalid response, length", len(response), response)
            return False
        marker = response[0]
        add0 = response[1]
        add1 = response[2]
        length = response[4]
        crcis = response[-3] << 8 | response[-2]
        print(add0, add1, length, response,  hex(crcis), len(response))
        print(hex(crc16v(response[1:-3])))
        response = response[5:]
        x = self.parse_float(response, 0 * 4)
        y = self.parse_float(response, 1 * 4)
        self.min_cell_voltage = self.parse_float(response, 2 * 4)
        self.max_cell_voltage = self.parse_float(response, 3 * 4)
        u = self.parse_float(response, 4 * 4)
        v = self.parse_float(response, 5 * 4)
        self.pack_voltage = self.parse_float(response, 6 * 4)
        self.state_of_charge = self.parse_float(response, 7 * 4)
        self.state_of_health = self.parse_float(response, 8 * 4)
        print(x, y, u, v)
        return True
        
    def parse_float(self, data, offset):
        result = struct.unpack('<f', data[offset:offset+4])[0]
        print(result)
        return result

    def render(self, label_voltage, label_min, label_max, 
               label_ampere, label_charge, label_health,
               httpserver):
        label_voltage.text =  f"{self.pack_voltage:>4.1f}V"
        label_min.text =  f"Min {self.min_cell_voltage:.2f}V"
        label_max.text =  f"Max {self.max_cell_voltage:.2f}V"
        label_charge.text = f"{self.state_of_charge:>4.0f}%"
        label_health.text = f"Health {self.state_of_health:0.0f}%"
        label_ampere.text = f"{self.current:>3.0f}A"
        httpserver.content = ("<html><title>LAK17 BMS</title>" +
            "<body style='font-size:50pt'>" +
            label_voltage.text + " " + label_charge.text + "<br>" +
            label_ampere.text + "<br>" +
            label_min.text + " " + label_max.text + "<br>" +
            label_health.text +
            "</body></html>")


class HttpServer:
    def __init__(self):
        self.pool = socketpool.SocketPool(wifi.radio)
        self.server = Server(self.pool, "/static", debug=True)
        self.server.start(str(wifi.radio.ipv4_address))
        self.content = "Hi!"
        
        self.server.add_routes([
            Route("/", GET, self.base)])
    
    def base(self, request: Request):
        return Response(request, self.content, content_type="text/html")

    def poll(self):
        self.server.poll()

class NetworkSwitcher:
    NETWORKS = {
        }

    def check_network(self):
        if wifi.radio.connected:
            return
            
        wifi.radio.stop_scanning_networks()
        networks = wifi.radio.start_scanning_networks()
        for network in networks:
            if network.ssid in self.NETWORKS:
                password = self.NETWORKS[network.ssid]
                wifi.radio.stop_scanning_networks()
                print("Trying to connect to", network.ssid)
                wifi.radio.connect(network.ssid, password)
                print("Connected to", network.ssid)
                return
        
        wifi.radio.stop_scanning_networks()

def render_status(label, query_success):
    out = str(int(time.monotonic() / 60)) + " min |"
    out += " " + str(wifi.radio.ipv4_address) + " |"
    out += f" {voltage:.2f}V"
    if query_success:
        out += " | OK" 
        label.color = 0xffffff
    else:
        out += " | Error" 
        label.color = 0xff0000
    label.text = out

wifi.radio.hostname = "recbms"

display = board.DISPLAY
font = terminalio.FONT

# Create the text label
label_pack_voltage = label.Label(font, y = 20, scale = 4)
label_charge = label.Label(font, y = 70, scale = 4)
x_col2 = 134
label_min_cell = label.Label(font, x = x_col2, y = 30, scale = 2)
label_max_cell = label.Label(font, x = x_col2, y = 10, scale = 2)
label_ampere = label.Label(font, x = x_col2 + 14, y = 70, scale = 4)
label_health = label.Label(font, x = 0, y = 110, scale = 1)

label_status = label.Label(font, x = 0, y = 130, scale = 1)

# Show it
root = displayio.Group()
display.root_group = root
root.append(label_pack_voltage)
root.append(label_charge)
root.append(label_ampere)
root.append(label_min_cell)
root.append(label_max_cell)
root.append(label_health)
root.append(label_status)

httpserver = HttpServer()
bms = RecBms()
adc = analogio.AnalogIn(board.BATTERY)
network_switcher = NetworkSwitcher()
while True:
    httpserver.poll()
    success = bms.query_lcd1()
    voltage = adc.value / 10000.0;

    render_status(label_status, success)
    bms.render(
        label_pack_voltage, label_min_cell, label_max_cell, 
        label_ampere, label_charge, label_health,
        httpserver)

    network_switcher.check_network()
    
    time.sleep(5)
    
