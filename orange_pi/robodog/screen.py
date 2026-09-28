"""320x240 UI: bottom touch menu, face, IMU, polar LiDAR.

Works as a PNG preview without hardware. Hardware backend requires explicit
SPI devices and GPIO character-device line offsets, measured on the board.
"""
import math
import os
import time
from pathlib import Path
from typing import Optional, Tuple
from PIL import Image, ImageDraw, ImageFont
from .state import Snapshot

WIDTH, HEIGHT, MENU_TOP = 320, 240, 190
NAV = (("ЛИЦО", 0, 106), ("IMU", 107, 212), ("ЛИДАР", 213, 319))
NAV_ASCII = (("FACE", 0, 106), ("IMU", 107, 212), ("LIDAR", 213, 319))


def font(size: int):
    for name in ("DejaVuSans.ttf", "/usr/share/fonts/dejavu/DejaVuSans.ttf",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def text(draw, xy, words, color, size=16):
    try:
        draw.text(xy, words, fill=color, font=font(size))
    except UnicodeEncodeError:
        draw.text(xy, words.encode("ascii", "replace").decode("ascii"),
                  fill=color, font=ImageFont.load_default())


def color_for_state(state: int) -> str:
    return {0: "#59b6ff", 1: "#4ee6a3", 2: "#4ee6a3",
            3: "#ffd36a", 4: "#ff6481", 5: "#ffb066"}.get(state, "#ff6481")


def paint(snapshot: Snapshot, page: int = 0, now: Optional[float] = None) -> Image.Image:
    now = time.monotonic() if now is None else now
    page = max(0, min(2, page))
    im = Image.new("RGB", (WIDTH, HEIGHT), "#0c1322")
    d = ImageDraw.Draw(im)
    d.rounded_rectangle((6, 5, 313, 31), radius=9, fill="#17263b")
    text(d, (15, 8), "ROBODOG", "#d8e9ff", 14)
    live = snapshot.telemetry and now-snapshot.telemetry_at < 1.0
    if live:
        t = snapshot.telemetry
        voltage = "{:.1f} V".format(t.battery_mv/1000)
        text(d, (236, 8), voltage, color_for_state(t.state), 14)
        d.ellipse((216, 14, 224, 22), fill=color_for_state(t.state))
    else:
        text(d, (221, 8), "NO MEGA", "#ff6481", 13)

    if page == 0:
        blink = (now % 4.2) > 4.0
        d.rounded_rectangle((28, 42, 292, 178), radius=36, fill="#172a40", outline="#2b5676", width=2)
        if blink:
            for x in (107, 212):
                d.arc((x-26, 77, x+26, 91), 0, 180, fill="#6cf3e0", width=5)
        else:
            for x in (107, 212):
                d.ellipse((x-24, 68, x+24, 116), fill="#5fe5db")
                d.ellipse((x-13, 70, x+13, 111), fill="#092535")
                d.ellipse((x-8, 74, x-1, 81), fill="#ffffff")
        d.arc((104, 101, 215, 165), 15, 165, fill="#ffbbca", width=5)
        d.ellipse((51, 122, 72, 131), fill="#c76b89")
        d.ellipse((248, 122, 269, 131), fill="#c76b89")
        footer = "Пульт: " + ("есть" if now-snapshot.control_at < .3 else "нет")
        text(d, (72, 164), footer, "#8fb4c9", 11)
    elif page == 1:
        valid = snapshot.imu and now-snapshot.imu_at < .7 and (snapshot.imu.flags & 2)
        d.rounded_rectangle((10, 40, 310, 182), radius=12, fill="#17263b")
        if valid:
            roll, pitch = snapshot.imu.roll, snapshot.imu.pitch
            text(d, (20, 47), "IMU MPU6050", "#77ddeb", 16)
            text(d, (18, 76), "Крен {:+d}°".format(roll), "#ffffff", 21)
            text(d, (18, 111), "Тангаж {:+d}°".format(pitch), "#ffffff", 19)
            text(d, (18, 151), "Темп. {}°C".format(snapshot.imu.temperature), "#9bc1d8", 12)
            cx, cy, radius = 246, 110, 48
            d.ellipse((cx-radius,cy-radius,cx+radius,cy+radius), outline="#456982", width=2)
            angle=math.radians(-roll)
            dx,dy=math.cos(angle)*40,math.sin(angle)*40
            d.line((cx-dx,cy-dy,cx+dx,cy+dy), fill="#5fe5db", width=4)
            d.ellipse((cx-4,cy-4,cx+4,cy+4), fill="#ffcf6e")
        else:
            text(d, (24, 82), "Нет свежих данных IMU", "#ffb066", 17)
            text(d, (24, 115), "Проверьте кадры I от Mega", "#9bc1d8", 11)
    else:
        valid = snapshot.lidar and now-snapshot.lidar_at < 1.0
        d.rounded_rectangle((10, 40, 310, 182), radius=12, fill="#17263b")
        cx, cy, radius = 160, 111, 61
        for ring in (radius//3,2*radius//3,radius):
            d.ellipse((cx-ring,cy-ring,cx+ring,cy+ring), outline="#31506a")
        d.line((cx-radius,cy,cx+radius,cy), fill="#31506a")
        d.line((cx,cy-radius,cx,cy+radius), fill="#31506a")
        d.polygon(((cx,cy-7),(cx-5,cy+5),(cx+5,cy+5)), fill="#ffcf6e")
        if valid:
            for degrees, meters in snapshot.lidar[:800]:
                if not (.05 <= meters <= 12 and 0 <= degrees < 360):
                    continue
                angle = math.radians(degrees)
                r = min(radius-2, max(2, meters/6.0*radius))
                px = cx + math.sin(angle)*r
                py = cy - math.cos(angle)*r
                d.ellipse((px-1,py-1,px+1,py+1),fill="#5fe5db")
            text(d, (17, 46), snapshot.lidar_label, "#77ddeb", 11)
        else:
            text(d, (14, 158), "Нет подтверждённого скана", "#ffb066", 13)

    d.rectangle((0, MENU_TOP, 319, 239), fill="#15243a")
    d.line((0,MENU_TOP,319,MENU_TOP), fill="#41627b", width=2)
    labels=NAV
    for index, (label,left,right) in enumerate(labels):
        if index==page:
            d.rounded_rectangle((left+4,194,right-4,235), radius=9, fill="#286779")
        else:
            d.rounded_rectangle((left+4,194,right-4,235), radius=9, fill="#21384d")
        text(d, (left+16,205), label, "#ffffff", 15)
    return im


def page_from_touch(x: int,y: int) -> Optional[int]:
    if 0 <= x < WIDTH and MENU_TOP <= y < HEIGHT:
        return min(x//107,2)
    return None


def rgb565(image: Image.Image) -> bytes:
    """Big-endian RGB565 pixel stream expected by ST7789/ILI9341."""
    pixels=image.convert("RGB").getdata()
    result=bytearray(len(pixels)*2)
    j=0
    for r,g,b in pixels:
        value=((r&0xF8)<<8)|((g&0xFC)<<3)|(b>>3)
        result[j]=value>>8; result[j+1]=value&255; j+=2
    return bytes(result)


class SpiScreen:
    def __init__(self, cfg):
        keys=("spi_display","spi_touch","gpiochip","dc_offset","reset_offset","touch_irq_offset")
        if any(cfg.get(key) is None for key in keys):
            raise ValueError("screen SPI devices/GPIO offsets must be measured and configured")
        if cfg["spi_display"]==cfg["spi_touch"]:
            raise ValueError("display and touch need distinct chip-select devices")
        kind=cfg.get("controller")
        if kind not in ("st7789","ili9341"):
            raise ValueError("set screen.controller to verified st7789 or ili9341")
        from periphery import GPIO, SPI
        self.display=SPI(cfg["spi_display"],0,16000000)
        self.touch=SPI(cfg["spi_touch"],0,1000000)
        chip=cfg["gpiochip"]
        self.dc=GPIO(chip,int(cfg["dc_offset"]),"out")
        self.reset=GPIO(chip,int(cfg["reset_offset"]),"out")
        self.irq=GPIO(chip,int(cfg["touch_irq_offset"]),"in")
        self.cfg=cfg
        self.reset.write(False); time.sleep(.05); self.reset.write(True); time.sleep(.15)
        self.command(0x01); time.sleep(.15)
        self.command(0x11); time.sleep(.15)
        if kind=="ili9341":
            # Common ILI9341 panel power settings; visual test selects controller.
            for opcode, params in ((0xCF,[0x00,0xC1,0x30]),(0xED,[0x64,0x03,0x12,0x81]),
                                   (0xE8,[0x85,0x00,0x78]),(0xCB,[0x39,0x2C,0x00,0x34,0x02]),
                                   (0xF7,[0x20]),(0xEA,[0x00,0x00]),(0xC0,[0x23]),
                                   (0xC1,[0x10]),(0xC5,[0x3E,0x28]),(0xC7,[0x86]),
                                   (0xB1,[0x00,0x18]),(0xB6,[0x08,0x82,0x27])):
                self.command(opcode,params)
        self.command(0x3A,[0x55])
        self.command(0x36,[int(cfg.get("madctl",0x68 if kind=="ili9341" else 0x60))])
        self.command(0x29); time.sleep(.1)

    def command(self, opcode, params=()):
        self.dc.write(False); self.display.transfer([opcode])
        if params:
            self.dc.write(True); self.display.transfer(list(params))

    def show(self,image:Image.Image):
        x=int(self.cfg.get("x_offset",0)); y=int(self.cfg.get("y_offset",0))
        self.command(0x2A,[x>>8,x&255,(x+WIDTH-1)>>8,(x+WIDTH-1)&255])
        self.command(0x2B,[y>>8,y&255,(y+HEIGHT-1)>>8,(y+HEIGHT-1)&255])
        self.command(0x2C)
        data=rgb565(image)
        self.dc.write(True)
        for start in range(0,len(data),4096):
            self.display.transfer(list(data[start:start+4096]))

    def touch_raw(self) -> Optional[Tuple[int,int]]:
        if self.irq.read():
            return None
        def adc(command):
            values=self.touch.transfer([command,0,0])
            return ((values[1]<<8)|values[2])>>3 & 0xFFF
        xs=sorted(adc(0xD0) for _ in range(5))
        ys=sorted(adc(0x90) for _ in range(5))
        xraw,yraw=xs[2],ys[2]
        return xraw,yraw

    def touch_point(self) -> Optional[Tuple[int,int]]:
        raw=self.touch_raw()
        if raw is None: return None
        xraw,yraw=raw
        if self.cfg.get("touch_swap_xy"):
            xraw,yraw=yraw,xraw
        xmin,xmax=self.cfg["touch_x_min"],self.cfg["touch_x_max"]
        ymin,ymax=self.cfg["touch_y_min"],self.cfg["touch_y_max"]
        if xmax<=xmin or ymax<=ymin:
            raise ValueError("invalid touch calibration")
        x=(xraw-xmin)*(WIDTH-1)//(xmax-xmin)
        y=(yraw-ymin)*(HEIGHT-1)//(ymax-ymin)
        if self.cfg.get("touch_invert_x"): x=WIDTH-1-x
        if self.cfg.get("touch_invert_y"): y=HEIGHT-1-y
        if not (0<=x<WIDTH and 0<=y<HEIGHT): return None
        return x,y

    def close(self):
        for device in (self.irq,self.reset,self.dc,self.touch,self.display):
            device.close()
