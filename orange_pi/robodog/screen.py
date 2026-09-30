"""Portrait 240x320 UI: bottom touch menu, ZE/RO, IMU, polar LiDAR.

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

WIDTH, HEIGHT, MENU_TOP = 240, 320, 270
NAV = (("ZERO", 0, 79), ("IMU", 80, 159), ("ЛИДАР", 160, 239))


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


def _stroke(draw, points, color, width=14):
    draw.line(points, fill=color, width=width, joint="curve")
    radius = width // 2
    for x, y in (points[:2], points[-2:]):
        draw.ellipse((x-radius, y-radius, x+radius, y+radius), fill=color)


def paint_wordmark(draw):
    """Font-independent block letters; fill the main 240x320 content area."""
    top, bottom = "#71e8f4", "#f0f7ff"
    _stroke(draw, (24, 58, 103, 58, 24, 142, 103, 142), top)
    _stroke(draw, (135, 58, 135, 142), top)
    for y in (58, 100, 142):
        _stroke(draw, (135, y, 216 if y != 100 else 204, y), top)
    _stroke(draw, (24, 168, 24, 252), bottom)
    _stroke(draw, (24, 168, 81, 168), bottom)
    draw.arc((55, 168, 112, 218), 270, 90, fill=bottom, width=14)
    _stroke(draw, (24, 215, 78, 215, 107, 252), bottom)
    draw.ellipse((137, 166, 216, 254), outline=bottom, width=14)


def paint(snapshot: Snapshot, page: int = 0, now: Optional[float] = None) -> Image.Image:
    now = time.monotonic() if now is None else now
    page = max(0, min(2, page))
    im = Image.new("RGB", (WIDTH, HEIGHT), "#0c1322")
    d = ImageDraw.Draw(im)
    d.rounded_rectangle((6, 5, 233, 31), radius=9, fill="#17263b")
    text(d, (13, 8), "ROBODOG", "#d8e9ff", 13)
    live = snapshot.telemetry and now-snapshot.telemetry_at < 1.0
    if live:
        t = snapshot.telemetry
        voltage = "{:.1f} V".format(t.battery_mv/1000)
        text(d, (163, 8), voltage, color_for_state(t.state), 13)
        d.ellipse((148, 14, 155, 21), fill=color_for_state(t.state))
    else:
        text(d, (157, 8), "NO MEGA", "#ff6481", 11)

    if page == 0:
        paint_wordmark(d)
    elif page == 1:
        valid = snapshot.imu and now-snapshot.imu_at < .7 and (snapshot.imu.flags & 2)
        d.rounded_rectangle((9, 41, 231, 260), radius=12, fill="#17263b")
        if valid:
            roll, pitch = snapshot.imu.roll, snapshot.imu.pitch
            text(d, (20, 47), "IMU MPU6050", "#77ddeb", 16)
            text(d, (20, 78), "Крен {:+d}°".format(roll), "#ffffff", 20)
            text(d, (20, 111), "Тангаж {:+d}°".format(pitch), "#ffffff", 18)
            text(d, (20, 239), "Темп. {}°C".format(snapshot.imu.temperature), "#9bc1d8", 12)
            cx, cy, radius = 120, 186, 43
            d.ellipse((cx-radius,cy-radius,cx+radius,cy+radius), outline="#456982", width=2)
            angle=math.radians(-roll)
            dx,dy=math.cos(angle)*37,math.sin(angle)*37
            d.line((cx-dx,cy-dy,cx+dx,cy+dy), fill="#5fe5db", width=4)
            d.ellipse((cx-4,cy-4,cx+4,cy+4), fill="#ffcf6e")
        else:
            text(d, (18, 92), "Нет свежих данных IMU", "#ffb066", 15)
            text(d, (18, 129), "Проверьте кадры I", "#9bc1d8", 12)
            text(d, (18, 150), "от Mega 2560", "#9bc1d8", 12)
    else:
        valid = snapshot.lidar and now-snapshot.lidar_at < 1.0
        d.rounded_rectangle((9, 41, 231, 260), radius=12, fill="#17263b")
        cx, cy, radius = 120, 159, 77
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
            text(d, (17, 47), snapshot.lidar_label[:28], "#77ddeb", 10)
        else:
            text(d, (19, 242), "Нет подтверждённого скана", "#ffb066", 12)

    d.rectangle((0, MENU_TOP, WIDTH-1, HEIGHT-1), fill="#15243a")
    d.line((0, MENU_TOP, WIDTH-1, MENU_TOP), fill="#41627b", width=2)
    for index, (label,left,right) in enumerate(NAV):
        d.rounded_rectangle((left+3, 277, right-3, 313), radius=9,
                            fill="#286779" if index == page else "#21384d")
        try:
            box=d.textbbox((0, 0), label, font=font(13))
        except UnicodeEncodeError:
            label="LIDAR"
            box=d.textbbox((0, 0), label, font=font(13))
        text(d, ((left+right-(box[2]-box[0]))//2, 287), label, "#ffffff", 13)
    return im


def page_from_touch(x: int,y: int) -> Optional[int]:
    if 0 <= x < WIDTH and MENU_TOP <= y < HEIGHT:
        return x//80
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
        if (cfg.get("width", WIDTH), cfg.get("height", HEIGHT)) != (WIDTH, HEIGHT):
            raise ValueError("screen must be configured for portrait 240x320")
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
        madctl=cfg.get("madctl")
        self.command(0x36,[int(madctl if madctl is not None else
                               (0x48 if kind=="ili9341" else 0x00))])
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
