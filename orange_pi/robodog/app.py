import argparse
import json
import logging
import math
import socket
import sys
import threading
import time
from pathlib import Path
from .gateway import Gateway
from .lidar import capture_raw, follow_coin_d6, follow_jsonl
from .protocol import Control, Imu, Telemetry
from .screen import SpiScreen, paint, page_from_touch
from .state import Store


def demo_update(store: Store,now: float):
    points=[(a,2.7+1.2*math.sin(math.radians(a*3)+now*.3)) for a in range(0,360,3)]
    store.update(telemetry=Telemetry(1,1,25800,0x000F),telemetry_at=now,
                 imu=Imu(int(14*math.sin(now/2)),int(8*math.sin(now/3)),29,0x0002),imu_at=now,
                 control=Control(1,0,7400,2,0),control_at=now,
                 lidar=points,lidar_at=now,lidar_label="ДЕМО: искусственный скан",
                 mega_status="ДЕМО",remote_status="ДЕМО",lidar_status="ДЕМО")


def run(config:dict,demo:bool=False,snapshot:bool=False,page:int=0):
    store=Store()
    gateway=None
    lidar_thread=None
    stop=threading.Event()
    screen=None
    backend=config.get("ui_backend","png")
    screen_config=config.get("screen",{})
    if backend=="spi": screen=SpiScreen(screen_config)
    elif backend!="png": raise ValueError("ui_backend must be png or spi")
    if not demo and not snapshot:
        gateway=Gateway(config,store); gateway.start()
        lidar=config.get("lidar",{})
        if lidar.get("source")=="jsonl":
            if not lidar.get("jsonl_path"): raise ValueError("lidar.jsonl_path is required")
            lidar_thread=threading.Thread(target=follow_jsonl,
                args=(lidar["jsonl_path"],store,stop),daemon=True)
            lidar_thread.start()
        elif lidar.get("source")=="coin_d6":
            if not lidar.get("usb_port"): raise ValueError("lidar.usb_port is required")
            lidar_thread=threading.Thread(target=follow_coin_d6,
                args=(lidar["usb_port"],int(lidar.get("baud",230400)),store,stop,
                      bool(lidar.get("send_start_command",True))),daemon=True)
            lidar_thread.start()
    try:
        previous_pressed=False
        while True:
            now=time.monotonic()
            if demo: demo_update(store,now)
            image=paint(store.snapshot(),page,now)
            if screen:
                screen.show(image)
                point=screen.touch_point()
                if point and not previous_pressed:
                    selection=page_from_touch(*point)
                    if selection is not None: page=selection
                previous_pressed=point is not None
            else:
                # Preview mode writes one PNG rather than creating a privileged display service.
                destination=Path(config.get("png_path","screen-preview.png"))
                destination.parent.mkdir(parents=True,exist_ok=True)
                image.save(str(destination))
            if snapshot: return
            time.sleep(.15)
    finally:
        stop.set()
        if gateway: gateway.close()
        if lidar_thread: lidar_thread.join(timeout=1)
        if screen: screen.close()


def main(argv=None):
    parser=argparse.ArgumentParser(description="RoboDog Orange Pi gateway and portrait 240x320 touchscreen")
    parser.add_argument("--config",default="config.json")
    parser.add_argument("--demo",action="store_true",help="clearly marked fake sensor data")
    parser.add_argument("--snapshot",action="store_true",help="render one PNG and exit")
    parser.add_argument("--page",type=int,choices=(0,1,2),default=0,
                        help="initial screen: 0 ZE/RO, 1 IMU, 2 LiDAR")
    parser.add_argument("--capture-lidar",metavar="PATH",help="record raw COIN-D6 USB-UART bytes")
    parser.add_argument("--lidar-start",action="store_true",
                        help="send CSPC TOF AA55F00F command before raw capture")
    parser.add_argument("--probe-touch",action="store_true",help="print raw XPT2046 values while pressed")
    parser.add_argument("--seconds",type=int,default=5)
    parser.add_argument("--arm",action="store_true",help="request local operator ARM")
    args=parser.parse_args(argv)
    config=json.loads(Path(args.config).read_text(encoding="utf-8"))
    logging.basicConfig(level=logging.INFO,format="%(asctime)s %(levelname)s %(message)s")
    if args.arm:
        path=config.get("arm_socket") or str(Path.home()/".local"/"share"/"robodog"/"arm.sock")
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as conn:
            conn.connect(path); conn.sendall(b"ARM\n"); print(conn.recv(64).decode("ascii").strip())
        return
    if args.capture_lidar:
        lidar=config.get("lidar",{})
        if not lidar.get("usb_port"): raise SystemExit("set lidar.usb_port to verified USB adapter")
        print("Captured {} raw bytes".format(capture_raw(lidar["usb_port"],
              int(lidar.get("baud",230400)),args.capture_lidar,args.seconds,args.lidar_start)))
        return
    if args.probe_touch:
        screen=SpiScreen(config["screen"])
        try:
            while True:
                raw=screen.touch_raw()
                if raw is not None:
                    print("touch raw x={} y={}".format(*raw),flush=True)
                time.sleep(.1)
        except KeyboardInterrupt:
            pass
        finally:
            screen.close()
        return
    try:
        run(config,args.demo,args.snapshot,args.page)
    except KeyboardInterrupt:
        pass


if __name__=="__main__": main()
