"""Single-owner Mega serial link and optional HC-05 RFCOMM link.

Only fresh, CRC-checked remote frames reach the MCU. ARM is an explicit
local Unix-socket request; starting the service never arms the robot.
"""
import logging
import os
from pathlib import Path
import queue
import socket
import threading
import time
from typing import Optional
from . import protocol
from .state import Store

LOG=logging.getLogger(__name__)


class Gateway:
    def __init__(self, config: dict, store: Store):
        self.config=config
        self.store=store
        self.stop=threading.Event()
        self.arm_requests=queue.Queue(maxsize=1)
        self.telemetry_to_remote=queue.Queue(maxsize=4)
        self.last_forwarded=None
        self.last_accepted_remote_seq=None
        self.threads=[]
        self.arm_socket=config.get("arm_socket") or str(Path.home()/".local"/"share"/"robodog"/"arm.sock")

    def start(self):
        self.threads.append(threading.Thread(target=self._mega_loop,name="mega",daemon=True))
        self.threads.append(threading.Thread(target=self._remote_loop,name="remote",daemon=True))
        self.threads.append(threading.Thread(target=self._arm_loop,name="arm",daemon=True))
        for t in self.threads: t.start()

    def close(self):
        self.stop.set()
        for t in self.threads: t.join(timeout=2)

    def _mega_loop(self):
        import serial
        port=self.config.get("mega_port")
        if not port:
            self.store.update(mega_status="Mega: порт не настроен")
            return
        while not self.stop.is_set():
            try:
                with serial.Serial(port=port,baudrate=int(self.config.get("mega_baud",38400)),
                                   timeout=.03,write_timeout=.1,rtscts=False,dsrdtr=False) as ser:
                    self.store.update(mega_status="Mega: UART открыт")
                    self.last_forwarded=None
                    framer=protocol.Framer()
                    while not self.stop.is_set():
                        for line in framer.feed(ser.read(128)):
                            try:
                                body=protocol.decode(line)
                                now=time.monotonic()
                                if body.startswith("T,"):
                                    t=protocol.telemetry(body)
                                    self.store.update(telemetry=t,telemetry_at=now,mega_status="Mega: телеметрия")
                                    try: self.telemetry_to_remote.put_nowait(line)
                                    except queue.Full:
                                        try: self.telemetry_to_remote.get_nowait()
                                        except queue.Empty: pass
                                        self.telemetry_to_remote.put_nowait(line)
                                elif body.startswith("I,"):
                                    self.store.update(imu=protocol.imu(body),imu_at=now)
                                elif body=="X":
                                    self.store.update(mega_status="Mega просит выключение ОС")
                                    LOG.warning("Mega requested shutdown; operator must complete it")
                            except ValueError:
                                LOG.warning("Rejected invalid Mega frame")

                        snap=self.store.snapshot()
                        age=time.monotonic()-snap.control_at
                        # Do not regenerate a new seq from a cached movement command.
                        if age<.20 and snap.control is not None and snap.control.seq!=self.last_forwarded:
                            ser.write(snap.control_line)
                            self.last_forwarded=snap.control.seq
                        try:
                            self.arm_requests.get_nowait()
                        except queue.Empty:
                            pass
                        else:
                            snap=self.store.snapshot()
                            age=time.monotonic()-snap.control_at
                            t=snap.telemetry
                            t_age=time.monotonic()-snap.telemetry_at
                            if (age<.12 and snap.control is not None and snap.control.buttons==0 and
                                t is not None and t_age<.5 and t.state not in (4,5) and
                                not (t.flags & ((1<<7)|(1<<5))) and not (t.flags & (1<<3))):
                                ser.write(protocol.encode("A,{}".format(snap.control.seq)))
                                self.store.update(mega_status="ARM отправлен; ждём ответ Mega")
                            else:
                                self.store.update(last_error="ARM отклонён: нет свежей нейтрали/телеметрии")
            except (OSError,serial.SerialException,ValueError) as error:
                self.store.update(mega_status="Mega: связь потеряна",last_error=str(error))
                LOG.warning("Mega serial unavailable: %s",error)
                self.stop.wait(1)

    def _remote_loop(self):
        mac=self.config.get("hc05_mac")
        channel=self.config.get("hc05_rfcomm_channel")
        if not mac or not channel:
            self.store.update(remote_status="HC-05: адрес/канал не настроен")
            return
        if not hasattr(socket,"AF_BLUETOOTH"):
            self.store.update(remote_status="Bluetooth Classic недоступен в Python")
            return
        while not self.stop.is_set():
            connection=None
            try:
                connection=socket.socket(socket.AF_BLUETOOTH,socket.SOCK_STREAM,socket.BTPROTO_RFCOMM)
                connection.settimeout(5)
                connection.connect((mac,int(channel)))
                connection.settimeout(.05)
                self.store.update(remote_status="HC-05: соединён")
                self.last_accepted_remote_seq=None
                framer=protocol.Framer()
                while not self.stop.is_set():
                    try: raw=connection.recv(128)
                    except socket.timeout: raw=None
                    if raw==b"":
                        raise OSError("RFCOMM closed")
                    if raw:
                        for line in framer.feed(raw):
                            try:
                                c=protocol.control(protocol.decode(line))
                                if (self.last_accepted_remote_seq is not None and
                                    not protocol.sequence_newer(c.seq,self.last_accepted_remote_seq)):
                                    continue
                                self.last_accepted_remote_seq=c.seq
                                self.store.update(control=c,control_at=time.monotonic(),control_line=line)
                            except ValueError:
                                LOG.warning("Rejected invalid HC-05 frame")
                    elif raw is None and time.monotonic()-self.store.snapshot().control_at>1.0:
                        self.store.update(remote_status="HC-05: ждём команды")
                    try: outgoing=self.telemetry_to_remote.get_nowait()
                    except queue.Empty: outgoing=None
                    if outgoing:
                        connection.sendall(outgoing)
                    if self.store.snapshot().control_at and time.monotonic()-self.store.snapshot().control_at<.3:
                        self.store.update(remote_status="HC-05: свежие команды")
            except OSError as error:
                self.store.update(remote_status="HC-05: нет соединения",last_error=str(error))
                LOG.warning("HC-05 unavailable: %s",error)
                self.stop.wait(1)
            finally:
                # Old button states must never be replayed after reconnect.
                self.store.update(control=None,control_at=0.0,control_line=b"")
                if connection: connection.close()

    def _arm_loop(self):
        path=self.arm_socket
        if not path:
            return
        try:
            Path(path).parent.mkdir(mode=0o700,parents=True,exist_ok=True)
            if os.path.exists(path): os.unlink(path)
            with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as server:
                server.bind(path)
                os.chmod(path,0o600)
                server.listen(1)
                server.settimeout(.2)
                while not self.stop.is_set():
                    try: conn,_=server.accept()
                    except socket.timeout: continue
                    with conn:
                        conn.settimeout(1)
                        if conn.recv(16).strip()==b"ARM":
                            try: self.arm_requests.put_nowait(True); conn.sendall(b"REQUESTED\n")
                            except queue.Full: conn.sendall(b"BUSY\n")
                        else: conn.sendall(b"UNKNOWN\n")
        except OSError as error:
            self.store.update(last_error="ARM socket: {}".format(error))
        finally:
            try:
                if os.path.exists(path): os.unlink(path)
            except OSError: pass
